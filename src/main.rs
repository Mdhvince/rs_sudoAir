use nalgebra::Vector6;
use plotpy::{Plot, Shapes};

mod quadrotor;

/*
position, velocity, acceleration : are coming from the trajectory planner
i.e:
    - state_des = [pos, vel, acc]
    - y_state_des = [pos, vel, acc]
    - x_state_des = [pos, vel, acc]
 */


fn main() {

    // trajectory planner
    let desired = quadrotor::State{
        z: 2.0, y: 0.0, phi: 0.0,
        z_dot: 0.1, y_dot: 0.1, phi_dot: 0.0,
        z_ddot: 0.0, y_ddot: 0.0, phi_ddot: 0.0
    };

    let actual = quadrotor::State{
        z: 0.0, y: 0.0, phi: 0.0,
        z_dot: 0.0, y_dot: 0.0, phi_dot: 0.0,
        z_ddot: 0.0, y_ddot: 0.0, phi_ddot: 0.0
    };

    // params
    let gravity: f32 = 9.81;
    let quad_mass = 0.027;      // 27g
    let min_thrust = 0.16;      // N
    let max_thrust = 0.56;      // N
    let I_x = 0.01;             // Moment of inertia around the x-axis

    // actual state: z, y, phi, z_dot, y_dot, phi_dot
    let mut state = Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    // for simulation and plotting
    let dt: f32 = 0.01;
    let iter = 5;
    let mut points: Vec<Vec<f32>> = Vec::new();
    let mut target_points: Vec<Vec<f32>> = Vec::new();

    // init
    let mut quadrotor = quadrotor::Quadrotor::new(quad_mass, gravity, I_x);

    for x in 0..iter {
        let mut u1 = quadrotor.control_altitude(&actual, &desired, dt);  // Collective thrust
        u1 = u1.clamp(min_thrust, max_thrust);

        let phi_cmd = quadrotor.control_lateral(u1, &actual, &desired);

        // inner loop
        let u2 = quadrotor.control_attitude(&actual, &desired, phi_cmd);  // Moment about the the x-axis
        
        // for simulation and plotting
        let phi = actual.phi;
        let z_ddot: f32 = gravity - u1 * phi.cos() / quad_mass;
        let y_ddot: f32 = u1 / quad_mass * phi.sin();
        let phi_ddot: f32 = u2 / I_x; 
        
        println!("{}", actual.z);
        update_state(dt, &actual, z_ddot, y_ddot, phi_ddot);
        println!("{}", actual.z);

        points.push(vec![x as f32, state[0]]);
        target_points.push(vec![x as f32, desired.z]);

        println!("{}", state);

    }

    // plot
    let mut shapes = Shapes::new();
    shapes.set_line_width(1.0).set_edge_color("blue").set_face_color("#ffffff");
    shapes.draw_polyline(&points, false);

    shapes.set_line_width(1.0).set_edge_color("red").set_face_color("#ffffff");
    shapes.draw_polyline(&target_points, false);
    
    let mut plot = Plot::new();
    plot.set_range(0.0, iter as f64, 0.0, 100.0)
        .add(&shapes);
    _ = plot.save("plot.png");


}

fn update_state(dt: f32, actual: &quadrotor::State, z_ddot: f32, y_ddot: f32, phi_ddot: f32) -> () {
    /* Update state (approximation - only for simulation)
    step 1 : find what is current the acceleration and angular acceleration of the system
    step 2 : reuse these accelerations to update the associated velocity and rate of change (integral)
    step 3 ; reuse velocity to update associated position (integral)
    */
    
    let state_dot: Vector6<f32> = Vector6::new(actual.z_dot, actual.y_dot, actual.phi_dot, z_ddot, y_ddot, phi_ddot);
    let actual_reformatted : Vector6<f32> = Vector6::new(actual.z_dot, actual.y_dot, actual.phi_dot, actual.z_ddot, actual.y_ddot, actual.phi_ddot);

    // new state after integration : pos_z, pos_y, angle_phi, vel_z, vel_y, ang_vel_phi
    let new_state = actual_reformatted + state_dot * dt;

    // update actual
    actual.z_dot = new_state[0];
    actual.y_dot = new_state[1];
    actual.phi_dot = new_state[2];
    actual.z_ddot = new_state[3];
    actual.y_ddot = new_state[4];
    actual.phi_ddot = new_state[5];
}
