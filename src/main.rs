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
        z: 10.0, y: 20.0, phi: 0.0,
        z_dot: 0.1, y_dot: 0.1, phi_dot: 0.0,
        z_ddot: 0.0, y_ddot: 0.0, phi_ddot: 0.0
    };

    // params
    let gravity: f32 = 9.81;
    let quad_mass = 0.027;  // 27g
    let min_thrust = 0.16;  // N
    let max_thrust = 0.56;  // N
    let I_x = 0.01;  // Moment of inertia around the x-axis

    // actual state: z, y, phi, z_dot, y_dot, phi_dot
    let mut state = Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    // for simulation and plotting
    let dt: f32 = 0.01;
    let iter = 10000;
    let mut points: Vec<Vec<f32>> = Vec::new();
    let mut target_points: Vec<Vec<f32>> = Vec::new();

    // init
    let mut quadrotor = quadrotor::Quadrotor::new(quad_mass, gravity, I_x);

    for _ in 0..iter {
        let mut u1 = quadrotor.control_altitude(&state, &desired, dt);  // Collective thrust
        u1 = u1.clamp(min_thrust, max_thrust);

        let phi_cmd = quadrotor.control_lateral(u1, &state, &desired);

        // inner loop
        for _ in 0..10 {
            let u2 = quadrotor.control_attitude(&state, &desired, phi_cmd);  // Moment about the the x-axis
            
            // for simulation and plotting
            let phi = state[2];
            let z_ddot: f32 = gravity - u1 * phi.cos() / quad_mass;
            let y_ddot: f32 = u1 / quad_mass * phi.sin();
            let phi_ddot: f32 = u2 / I_x;        
            
            state = update_state(dt, &state, z_ddot, y_ddot, phi_ddot);
            points.push(vec![state[1], state[0]]);
            target_points.push(vec![desired.y, desired.z]);
        }

        println!("{}", state);

    }

    // // plot
    // let mut shapes = Shapes::new();
    // shapes.set_line_width(1.0).set_edge_color("blue").set_face_color("#ffffff");
    // shapes.draw_polyline(&points, false);

    // shapes.set_line_width(1.0).set_edge_color("red").set_face_color("#ffffff");
    // shapes.draw_polyline(&target_points, false);
    
    // let mut plot = Plot::new();
    // plot.set_range(0.0, 100.0, 0.0, 100.0)
    //     .add(&shapes);
    // _ = plot.save("altitude_pd_controller.png");


}

fn update_state(dt: f32, state: &Vector6<f32>, z_ddot: f32, y_ddot: f32, phi_ddot: f32) -> Vector6<f32> {
    /* Update state (approximation - only for simulation)
    step 1 : find what is current the acceleration and angular acceleration of the system
    step 2 : reuse these accelerations to update the associated velocity and rate of change (integral)
    step 3 ; reuse velocity to update associated position (integral)
    */
    
    // vel_z, vel_y, angle_vel_phi, acc_z, acc_y, acc_phi
    let state_dot: Vector6<f32> = Vector6::new(state[3], state[4], state[5], z_ddot, y_ddot, phi_ddot);  

    // new state after integration : pos_z, pos_y, angle_phi, vel_z, vel_y, ang_vel_phi
    let new_state = state + state_dot * dt;
    new_state
}
