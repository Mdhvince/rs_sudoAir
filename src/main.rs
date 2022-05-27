use nalgebra::Vector4;
use plotpy::{Plot, Shapes};

mod quadrotor;


fn main() {
    let gravity: f32 = 9.81;
    // setting up a 27g quadrotor
    let quad_mass = 0.027;
    let quadrotor = quadrotor::Quadrotor::new(quad_mass, gravity);

    let min_thrust = 0.16;
    let max_thrust = 0.56;

    let mut z_state = Vector4::new(0.0, 0.0, 0.0, 0.0);                                             // position Z, velocity Z, yaw angle psi, angular velocity psi_dot
    let z_state_des = Vector4::new(50.0, 5.0, 0.0, 0.0);                                           
    let dt: f32 = 0.01;
    let z_ff: f32 = 0.0;
    let iter = 10000;

    let mut points: Vec<Vec<f32>> = Vec::new();

    for x in 0..iter {
        let mut z_thrust = quadrotor.control_altitude(&z_state, &z_state_des, &z_ff);
        z_thrust = z_thrust.clamp(min_thrust, max_thrust);

        let net_thrust = quad_mass * gravity - z_thrust;
        let z_ddot: f32 = net_thrust / quad_mass;
        let psi_ddot: f32 = 0.0;

        z_state = update_state(dt, &z_state, z_ddot, psi_ddot);

        // println!("Force (N): {}", z_thrust);
        // println!("Acceleration: {}", z_ddot);
        // println!("Z state: {}", z_state);

        points.push(vec![x as f32, z_state[0]]);
    }

    // plot
    let mut shapes = Shapes::new();
    shapes.set_line_width(1.0).set_edge_color("blue").set_face_color("#ffffff");
    shapes.draw_polyline(&points, false);
    
    let mut plot = Plot::new();
    plot.set_range(0.0, iter as f64, 0.0, 100.0)
        .add(&shapes);
    _ = plot.save("altitude_pd_controller.png");


}

fn update_state(dt: f32, z_state: &Vector4<f32>, z_ddot: f32, psi_ddot: f32) -> Vector4<f32> {
    /* Update state (approximation - only for simulation)
    step 1 : find what is current the acceleration and angular acceleration of the system
    step 2 : reuse these accelerations to update the associated velocity and rate of change (integral)
    step 3 ; reuse velocity to update associated position (integral)
    */
    let z_dot_state: Vector4<f32> = Vector4::new(z_state[1], z_ddot, z_state[3], psi_ddot);
    let delta = z_dot_state * dt;

    let new_state = z_state + delta;
    new_state
}





// // 2
    // let delta_z_dot: f32 = z_ddot * dt;
    // let delta_psi_dot: f32 = psi_ddot * dt;
    // z_state[1] += delta_z_dot;  // velocity
    // z_state[3] += delta_psi_dot; // angular velocity

    // // 3
    // let delta_z: f32 = z_state[1] * dt;
    // let delta_psi: f32 = z_state[3] * dt;
    // z_state[0] += delta_z;  // position
    // z_state[2] += delta_psi;  // yaw angle
