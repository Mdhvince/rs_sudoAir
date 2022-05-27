use nalgebra::Vector4;

struct Quadrotor {
    mass: f32, gravity: f32
}

impl Quadrotor {

    fn new(mass: f32, gravity: f32) -> Quadrotor {
        Quadrotor {
            mass: mass, gravity: gravity
        }
    }

    fn control_altitude(&self, state_z: &Vector4<f32>, z_state_des: &Vector4<f32>, z_ff: &f32) -> f32 {
        /*
        Inputs
            - state_z: Array of 3 float containing the current Z position, current velocity on Z, current yaw angle psi, and angular velocity psi_dot
            - z_state_des: Array of desired state
            - z_ff: feedforward acceleration
        Output
            - Force/Thrust to apply to the rotors in Newton
        */
        
        // tuning proportional and derivative gains
        let damping_ratio = 0.85;  // damping_ratio: should remain between 0.7 and 1.0 - 1.0 means no overshoot but longer rise time
        let wn: f32 = 100.0;  // natural frequency: should be large to have a small settling time
        // let rise_time = 1.57 * (1.0 / wn);
        
        let kp: f32 = wn.powf(2.0);
        let kd: f32 = 2.0 * damping_ratio * wn;




        let pos_error: f32 = z_state_des[0] - state_z[0];
        let vel_error: f32 = z_state_des[1] - state_z[1];

        // let kp: f32 = 1.0;  // proportional gain
        // let kd: f32 = 70.0;  // derivative gain

        // acceleration
        let acc_cmd: f32 = kp * pos_error + kd * vel_error + z_ff;  // required Z acceleration
        
        // direction of the acceleration : Mz = Iz * psi_ddot(angular velocity in rad/s2)
        // the moment about the z axis will cause an acceleration in the psi coordinate
        
        let fz_cmd: f32 = self.mass * (self.gravity - acc_cmd);  // Force or thrust in Newton : F = m*a
        fz_cmd
    }
}


/*
We can use trigonometry to decompose the thrust vector into multiple direction, so we know what amount
of force have to be applied on what axis in order to move somewhere.
 */


fn main() {
    let gravity: f32 = 9.81;
    // setting up a 27g quadrotor
    let quad_mass = 0.027;
    let quadrotor = Quadrotor::new(quad_mass, gravity);

    let min_thrust = 0.16;
    let max_thrust = 0.56;

    let mut z_state = Vector4::new(0.0, 0.0, 0.0, 0.0);                                             // position Z, velocity Z, yaw angle psi, angular velocity psi_dot
    let z_state_des = Vector4::new(50.0, 5.0, 0.0, 0.0);                                           
    let dt: f32 = 0.01;
    let z_ff: f32 = 0.0;

    for _ in 0..10000000 {
        let mut z_thrust = quadrotor.control_altitude(&z_state, &z_state_des, &z_ff);
        z_thrust = z_thrust.clamp(min_thrust, max_thrust);

        let net_thrust = quad_mass * gravity - z_thrust;
        let z_ddot: f32 = net_thrust / quad_mass;
        let psi_ddot: f32 = 0.0;

        z_state = update_state(dt, &z_state, z_ddot, psi_ddot);

        // println!("Force (N): {}", z_thrust);
        // println!("Acceleration: {}", z_ddot);
        println!("Z state: {}", z_state);
    }

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
