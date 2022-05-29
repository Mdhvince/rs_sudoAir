
#[derive(Debug)]
pub struct State {
    pub x: f32, pub y: f32, pub z: f32,                         // locations
    pub phi: f32, pub theta: f32, pub psi: f32,                 // orientations
    pub x_dot: f32, pub y_dot: f32, pub z_dot: f32,             // velocities
    pub phi_dot: f32, pub theta_dot: f32, pub psi_dot: f32,     // angular velocities in the world frame
    pub p: f32, pub q: f32, pub r: f32,                         // angular velocities in the body frame
    pub x_ddot: f32, pub y_ddot: f32, pub z_ddot: f32,          // accelerations
    pub phi_ddot: f32, pub theta_ddot: f32, pub psi_ddot: f32   // angular accelerations
}



#[derive(Debug)]
pub struct Quadrotor {
    mass: f32, gravity: f32, integral_error: f32, i_x: f32
}

impl Quadrotor {

    pub fn new(mass: f32, gravity: f32, i_x: f32) -> Quadrotor {
        Quadrotor {
            mass: mass, gravity: gravity, integral_error: 0.0, i_x: i_x
        }
    }

    pub fn control_altitude(&mut self, actual: &State, desired: &State, dt: f32) -> f32 {
        /*
        Inputs
            - state: Array of 6 float containing the current: pos_z, pos_y, angle_phi, vel_z, vel_y, angle_vel_phi
            - state_des: Array of desired state
        Output
            - Force/Thrust to apply to the rotors in Newton
        */
        let z_error: f32 = desired.z - actual.z;
        let z_error_dot: f32 = desired.z_dot - actual.z_dot;
        self.integral_error += z_error * dt;

        let kp: f32 = 50.0;  // proportional gain
        let kd: f32 = 150.0;  // derivative gain
        let ki: f32 = 8.0;  // integral gain

        // acceleration command u1_bar
        let acc_cmd: f32 = kp * z_error + kd * z_error_dot  + ki * self.integral_error + desired.z_ddot;

        // collective thrust in Newton
        let u1: f32 = self.mass * (acc_cmd - self.gravity) / actual.phi.cos();
        u1
    }

    pub fn control_lateral(&self, u1: f32, actual: &State, desired: &State) -> f32 {
        /* Controller for Computing a target roll angle */

        let y_error: f32 = desired.y - actual.y;
        let y_error_dot: f32 = desired.y_dot - actual.y_dot;

        let kp: f32 = 180.0;
        let kd: f32 = 5.0;

        // acceleration_cmd on y
        let y_acc_cmd = kp * y_error + kd * y_error_dot + desired.y_ddot;

        let phi_cmd = (self.mass * y_acc_cmd / u1).clamp(-0.99, 0.99).asin();
        phi_cmd
    }


    pub fn control_attitude(&self, actual: &State, desired: &State, phi_cmd: f32) -> f32 {
        /* Controller for Computing the moment about the x-axis */

        let phi_error: f32 = phi_cmd - actual.phi;
        let phi_error_dot: f32 = desired.phi_dot - actual.phi_dot;

        let kp: f32 = 40.0;
        let kd: f32 = 150.0;

        // u2_bar command
        let u2_bar = kp * phi_error + kd * phi_error_dot + desired.phi_ddot;
        let u2 = u2_bar * self.i_x;
        u2
    }
}











/*
We can use trigonometry to decompose the thrust vector into multiple direction, so we know what amount
of force have to be applied on what axis in order to move somewhere.
 */

// tuning proportional and derivative gains
// let damping_ratio = 0.7;  // damping_ratio: should remain between 0.7 and 1.0 - 1.0 means no overshoot but longer rise time
// let wn: f32 = 0.9;  // natural frequency: should be large to have a small settling time
// let time_constant = 1.0 / wn;
// let rise_time = 1.57 * time_constant;


// let kp: f32 = (1.0 / time_constant.powf(2.0)) * (1.0 + 2.0 * damping_ratio);  // proportional gain
// let kd: f32 = (1.0 / time_constant) * (1.0 + 2.0 * damping_ratio);  // derivative gain
// let ki: f32 = 1.0 / time_constant.powf(3.0);  // integral gain