use nalgebra::Vector6;

pub struct State {
    pub z: f32, pub y: f32, pub phi: f32,
    pub z_dot: f32, pub y_dot: f32, pub phi_dot: f32,
    pub z_ddot: f32, pub y_ddot: f32, pub phi_ddot: f32
}


pub struct Quadrotor {
    mass: f32, gravity: f32, integral_error: f32, I_x: f32
}

impl Quadrotor {

    pub fn new(mass: f32, gravity: f32, I_x: f32) -> Quadrotor {
        Quadrotor {
            mass: mass, gravity: gravity, integral_error: 0.0, I_x: I_x
        }
    }

    pub fn control_altitude(&mut self, state: &Vector6<f32>, desired: &State, dt: f32) -> f32 {
        /*
        Inputs
            - state: Array of 6 float containing the current: pos_z, pos_y, angle_phi, vel_z, vel_y, angle_vel_phi
            - state_des: Array of desired state
        Output
            - Force/Thrust to apply to the rotors in Newton
        */
        let z_error: f32 = desired.z - state[0];
        let z_error_dot: f32 = desired.z_dot - state[3];
        self.integral_error += z_error * dt;

        let kp: f32 = 60.0;  // proportional gain
        let kd: f32 = 150.0;  // derivative gain
        let ki: f32 = 8.0;  // integral gain

        // acceleration command u1_bar
        let acc_cmd: f32 = kp * z_error + kd * z_error_dot  + ki * self.integral_error + desired.z_ddot;

        // collective thrust in Newton : u1 = m*a
        let phi_actual = state[2];
        let u1: f32 = self.mass * (self.gravity - acc_cmd) / phi_actual.cos();
        u1
    }

    pub fn control_lateral(&self, u1: f32, state: &Vector6<f32>, desired: &State) -> f32 {
        /* Controller for Computing a target roll angle */

        let y_error: f32 = desired.y - state[1];
        let y_error_dot: f32 = desired.y_dot - state[4];

        let kp: f32 = 60.0;
        let kd: f32 = 150.0;

        // acceleration_cmd on y
        let y_acc_cmd = kp * y_error + kd * y_error_dot + desired.y_ddot;

        let phi_cmd = (self.mass * y_acc_cmd / u1).clamp(-0.99, 0.99).asin();
        phi_cmd
    }


    pub fn control_attitude(&self, state: &Vector6<f32>, desired: &State, phi_cmd: f32) -> f32 {
        /* Controller for Computing the moment about the x-axis */

        let phi_error: f32 = phi_cmd - state[2];
        let phi_error_dot: f32 = desired.phi_dot - state[5];

        let kp: f32 = 60.0;
        let kd: f32 = 150.0;

        // u2_bar command
        let u2_bar = kp * phi_error + kd * phi_error_dot + desired.phi_ddot;
        let u2 = u2_bar * self.I_x;
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