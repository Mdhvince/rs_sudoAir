use nalgebra::Vector4;


pub struct Quadrotor {
    mass: f32, gravity: f32, integral_error: f32
}

impl Quadrotor {

    pub fn new(mass: f32, gravity: f32) -> Quadrotor {
        Quadrotor {
            mass: mass, gravity: gravity, integral_error: 0.0
        }
    }

    pub fn control_altitude(&mut self, state_z: &Vector4<f32>, z_state_des: &Vector4<f32>, z_ff: &f32, dt: f32) -> f32 {
        /*
        Inputs
            - state_z: Array of 3 float containing the current Z position, current velocity on Z, current yaw angle psi, and angular velocity psi_dot
            - z_state_des: Array of desired state
            - z_ff: feedforward acceleration
        Output
            - Force/Thrust to apply to the rotors in Newton
        */
        
        // tuning proportional and derivative gains
        // let damping_ratio = 0.7;  // damping_ratio: should remain between 0.7 and 1.0 - 1.0 means no overshoot but longer rise time
        // let wn: f32 = 0.9;  // natural frequency: should be large to have a small settling time
        // let time_constant = 1.0 / wn;
        // let rise_time = 1.57 * time_constant;

        
        // let kp: f32 = (1.0 / time_constant.powf(2.0)) * (1.0 + 2.0 * damping_ratio);  // proportional gain
        // let kd: f32 = (1.0 / time_constant) * (1.0 + 2.0 * damping_ratio);  // derivative gain
        // let ki: f32 = 1.0 / time_constant.powf(3.0);  // integral gain




        let pos_error: f32 = z_state_des[0] - state_z[0];
        let vel_error: f32 = z_state_des[1] - state_z[1];
        self.integral_error += pos_error * dt;

        let kp: f32 = 60.0;  // proportional gain
        let kd: f32 = 150.0;  // derivative gain
        let ki: f32 = 8.0;  // integral gain

        // acceleration
        let acc_cmd: f32 = kp * pos_error + kd * vel_error  + ki * self.integral_error + z_ff;  // required Z acceleration
        
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