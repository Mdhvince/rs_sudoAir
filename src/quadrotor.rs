use nalgebra::Vector4;


pub struct Quadrotor {
    mass: f32, gravity: f32
}

impl Quadrotor {

    pub fn new(mass: f32, gravity: f32) -> Quadrotor {
        Quadrotor {
            mass: mass, gravity: gravity
        }
    }

    pub fn control_altitude(&self, state_z: &Vector4<f32>, z_state_des: &Vector4<f32>, z_ff: &f32) -> f32 {
        /*
        Inputs
            - state_z: Array of 3 float containing the current Z position, current velocity on Z, current yaw angle psi, and angular velocity psi_dot
            - z_state_des: Array of desired state
            - z_ff: feedforward acceleration
        Output
            - Force/Thrust to apply to the rotors in Newton
        */
        
        // // tuning proportional and derivative gains
        // let damping_ratio = 0.85;  // damping_ratio: should remain between 0.7 and 1.0 - 1.0 means no overshoot but longer rise time
        // let wn: f32 = 100.0;  // natural frequency: should be large to have a small settling time
        // // let rise_time = 1.57 * (1.0 / wn);
        
        // let kp: f32 = wn.powf(2.0);
        // let kd: f32 = 2.0 * damping_ratio * wn;




        let pos_error: f32 = z_state_des[0] - state_z[0];
        let vel_error: f32 = z_state_des[1] - state_z[1];

        let kp: f32 = 60.0;  // proportional gain
        let kd: f32 = 80.0;  // derivative gain

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