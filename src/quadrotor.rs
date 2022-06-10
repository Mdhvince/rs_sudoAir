pub mod vehicle {
    use std::collections::HashMap;

    pub struct Quadrotor<'a> {
        pub state: HashMap<&'a str, f32>,
        pub mass: f32,
        pub u_min: f32,
        pub u_max: f32,
        pub ixx: f32,
        pub iyy: f32,
        pub izz: f32
    }

    impl<'a> Quadrotor<'a> {
        
        pub fn new(mass: f32, initial_state: HashMap<&'a str, f32>) -> Quadrotor<'a> {
            Quadrotor::<'a> {
                mass: mass,
                state: initial_state,
                u_min: 0.0,
                u_max: 3.5316,
                ixx: 1.0,
                iyy: 1.0,
                izz: 1.0
            }
        }

        pub fn update_state(&mut self,
                            u1: &f32,
                            u2: &(f32, f32, f32),
                            params: &HashMap<&'a str, f32>) -> () {
            
            let z_ddot = u1 / &self.mass - params["gravity"];
            let x_ddot = params["gravity"]
                         * (&self.state["theta"] * &self.state["psi"].cos() + &self.state["phi"] * &self.state["psi"].sin());
            let y_ddot = params["gravity"]
                         * (&self.state["theta"] * &self.state["psi"].sin() - &self.state["phi"] * &self.state["psi"].cos());
            
            let _ = &self.state.insert("x_ddot", x_ddot);
            let _ = &self.state.insert("y_ddot", y_ddot);
            let _ = &self.state.insert("z_ddot", z_ddot);
            let _ = &self.state.insert("x_dot", &self.state["x_dot"] + &self.state["x_ddot"] * params["dt"]);
            let _ = &self.state.insert("y_dot", &self.state["y_dot"] + &self.state["y_ddot"] * params["dt"]);
            let _ = &self.state.insert("z_dot", &self.state["z_dot"] + &self.state["z_ddot"] * params["dt"]);
            let _ = &self.state.insert("x", &self.state["x"] + &self.state["x_dot"] * params["dt"]);
            let _ = &self.state.insert("y", &self.state["y"] + &self.state["y_dot"] * params["dt"]);
            let _ = &self.state.insert("z", &self.state["z"] + &self.state["z_dot"] * params["dt"]);

            let p_dot = (u2.0 - &self.state["r"] * &self.state["q"] * (&self.izz - &self.iyy)) / &self.ixx;
            let q_dot = (u2.1 - &self.state["r"] * &self.state["p"] * (&self.ixx - &self.izz)) / &self.iyy;
            let r_dot = (u2.2 - &self.state["q"] * &self.state["p"] * (&self.iyy - &self.ixx)) / &self.izz;

            let _ = &self.state.insert("p", &self.state["p"] + p_dot * params["dt"]);
            let _ = &self.state.insert("q", &self.state["q"] + q_dot * params["dt"]);
            let _ = &self.state.insert("r", &self.state["r"] + r_dot * params["dt"]);
            
            // euler derivatives
            let c_phi_c_theta_square = &self.state["phi"].cos() * &self.state["theta"].cos().powf(2.0);
            let c_phi_s_theta_square = &self.state["phi"].cos() * &self.state["theta"].sin().powf(2.0);

            let phi_dot = (&self.state["p"] * &self.state["theta"].cos())
                          / (&self.state["theta"].cos().powf(2.0) + &self.state["theta"].sin().powf(2.0)) 
                          + (&self.state["r"] * &self.state["theta"].sin())
                          / (&self.state["theta"].cos().powf(2.0) + &self.state["theta"].sin().powf(2.0));

            let theta_dot = &self.state["q"]
                            - (&self.state["r"] * &self.state["theta"].cos() * &self.state["phi"].sin())
                            / (c_phi_c_theta_square + c_phi_s_theta_square)
                            + (&self.state["p"] * &self.state["phi"].sin() * &self.state["theta"].sin())
                            / (c_phi_c_theta_square + c_phi_s_theta_square);
                            
            let psi_dot = (&self.state["r"] * &self.state["theta"].cos())
                          / (c_phi_c_theta_square + c_phi_s_theta_square)
                          - (&self.state["p"] * &self.state["theta"].sin())
                          / (c_phi_c_theta_square + c_phi_s_theta_square);
            
            let _ = &self.state.insert("phi_dot", phi_dot);
            let _ = &self.state.insert("theta_dot", theta_dot);
            let _ = &self.state.insert("psi_dot", psi_dot);
            let _ = &self.state.insert("phi", &self.state["phi"] + &self.state["phi_dot"] * params["dt"]);
            let _ = &self.state.insert("theta", &self.state["theta"] + &self.state["theta_dot"] * params["dt"]);
            let _ = &self.state.insert("psi", &self.state["psi"] + &self.state["psi_dot"] * params["dt"]);
            
        }

    }
}