pub mod vehicle {
    use std::collections::HashMap;

    pub struct Quadrotor<'a> {
        pub state: HashMap<&'a str, f32>,
        pub mass: f32,
        pub u_min: f32,
        pub u_max: f32
    }

    impl<'a> Quadrotor<'a> {
        
        pub fn new(mass: f32, initial_state: HashMap<&'a str, f32>) -> Quadrotor<'a> {
            Quadrotor::<'a> {
                mass: mass,
                state: initial_state,
                u_min: 0.0,
                u_max: 1.2 * mass * 9.81
            }
        }

        pub fn update_state(&mut self, dt: &f32, u1: &f32) -> () {
            let z_ddot = u1 / &self.mass - 9.81;
            let _ = &self.state.insert("z_ddot", z_ddot);
            let _ = &self.state.insert("z_dot", &self.state["z_dot"] + &self.state["z_ddot"] * dt);
            let _ = &self.state.insert("z", &self.state["z"] + &self.state["z_dot"] * dt);
        }

    }
}