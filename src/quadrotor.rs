pub mod vehicle {
    use std::collections::HashMap;

    pub struct Quadrotor<'a> {
        pub state: HashMap<&'a str, f32>,
        pub mass: f32,
    }

    impl<'a> Quadrotor<'a> {
        
        pub fn new(mass: f32, initial_state: HashMap<&'a str, f32>) -> Quadrotor<'a> {
            Quadrotor::<'a> { mass: mass, state: initial_state }
        }

        pub fn update_state(&mut self, dt: f32, z_ddot: f32) -> () {
            let _ = &self.state.insert("z_ddot", z_ddot);
            let _ = &self.state.insert("z_dot", &self.state["z_dot"] + &self.state["z_ddot"] * dt);
            let _ = &self.state.insert("z", &self.state["z"] + &self.state["z_dot"] * dt);
        }

    }
}