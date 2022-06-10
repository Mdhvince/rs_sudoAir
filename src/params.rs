// pub mod parameters{
    pub struct Params{
        pub integral_error_z: f32, pub integral_error_x: f32, pub integral_error_y: f32,
        pub kv_z: f32, pub kp_z: f32, pub ki_z: f32,
        pub kv_xy: f32, pub kp_xy: f32, pub ki_xy: f32,
        pub kv_angle: f32, pub kp_angle: f32, pub ki_angle: f32,
        pub dt: f32,
        pub gravity: f32
    }

    impl Params {
        pub fn new() -> Params {
            Params {
                integral_error_z: 0.0, integral_error_x: 0.0, integral_error_y: 0.0,
                
                kv_z: 60.0, kp_z: 40.0, ki_z: 0.0,
                kv_xy: 90.0, kp_xy: 22.0, ki_xy: 0.0,
                kv_angle: 15.0, kp_angle: 15.0, ki_angle: 0.0,

                dt: 0.1,
                gravity: 9.81
            }
        }
    }
// }