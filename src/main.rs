use std::collections::HashMap;
use gnuplot::*;

mod params;
use params::Params;
mod quadrotor;
use quadrotor::vehicle::Quadrotor;


fn altitude_controller(state: &HashMap<&str, f32>,
                       state_des: &HashMap<&str, f32>,
                       params: &mut Params,
                       mass: &f32) -> f32{
    
    let position_error = state_des["z"] - state["z"];
    let velocity_error = state_des["z_dot"] - state["z_dot"];
    params.integral_error_z += position_error * params.dt;

    let z_ddot_c = state_des["z_ddot"] 
                   + params.kv_z * velocity_error 
                   + params.kp_z * position_error
                   + params.ki_z * params.integral_error_z;
    
    let u1 = mass * (params.gravity + z_ddot_c);
    u1
}

fn position_controller(state: &HashMap<&str, f32>,
    state_des: &HashMap<&str, f32>,
    params: &mut Params) -> (f32, f32, f32) {

    let x_error = state_des["x"] - state["x"];
    let x_dot_error = state_des["x_dot"] - state["x_dot"];
    params.integral_error_x += x_error * params.dt;

    let x_ddot_c = state_des["x_ddot"]
                    + params.kv_xy * x_dot_error 
                    + params.kp_xy * x_error
                    + params.ki_xy * params.integral_error_x;
    
    let y_error = state_des["y"] - state["y"];
    let y_dot_error = state_des["y_dot"] - state["y_dot"];
    params.integral_error_y += y_error * params.dt;

    let y_ddot_c = state_des["y_ddot"]
                    + params.kv_xy * y_dot_error 
                    + params.kp_xy * y_error
                    + params.ki_xy * params.integral_error_y;
    
    let phi_c = (1.0 / params.gravity) * (x_ddot_c * state_des["psi"].sin() - y_ddot_c * state_des["psi"].cos());
    let theta_c = (1.0 / params.gravity) * (x_ddot_c * state_des["psi"].cos() + y_ddot_c * state_des["psi"].sin());
    let psi_c = state_des["psi"];

    (phi_c, theta_c, psi_c)
}

fn attitude_controller(state: &HashMap<&str, f32>,
                       state_des: &HashMap<&str, f32>,
                       params: &Params,
                       phi_c: &f32, theta_c: &f32, psi_c: &f32) -> (f32, f32, f32){
    
    let p_c = 0.0;
    let q_c = 0.0;
    let r_c = state_des["psi_dot"];

    let u2_roll = params.kp_angle * (phi_c - state["phi"]) + params.kv_angle * (p_c - state["p"]);
    let u2_pitch = params.kp_angle * (theta_c - state["theta"]) + params.kv_angle * (q_c - state["q"]);
    let u2_yaw = params.kp_angle * (psi_c - state["psi"]) + params.kv_angle * (r_c - state["r"]);
    
    (u2_roll, u2_pitch, u2_yaw) // u2
}

// fn motor_controller(quadrotor: &Quadrotor, u1: f32, u2: (f32, f32, f32)) -> () {
//     let M = (quadrotor.ixx * u2.0, quadrotor.iyy * u2.1, quadrotor.izz * u2.2);
// }

/* ----------------------------------------------------- */

fn main() {
    // let mut points: Vec<(f64, f64)> = Vec::new();
    // let mut targets: Vec<(f64, f64)> = Vec::new();
    let mut points_x: Vec<f64> = Vec::new();
    let mut points_y: Vec<f64> = Vec::new();
    let mut points_z: Vec<f64> = Vec::new();
    let mut target_x: Vec<f64> = Vec::new();
    let mut target_y: Vec<f64> = Vec::new();
    let mut target_z: Vec<f64> = Vec::new();

    let mut params = Params::new();
    
    let initial_state: HashMap<&str, f32> = HashMap::from([
        ("z", 0.0), ("z_dot", 0.0), ("z_ddot", 0.0),
        ("y", 0.0), ("y_dot", 0.0), ("y_ddot", 0.0),
        ("x", 0.0), ("x_dot", 0.0), ("x_ddot", 0.0),
        ("phi", 0.0), ("theta", 0.0), ("psi", 0.0),
        ("p", 0.0), ("q", 0.0), ("r", 0.0)
    ]);

    let state_des: HashMap<&str, f32> = HashMap::from([
        ("x", 50.0), ("x_dot", 0.01), ("x_ddot", 0.0),
        ("y", 30.0), ("y_dot", 0.01), ("y_ddot", 0.0),
        ("z", 60.0), ("z_dot", 0.01), ("z_ddot", 0.0),
        ("psi", 0.0), ("psi_dot", 0.0)
    ]);
    
    
    let mut quadrotor = Quadrotor::new(0.18, initial_state);
    let deg_to_rad = std::f32::consts::PI / 180.0;
    let num_iter: usize = 1000;
    let inner_loop_iter: usize = num_iter/10;
    let dt_update = params.dt / inner_loop_iter as f32;
    
    // targets.push((state_des["x"] as f64, state_des["z"] as f64));
    target_x.push(state_des["x"] as f64);
    target_y.push(state_des["y"] as f64);
    target_z.push(state_des["z"] as f64);

    for _x in 0..num_iter{
        let u1 = altitude_controller(&quadrotor.state,
                                     &state_des,
                                     &mut params,
                                     &quadrotor.mass);
        let u1 = u1.clamp(quadrotor.u_min, quadrotor.u_max);
        let (mut phi_c, mut theta_c, mut psi_c) = position_controller(&quadrotor.state,
                                                                      &state_des,
                                                                      &mut params);
        
        phi_c = phi_c.clamp(-30.0 * deg_to_rad, 30.0 * deg_to_rad);
        theta_c = theta_c.clamp(-30.0 * deg_to_rad, 30.0 * deg_to_rad);
        psi_c = psi_c.clamp(-360.0 * deg_to_rad, 360.0 * deg_to_rad);
        
        // points.push((quadrotor.state["x"] as f64, quadrotor.state["z"] as f64));
        points_x.push(quadrotor.state["x"] as f64);
        points_y.push(quadrotor.state["y"] as f64);
        points_z.push(quadrotor.state["z"] as f64);

        //inner-loop
        for _ in 0..inner_loop_iter{
            let u2 = attitude_controller(&quadrotor.state,
                                         &state_des,
                                         &params,
                                         &phi_c, &theta_c, &psi_c);
            
            quadrotor.update_state(&dt_update, &u1, &u2, &params);
        }
        // println!("----------------------------------");
        // println!("Z: {}", quadrotor.state["z"]);
        // println!("Y: {}", quadrotor.state["y"]); 
        // println!("X: {}", quadrotor.state["x"]);
    }
    
    
    let mut fg = Figure::new();
    fg.axes3d()
    .set_x_label("Pos X {}", &[])
    .set_y_label("Pos Y", &[])
    .set_z_label("Alt Z", &[])
    .lines_points(&points_x, &points_y, &points_z,&[PointSymbol('o'), PointSize(1.0), Caption("Actual"), Color("blue")])
    .points(&target_x, &target_y, &target_z, &[PointSymbol('*'), PointSize(5.0), Caption("Target"), Color("red")]);
    // fg.set_scale(1.0, 1.0);
    fg.show().unwrap();
}
