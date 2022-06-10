use std::collections::HashMap;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::page::Page;
use plotlib::style::{PointStyle, LineStyle, LineJoin};

mod params;
use params::Params;
mod quadrotor;
use quadrotor::vehicle::Quadrotor;

fn plot_trajectories(points: Vec<(f64, f64)>,
                     targets: Vec<(f64, f64)>,
                     min_x: f64, max_x: f64,
                     min_y: f64, max_y: f64,
                     filename: &str){
    let plot_target: Plot = Plot::new(targets).point_style(
        PointStyle::new()
        .colour("#000000")
        .size(3.0)
    );

    let plot_state: Plot = Plot::new(points).line_style(
        LineStyle::new()
        .colour("#DD3355")
        .linejoin(LineJoin::Round)
    );

    

    let canvas = ContinuousView::new()
        .add(plot_target).add(plot_state)
        .x_range(min_x, max_x)
        .y_range(min_y, max_y)
        .x_label("Pos X")
        .y_label("Altitude Z");

    Page::single(&canvas).save(filename).unwrap();
}

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
    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut targets: Vec<(f64, f64)> = Vec::new();
    let mut params = Params::new();
    
    let initial_state: HashMap<&str, f32> = HashMap::from([
        ("z", 0.0), ("z_dot", 0.0), ("z_ddot", 0.0),
        ("y", 0.0), ("y_dot", 0.0), ("y_ddot", 0.0),
        ("x", 0.0), ("x_dot", 0.0), ("x_ddot", 0.0),
        ("phi", 0.0), ("theta", 0.0), ("psi", 0.0),
        ("p", 0.0), ("q", 0.0), ("r", 0.0)
    ]);

    let state_des: HashMap<&str, f32> = HashMap::from([
        ("x", 60.0), ("x_dot", 0.01), ("x_ddot", 0.0),
        ("y", 0.0), ("y_dot", 0.0), ("y_ddot", 0.0),
        ("z", 10.0), ("z_dot", 0.01), ("z_ddot", 0.0),
        ("psi", 0.0), ("psi_dot", 0.0)
    ]);
    
    
    let mut quadrotor = Quadrotor::new(0.18, initial_state);
    let deg_to_rad = std::f32::consts::PI / 180.0;
    let num_iter: usize = 1000;
    let inner_loop_iter: usize = num_iter/10;
    let dt_update = params.dt / inner_loop_iter as f32;
    targets.push((state_des["x"] as f64, state_des["z"] as f64));

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
        
        points.push((quadrotor.state["x"] as f64, quadrotor.state["z"] as f64));

        //inner-loop
        for _ in 0..inner_loop_iter{
            let u2 = attitude_controller(&quadrotor.state,
                                         &state_des,
                                         &params,
                                         &phi_c, &theta_c, &psi_c);
            
            quadrotor.update_state(&dt_update, &u1, &u2, &params);
        }
        println!("----------------------------------");
        println!("Z: {}", quadrotor.state["z"]);
        println!("Y: {}", quadrotor.state["y"]); 
        println!("X: {}", quadrotor.state["x"]);
    }
    
    let (min_x, max_x, min_y, max_y) = (
        0.0, state_des["x"] as f64 + 100.0, 0.0, state_des["z"] as f64 + 10.0);
    plot_trajectories(points, targets, min_x, max_x, min_y, max_y, "pid_3d.svg");
}
