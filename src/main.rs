use std::collections::HashMap;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::page::Page;
use plotlib::style::{PointStyle, LineStyle, LineJoin};

mod quadrotor;
use quadrotor::vehicle::Quadrotor;

// use nalgebra::Matrix3;
// use nalgebra::Vector3;

// from sim in matlab
// gravity: 9.8100
//           mass: 0.1800
//     arm_length: 0.0860
//          u_min: 0
//          u_max: 2.1190



fn altitude_controller(state: &HashMap<&str, f32>,
                       state_des: &HashMap<&str, f32>,
                       params: &mut HashMap<&str, f32>,
                       mass: &f32) -> f32{
    
    let position_error = state_des["z"] - state["z"];
    let velocity_error = state_des["z_dot"] - state["z_dot"];
    params.insert("integral_error_z", params["integral_error_z"] + position_error * params["dt"]);

    let z_ddot_c = state_des["z_ddot"] 
                   + params["kv_z"] * velocity_error 
                   + params["kp_z"] * position_error
                   + params["ki_z"] * params["integral_error_z"];
    
    let u1 = mass * (params["gravity"] + z_ddot_c);               // thrust u1 cmd
    u1
}

fn position_controller(state: &HashMap<&str, f32>,
    state_des: &HashMap<&str, f32>,
    params: &mut HashMap<&str, f32>) -> (f32, f32, f32){

    let x_error = state_des["x"] - state["x"];
    let x_dot_error = state_des["x_dot"] - state["x_dot"];
    params.insert("integral_error_x", params["integral_error_x"] + x_error * params["dt"]);
    let x_ddot_c = state_des["x_ddot"]
                    + params["kv_xy"] * x_dot_error 
                    + params["kp_xy"] * x_error
                    + params["ki_xy"] * params["integral_error_x"];
    
    let y_error = state_des["y"] - state["y"];
    let y_dot_error = state_des["y_dot"] - state["y_dot"];
    params.insert("integral_error_y", params["integral_error_y"] + y_error * params["dt"]);
    let y_ddot_c = state_des["y_ddot"]
                    + params["kv_xy"] * y_dot_error 
                    + params["kp_xy"] * y_error
                    + params["ki_xy"] * params["integral_error_y"];

    let phi_c = (1.0 / params["gravity"]) * (x_ddot_c * state_des["psi"].sin() - y_ddot_c * state_des["psi"].cos());
    let theta_c = (1.0 / params["gravity"]) * (x_ddot_c * state_des["psi"].cos() + y_ddot_c * state_des["psi"].sin());
    let psi_c = state_des["psi"];

    (phi_c, theta_c, psi_c)
}

fn attitude_controller(state: &HashMap<&str, f32>,
                       state_des: &HashMap<&str, f32>,
                       params: &HashMap<&str, f32>,
                       phi_c: &f32, theta_c: &f32, psi_c: &f32) -> (f32, f32, f32){
    
    let p_c = 0.0;
    let q_c = 0.0;
    let r_c = state_des["psi_dot"];

    let u2_roll = params["kp_angle"] * (phi_c - state["phi"]) + params["kv_angle"] * (p_c - state["p"]);
    let u2_pitch = params["kp_angle"] * (theta_c - state["theta"]) + params["kv_angle"] * (q_c - state["q"]);
    let u2_yaw = params["kp_angle"] * (psi_c - state["psi"]) + params["kv_angle"] * (r_c - state["r"]);
    
    (u2_roll, u2_pitch, u2_yaw) // u2
}

fn motor_controller(quadrotor: &Quadrotor, u1: f32, u2: (f32, f32, f32)) -> () {
    let M = (quadrotor.ixx * u2.0, quadrotor.iyy * u2.1, quadrotor.izz * u2.2);
}


fn plot_trajectories(points: Vec<(f64, f64)>,
                     targets: Vec<(f64, f64)>,
                     min_x: f64, max_x: f64,
                     min_y: f64, max_y: f64,
                     filename: &str){
    let plot_target: Plot = Plot::new(targets).point_style(
        PointStyle::new()
        .colour("#000000")
        .size(2.0)
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
        .x_label("Iteration")
        .y_label("Altitude Z");

    Page::single(&canvas).save(filename).unwrap();
}

/* ----------------------------------------------------- */

fn main() {
    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut targets: Vec<(f64, f64)> = Vec::new();
    
    let mut params: HashMap<&str, f32> = HashMap::from([
        ("integral_error_z", 0.0),
        ("integral_error_x", 0.0),
        ("integral_error_y", 0.0),
        ("gravity", 9.81),
        ("kv_z", 40.0),
        ("kp_z", 130.0),
        ("ki_z", 0.0),
        ("kv_xy", 40.0),
        ("kp_xy", 130.0),
        ("ki_xy", 0.0),
        ("kv_angle", 1.0),
        ("kp_angle", 5.0),
        ("ki_angle", 0.0),
        ("dt", 0.1)
    ]);
    
    let initial_state: HashMap<&str, f32> = HashMap::from([
        ("z", 0.0), ("z_dot", 0.0), ("z_ddot", 0.0),
        ("y", 0.0), ("y_dot", 0.0), ("y_ddot", 0.0),
        ("x", 0.0), ("x_dot", 0.0), ("x_ddot", 0.0),
        ("phi", 0.0), ("theta", 0.0), ("psi", 0.0),
        ("p", 0.0), ("q", 0.0), ("r", 0.0)
    ]);

    let state_des: HashMap<&str, f32> = HashMap::from([
        ("z", 10.0), ("z_dot", 0.01), ("z_ddot", 0.0),
        ("y", 100.0), ("y_dot", 0.1), ("y_ddot", 0.0),
        ("x", 0.0), ("x_dot", 0.0), ("x_ddot", 0.0),
        ("psi", 0.0), ("psi_dot", 0.1)
    ]);
    
    
    let mut quadrotor = Quadrotor::new(0.18, initial_state);
    let m = quadrotor.mass;

    for _x in 0..1000{
        let u1 = altitude_controller(&quadrotor.state, &state_des, &mut params, &m);
        let u1 = u1.clamp(quadrotor.u_min, quadrotor.u_max);
        let (phi_c, theta_c, psi_c) = position_controller(&quadrotor.state, &state_des, &mut params);  // we will need to clamp them too
        
        //inner-loop
        let u2 = attitude_controller(&quadrotor.state, &state_des, &params, &phi_c, &theta_c, &psi_c);
        motor_controller(&quadrotor, u1, u2);  // will be used to send command to the motors (not completed yet)
        

        /* update state
        x_ddot = g * (theta * cos(psi) + phi * sin(psi))
        y_ddot = g * (theta * sin(psi) - phi * cos(psi))
        z_ddot = u1 / mass - 9.81;
        */

        
        
        points.push((_x as f64, quadrotor.state["z"] as f64));
        if _x % 15 == 0 {
            targets.push((_x as f64, state_des["z"] as f64));
        }

        quadrotor.update_state(&params["dt"], &u1);
    }
    
    let (min_x, max_x, min_y, max_y) = (0.0, 1000.0, 0.0, state_des["z"] as f64 + 10.0);
    plot_trajectories(points, targets, min_x, max_x, min_y, max_y, "pid_altitude.svg");
}
