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

    let acc_z = state_des["z_ddot"] 
                + params["kv_z"] * velocity_error 
                + params["kp_z"] * position_error
                + params["ki_z"] * params["integral_error_z"];
    
    let u1 = mass * (acc_z + params["gravity"]);               // thrust u1 cmd
    u1
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
        ("gravity", 9.81),
        ("kv_z", 40.0),
        ("kp_z", 130.0),
        ("ki_z", 0.0),
        ("dt", 0.1)
    ]);
    
    let initial_state: HashMap<&str, f32> = HashMap::from([
        ("z", 0.0), ("z_dot", 0.0), ("z_ddot", 0.0)
    ]);

    let state_des: HashMap<&str, f32> = HashMap::from([
        ("z", 10.0), ("z_dot", 0.01), ("z_ddot", 0.0)
    ]);
    
    
    let mut quadrotor = Quadrotor::new(0.18, initial_state);
    let m = quadrotor.mass;

    for _x in 0..1000{
        let u1 = altitude_controller(&quadrotor.state, &state_des, &mut params, &m);
        let u1 = u1.clamp(quadrotor.u_min, quadrotor.u_max);
        
        points.push((_x as f64, quadrotor.state["z"] as f64));
        if _x % 15 == 0 {
            targets.push((_x as f64, state_des["z"] as f64));
        }

        quadrotor.update_state(&params["dt"], &u1);
        println!("{}", quadrotor.state["z"]);
    }
    
    let (min_x, max_x, min_y, max_y) = (0.0, 1000.0, 0.0, state_des["z"] as f64 + 10.0);
    plot_trajectories(points, targets, min_x, max_x, min_y, max_y, "pid_altitude.svg");
}
