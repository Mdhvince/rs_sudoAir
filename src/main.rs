use std::collections::HashMap;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::page::Page;
use plotlib::style::{PointStyle, LineStyle, LineJoin};
// use nalgebra::Matrix3;
// use nalgebra::Vector3;



fn altitude_controller(dt: &f32,
                       state: &HashMap<&str, f32>,
                       state_des: &HashMap<&str, f32>,
                       params: &mut HashMap<&str, f32>,
                       quad: &HashMap<&str, f32>) -> f32{
    
    


    let position_error = state_des["z"] - state["z"];
    let velocity_error = state_des["z_dot"] - state["z_dot"];
    params.insert("integral_error_z", params["integral_error_z"] + position_error * dt);

    let acc_z = state_des["z_ddot"] 
                + params["kv_z"] * velocity_error 
                + params["kp_z"] * position_error
                + params["ki_z"] * params["integral_error_z"];
    
    let u1 = quad["mass"] * (acc_z + params["gravity"]);               // thrust u1 cmd
    u1
}

fn main() {
    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut targets: Vec<(f64, f64)> = Vec::new();

    let mut params: HashMap<&str, f32> = HashMap::new();
    params.insert("integral_error_z", 0.0);
    params.insert("gravity", 9.81);
    params.insert("kv_z", 15.0);
    params.insert("kp_z", 50.0);
    params.insert("ki_z", 8.0);

    let mut state: HashMap<&str, f32> = HashMap::new();
    state.insert("z", 0.0);
    state.insert("z_dot", 0.0);
    state.insert("z_ddot", 0.0);

    let mut state_des: HashMap<&str, f32> = HashMap::new();
    state_des.insert("z", 10.0);
    state_des.insert("z_dot", 0.01);
    state_des.insert("z_ddot", 0.0);

    let mut quad: HashMap<&str, f32> = HashMap::new();
    quad.insert("mass", 0.057);

    let dt: f32 = 0.1;
    let mg = quad["mass"] * params["gravity"];

    for _x in 0..1000{
        let u1 = altitude_controller(&dt, &state, &state_des, &mut params, &quad);
        let z_ddot = (u1 - mg) / quad["mass"];
        state.insert("z_ddot", z_ddot);

        points.push((_x as f64, state["z"] as f64));
        targets.push((_x as f64, state_des["z"] as f64));

        // advance_state(self, dt):
        state.insert("z_dot", state["z_dot"] + state["z_ddot"] * dt);
        state.insert("z", state["z"] + state["z_dot"] * dt);
    }
    

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
        .x_range(0., 1000.)
        .y_range(0., state_des["z"] as f64 + 10.0)
        .x_label("Iteration")
        .y_label("Altitude Z");

    Page::single(&canvas).save("pid_altitude.svg").unwrap();

}