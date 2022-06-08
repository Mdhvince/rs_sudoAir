use std::collections::HashMap;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::page::Page;
use plotlib::style::{PointStyle, LineStyle, LineJoin};

mod quadrotor;
use quadrotor::vehicle::Quadrotor;

// use nalgebra::Matrix3;
// use nalgebra::Vector3;



fn altitude_controller(dt: &f32,
                       state: &HashMap<&str, f32>,
                       state_des: &HashMap<&str, f32>,
                       params: &mut HashMap<&str, f32>,
                       mass: &f32) -> f32{
    
    


    let position_error = state_des["z"] - state["z"];
    let velocity_error = state_des["z_dot"] - state["z_dot"];
    params.insert("integral_error_z", params["integral_error_z"] + position_error * dt);

    let acc_z = state_des["z_ddot"] 
                + params["kv_z"] * velocity_error 
                + params["kp_z"] * position_error
                + params["ki_z"] * params["integral_error_z"];
    
    let u1 = mass * (acc_z + params["gravity"]);               // thrust u1 cmd
    u1
}

fn main() {
    let mut points: Vec<(f64, f64)> = Vec::new();
    let mut targets: Vec<(f64, f64)> = Vec::new();

    let mass: f32 = 0.057;

    let mut params: HashMap<&str, f32> = HashMap::from([
        ("integral_error_z", 0.0),
        ("gravity", 9.81),
        ("kv_z", 15.0),
        ("kp_z", 50.0),
        ("ki_z", 8.0),
    ]);
    
    let mut state: HashMap<&str, f32> = HashMap::from([
        ("z", 0.0),
        ("z_dot", 0.0),
        ("z_ddot", 0.0)
    ]);

    let mut state_des: HashMap<&str, f32> = HashMap::from([
        ("z", 10.0),
        ("z_dot", 0.01),
        ("z_ddot", 0.0)
    ]);

    let dt: f32 = 0.1;
    

    let mut quadrotor = Quadrotor::new(mass, state);
    let m = quadrotor.mass;
    let mg = m * params["gravity"];


    for _x in 0..1000{
        let u1 = altitude_controller(&dt, &quadrotor.state, &state_des, &mut params, &m);
        let z_ddot = (u1 - mg) / m;
        
        // plot
        points.push((_x as f64, quadrotor.state["z"] as f64));
        if _x % 15 == 0 {
            targets.push((_x as f64, state_des["z"] as f64));
        }
        
        quadrotor.update_state(dt, z_ddot);
        // advance_state(self, dt):
        // quadrotor.state.insert("z_ddot", z_ddot);
        // quadrotor.state.insert("z_dot", quadrotor.state["z_dot"] + quadrotor.state["z_ddot"] * dt);
        // quadrotor.state.insert("z", quadrotor.state["z"] + quadrotor.state["z_dot"] * dt);
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