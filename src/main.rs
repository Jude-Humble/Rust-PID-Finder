pub mod pid;
pub mod rocket;

use crate::rocket::{RocketState, RotationalData, BodyData, Mass, DragData, ThrustData, Rocket};
use nalgebra::{Vector3, UnitQuaternion, Point3, Quaternion};
use csv::Writer;
use std::error::Error;
use std::process::Command;

fn main() -> Result<(), Box<dyn Error>> {

    let thrust = ThrustData::new(25.0);
    let mass = Mass::new(0.5, 1.0, 0.1);
    let body_data = BodyData::new(0.08, 1.0);
    
    let cp = Vector3::new(0.0, 0.0, 0.2);
    let cmp = Vector3::new(0.0, 0.0, 0.05);

    let drag_cos = Vector3::new(1.2, 1.2, 0.5);
    
    let drag = DragData::new(drag_cos, 1.125);

    let rot_data = RotationalData::new(cp, cmp, 0.01, &mass, &body_data);

    let mut log_tracker = RocketState::new();
    
    let mut rocket = Rocket::new(
        Point3::new(0.0,0.0,0.0),
        UnitQuaternion::from_quaternion(Quaternion::from_parts(0.0, Vector3::new(0.0,0.0,1.0))),
        body_data,
        thrust,
        mass,
        rot_data,
        drag,
        0.01,
        1000,
    );

    rocket.full_physics(&mut log_tracker);

    let mut writer = Writer::from_path("output.csv")?;

    for row in log_tracker.this_rocket.iter() {
        writer.write_record(row.position.coords.as_slice().to_vec().iter().map(|&v| v.to_string()))?;
    }

    writer.flush()?;

    let mut generate_plots = Command::new("python3");
    generate_plots.arg("plotter/plotter.py");
    println!("test");
    
    let mut open_plots = Command::new("explorer.exe");
    open_plots.arg("plot.html");
    println!("test");

    generate_plots.status().expect("Failed to execute generation command");
    open_plots.status().expect("Failed to execute file opening command");

    Ok(())
}
