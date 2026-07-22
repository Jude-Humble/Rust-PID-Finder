pub mod pid;
pub mod rocket;

use crate::rocket::{RotationalData, BodyData, Mass, DragData, ThrustData, Rocket};
use nalgebra::{Vector3, UnitQuaternion, Point3, Quaternion};

fn main() -> Result<(), String> {

    let thrust = ThrustData::new(25.0);
    let mass = Mass::new(0.5, 1.0, 0.1);
    let body_data = BodyData::new(0.08, 1.0);
    
    let cp = Vector3::new(0.0, 0.0, 0.2);
    let cmp = Vector3::new(0.0, 0.0, 0.05);

    let drag_cos = Vector3::new(1.2, 1.2, 0.5);
    
    let drag = DragData::new(drag_cos, 1.125);

    let rot_data = RotationalData::new(cp, cmp, 0.01, &mass, &body_data);
    
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

    rocket.full_physics();

    Ok(())
}
