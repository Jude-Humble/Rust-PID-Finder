use crate::pid::{PID, PID_Controllable};
use nalgebra::{Vector3, Point3, Matrix3, Quaternion, matrix};

/////////////////////////////////////////////////////////////////////////

struct Mass {
    dry_mass: f32,
    wet_mass: f32,
    current_mass: f32,
    delta_mass: f32,
}

impl Mass {
    pub fn new(dry_mass: f32, wet_mass: f32, delta_mass: f32) -> Self {
        Self {
            dry_mass,
            wet_mass,
            current_mass: wet_mass,
            delta_mass,
        }
    }
}

/////////////////////////////////////////////////////////////////////////

struct BodyData {
    radius: f32,
    height: f32,
}

impl BodyData {
    pub fn new(radius: f32, height: f32) -> Self {
        Self {
            radius,
            height,
        }
    }
}

/////////////////////////////////////////////////////////////////////////

struct RotationalData {
    cp: Vector3<f32>,
    cg: Vector3<f32>,
    cmp: Vector3<f32>,
    inertia_matrix: Matrix3<f32>,
    angular_velocity: Vector3<f32>,
    angular_acceleration: Vector3<f32>,
    dampening_constant: f32,
}

impl RotationalData {
    pub fn new(cp: Vector3<f32>, cg: Vector3<f32>, cmp: Vector3<f32>, dampening_constant: f32) -> Self {

        let set_inertial_matrix = matrix! [
            0.0, 0.0, 0.0;
            0.0, 0.0, 0.0;
            0.0, 0.0, 0.0;
        ];

        Self {
            cp, cg, cmp, dampening_constant,
            angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            angular_acceleration: Vector3::new(0.0, 0.0, 0.0),
            inertia_matrix: set_inertial_matrix,
        }
    }
}

/////////////////////////////////////////////////////////////////////////

struct DragData {
    drag_coefficient: Vector3<f32>,
    air_density: f32,
}

impl DragData {
    pub fn new(drag_coefficient: Vector3<f32>, air_density: f32) -> Self {
        Self { drag_coefficient, air_density }
    }
}

/////////////////////////////////////////////////////////////////////////

struct ThrustData {
    thrust: f32,
    powered: bool,
    duration: f32,
    engine_orientation: Vector3<f32>,
}

impl ThrustData {
    pub fn new(thrust: f32, duration: f32) -> Self {
        Self {
            thrust,
            duration,
            powered: true,
            engine_orientation: Vector3::new(0.0,0.0,0.0),
        }
    }
}

/////////////////////////////////////////////////////////////////////////

struct RocketState {
    this_rocket: Vec<Rocket>,
}

impl RocketState {
    fn new() -> Self {
        Self { this_rocket: Vec::new() }
    }
}

/////////////////////////////////////////////////////////////////////////


struct Rocket {
    position: Point3<f32>,
    velocity: Vector3<f32>,
    acceleration: Vector3<f32>,
    orientation: Quaternion<f32>,
    body: BodyData,
    thrust: ThrustData,
    mass: Mass,
    rotational: RotationalData,
    drag: DragData,
}

impl Rocket {
    pub fn new(
        position: Point3<f32>,
        orientation: Quaternion<f32>,
        body: BodyData,
        thrust: ThrustData,
        mass: Mass,
        rotational: RotationalData,
        drag: DragData) -> Self {
        
        Self {
            position,
            orientation,
            body,
            thrust,
            mass,
            rotational,
            drag,
            velocity: Vector3::new(0.0,0.0,0.0),
            acceleration: Vector3::new(0.0,0.0,0.0),
        }

    }
}
