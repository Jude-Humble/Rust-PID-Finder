use crate::pid::{PID, PID_Controllable};
use nalgebra::{Vector3, Point3, Matrix3, UnitQuaternion, matrix, Unit};

/////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone)]
pub struct Mass {
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

#[derive(Copy, Clone)]
pub struct BodyData {
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

    pub fn get_cross_areas(&self) -> Vector3<f32> {
        Vector3::new(
            2.0*self.radius*self.height,
            2.0*self.radius*self.height,
            3.1415*self.radius.powi(2),
        )
    }
}

/////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone)]
pub struct RotationalData {
    cp: Vector3<f32>,
    cmp: Vector3<f32>,
    inertia_matrix: Matrix3<f32>,
    angular_velocity: Vector3<f32>,
    angular_acceleration: Vector3<f32>,
    dampening_constant: f32,
}

impl RotationalData {
    pub fn new(cp: Vector3<f32>, cmp: Vector3<f32>, dampening_constant: f32, mass: &Mass, bdat: &BodyData) -> Self {

        let set_inertial_matrix = matrix! [
            ((1.0/12.0) * mass.current_mass * (3.0 * (bdat.radius.powi(2)) + (bdat.height.powi(2)))), 0.0, 0.0;
            0.0, ((1.0/12.0) * mass.current_mass * (3.0 * (bdat.radius.powi(2)) + (bdat.height.powi(2)))), 0.0;
            0.0, 0.0, (1.0/2.0) * mass.current_mass * bdat.radius.powi(2);
        ];

        Self {
            cp, cmp, dampening_constant,
            angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            angular_acceleration: Vector3::new(0.0, 0.0, 0.0),
            inertia_matrix: set_inertial_matrix,
        }
    }
}

/////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone)]
pub struct DragData {
    drag_coefficient: Vector3<f32>,
    air_density: f32,
}

impl DragData {
    pub fn new(drag_coefficient: Vector3<f32>, air_density: f32) -> Self {
        Self { drag_coefficient, air_density }
    }

    pub fn trans_drag(&self, vel: &Vector3<f32>, ref_areas: &Vector3<f32>) -> Vector3<f32> {
        let sc = 0.5 * self.air_density;
        let v2 = vel.map(|v| v.powi(2));

        Vector3::new(
            sc * v2.x * ref_areas.x * self.drag_coefficient.x,
            sc * v2.y * ref_areas.y * self.drag_coefficient.y,
            sc * v2.z * ref_areas.z * self.drag_coefficient.z,
        )
    }

    pub fn fetch_rot_drag(&self, ang_vel: &Vector3<f32>, ref_areas: &Vector3<f32>, radii: &Vector3<f32>) -> Vector3<f32> {
        let sc = 0.5 * self.air_density;
        let r3 = radii.map(|v| v.powi(3));
        let w2 = ang_vel.map(|v| v.powi(2));

        Vector3::new(
            sc * r3.x * w2.x * ref_areas.x * self.drag_coefficient.x,
            sc * r3.y * w2.y * ref_areas.y * self.drag_coefficient.y,
            sc * r3.z * w2.z * ref_areas.z * self.drag_coefficient.z,
        )
    }
}

/////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone)]
pub struct ThrustData {
    thrust: f32,
    powered: bool,
    engine_orientation: Unit<Vector3<f32>>,
}

impl ThrustData {
    pub fn new(thrust: f32) -> Self {
        Self {
            thrust,
            powered: true,
            engine_orientation: Unit::new_normalize(Vector3::new(0.385,0.2,0.9)),
        }
    }
}

/////////////////////////////////////////////////////////////////////////

pub struct RocketState {
    pub this_rocket: Vec<Rocket>,
}

impl RocketState {
    pub fn new() -> Self {
        Self { this_rocket: Vec::new() }
    }

    fn push_new(&mut self, rocket: &Rocket) {
        self.this_rocket.push(rocket.clone());
    }
}

/////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone)]
pub struct Rocket {
    pub position: Point3<f32>,
    pub velocity: Vector3<f32>,
    pub acceleration: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
    pub body: BodyData,
    pub thrust: ThrustData,
    pub mass: Mass,
    pub rotational: RotationalData,
    pub drag: DragData,
    pub time_step: f32,
    pub sim_duration: u32,
}

impl Rocket {
    pub fn new(
        position: Point3<f32>,
        orientation: UnitQuaternion<f32>,
        body: BodyData,
        thrust: ThrustData,
        mass: Mass,
        rotational: RotationalData,
        drag: DragData,
        time_step: f32,
        sim_duration: u32) -> Self {
        
        Self {
            position,
            orientation,
            body,
            thrust,
            mass,
            rotational,
            drag,
            time_step, sim_duration,
            velocity: Vector3::new(0.0,0.0,0.0),
            acceleration: Vector3::new(0.0,0.0,0.0),
        }

    }

    pub fn rotate_physics(&mut self) {

        let motor_moment = self.rotational.cmp;
        let aerodynamic_moment = self.rotational.cp;

        let airflow = if self.velocity.norm_squared() > 1e-8 {
            -self.velocity.normalize()
        } else {
            Vector3::new(0.0,0.0,0.0)
        };
        
        let t_force = self.orientation.transform_vector(self.thrust.engine_orientation.as_ref()) * self.thrust.thrust;

        let d_torque = self.drag.fetch_rot_drag(&self.rotational.angular_velocity, &self.body.get_cross_areas(), &self.rotational.cp);
        let total_torque = motor_moment.cross(&t_force) + aerodynamic_moment.cross(&d_torque) + (-self.rotational.dampening_constant * self.rotational.angular_velocity);

        self.rotational.inertia_matrix = matrix![
            ((1.0/12.0) * self.mass.current_mass * (3.0 * (self.body.radius.powi(2)) + (self.body.height.powi(2)))), 0.0, 0.0;
            0.0, ((1.0/12.0) * self.mass.current_mass * (3.0 * (self.body.radius.powi(2)) + (self.body.height.powi(2)))), 0.0;
            0.0, 0.0, (1.0/2.0) * self.mass.current_mass * self.body.radius.powi(2);
        ];
        let inertial_inverse = self.rotational.inertia_matrix.try_inverse().expect("Inertia tensor was not invertable!");

        self.rotational.angular_acceleration = inertial_inverse * total_torque;
        self.rotational.angular_velocity += self.rotational.angular_acceleration * self.time_step;

        let rotational_quaternion = UnitQuaternion::from_scaled_axis(self.rotational.angular_velocity * self.time_step);

        self.orientation *= rotational_quaternion;
    }

    pub fn full_physics(&mut self, history_logger: &mut RocketState) {
        let mut current_cycle: u32 = 0;
        for i in 0..self.sim_duration {
            current_cycle += 1;
            if self.position.z < 0.0 {
                break;
            }

            let g_force = Vector3::new(0.0, 0.0, -9.81 * self.mass.current_mass);
            let d_force = self.drag.trans_drag(&self.velocity, &self.body.get_cross_areas());

            let t_force = if self.thrust.powered {
                Vector3::new(
                    self.orientation.coords.x * (self.thrust.thrust * self.thrust.engine_orientation.x),
                    self.orientation.coords.y * (self.thrust.thrust * self.thrust.engine_orientation.y),
                    self.orientation.coords.z * (self.thrust.thrust * self.thrust.engine_orientation.z)
                )
            } else {
                Vector3::new(0.0,0.0,0.0)
            };

            self.acceleration = (g_force + t_force - d_force) / self.mass.current_mass;
            self.velocity += self.acceleration * self.time_step;
            self.position += self.velocity * self.time_step;

            self.rotate_physics();

            if self.mass.current_mass <= self.mass.dry_mass {
                self.thrust.powered = false;
            }

            if self.thrust.powered {
                self.mass.current_mass -= self.mass.delta_mass * self.time_step;
            }

            history_logger.push_new(self);
        }
    }
}
