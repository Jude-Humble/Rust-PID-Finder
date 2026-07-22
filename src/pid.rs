use nalgebra::{Vector3};

pub trait PID_Controllable {
    fn control_pid(&mut self, pid: PID);
}

pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    derivative: Vector3<f32>,
    integral: Vector3<f32>,
    target: Vector3<f32>,
    prev_err: Vector3<f32>,
}

impl PID {
    pub fn PID(kp: f32, ki: f32, kd: f32, target: Vector3<f32>) -> Self {
        Self {
            kp,
            ki,
            kd,
            derivative: Vector3::new(0.0,0.0,0.0),
            integral: Vector3::new(0.0,0.0,0.0),
            target,
            prev_err: Vector3::new(0.0,0.0,0.0)
        }
    }

    pub fn run_pid(&mut self, reference: &Vector3<f32>, time_step: f32) -> Vector3<f32> {
        self.integral += reference * time_step;
        self.derivative = (reference - self.prev_err) / time_step;
        self.prev_err = reference.clone();

        Vector3::new(
            (-0.00001 * self.kp * reference.x)
            + (-0.00001 * self.integral.x * self.ki)
            + (-0.00001 * self.derivative.x * self.kd),

            (-0.00001 * self.kp * reference.y)
            + (-0.00001 * self.integral.y * self.ki)
            + (-0.00001 * self.derivative.y * self.kd),

            (-0.00001 * self.kp * reference.z)
            + (-0.00001 * self.integral.z * self.ki)
            + (-0.00001 * self.derivative.z * self.kd),
        )
    }
}
