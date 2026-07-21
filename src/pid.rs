pub trait PID_Controllable {
    fn control_pid(&mut self, pid: PID);
}

pub struct PID {
    kp: f32,
    ki: f32,
    kd: f32,
    derivative: f32,
    integral: f32,
    target: f32,
}
