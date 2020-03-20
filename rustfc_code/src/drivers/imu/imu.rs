pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub enum ImuError<E, E1> {
    GenericError(E),
    NssError(E1),
}

impl<E, E1> core::convert::From<E> for ImuError<E, E1> {
    fn from(error: E) -> Self {
        ImuError::GenericError(error) 
    }
}

pub trait IMU {
    type Error;
    fn set_sample_rate(&mut self, rate: u32) -> Result<(), Self::Error>;
    // pub fn get_sample_rate() -> Result<u32>;
    fn get_6dof(&mut self, gyro: &mut Vec3, accel: &mut Vec3) -> Result<(), Self::Error>;
}
