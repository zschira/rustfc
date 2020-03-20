use crate::drivers::imu::imu::{IMU, Vec3, ImuError};
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;

// Define registers
enum Register {
    SMPLRT_DIV = 0x19,
    CONFIG = 0x1A,
    GYRO_CONFIG = 0x1B,
    ACCEL_CONFIG = 0x1C,
    FIFO_EN = 0x23,
    INT_PIN_CFG = 0x37,
    INT_ENABLE = 0x38,
    INT_STATUS = 0x3A,
    ACCEL_X_OUT_H = 0x3B,
    ACCEL_X_OUT_L = 0x3C,
    ACCEL_Y_OUT_H = 0x3D,
    ACCEL_Y_OUT_L = 0x3E,
    ACCEL_Z_OUT_H = 0x3F,
    ACCEL_Z_OUT_L = 0x40,
    TEMP_OUT_H = 0x41,
    TEMP_OUT_L = 0x42,
    GYRO_X_OUT_H = 0x43,
    GYRO_X_OUT_L = 0x44,
    GYRO_Y_OUT_H = 0x45,
    GYRO_Y_OUT_L = 0x46,
    GYRO_Z_OUT_H = 0x47,
    GYRO_Z_OUT_L = 0x48,
    USER_CTRL = 0x6A,
    PWR_MGMT_1 = 0x6B,
}

pub enum DLPF_CFG {
    BANDWIDTH_256 = 0,
    BANDWIDTH_188 = 1,
    BANDWIDTH_98 = 2,
    BANDWIDTH_42 = 3,
    BANDWIDTH_20 = 4,
    BANDWIDTH_10 = 5,
    BANDWIDTH_5 = 6,
    RESERVED = 7,
}

// mput6000 struct
pub struct Mpu6000<SPI, GPIO, GYRO_CONFIG, ACCEL_CONFIG> {
    spi: SPI,
    cs: GPIO,
    gyro_config: GYRO_CONFIG,
    accel_config: ACCEL_CONFIG,
    gyro_conversion: f32,
    accel_conversion: f32,
}

pub struct Gyro250DpS;
pub struct Gyro500DpS;
pub struct Gyro1000DpS;
pub struct Gyro2000DpS;

pub struct Accel2g;
pub struct Accel4g;
pub struct Accel8g;
pub struct Accel16g;

pub struct Unknown;

impl<SPI, GPIO, E, E1> Mpu6000<SPI, GPIO, Unknown, Unknown>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          GPIO: OutputPin<Error = E1>
{
    pub fn create_default(_spi: SPI, _cs: GPIO) -> Mpu6000<SPI, GPIO, Unknown, Unknown> {
         let mut mpu = Mpu6000 {
            spi: _spi,
            cs: _cs,
            gyro_config: Unknown,
            accel_config: Unknown,
            gyro_conversion: 0f32,
            accel_conversion: 0f32,
        };
        mpu.write_register(Register::PWR_MGMT_1, 0x80);
        mpu
    }
}

impl<SPI, GPIO, GYRO_CONFIG, ACCEL_CONFIG, E, E1> Mpu6000<SPI, GPIO, GYRO_CONFIG, ACCEL_CONFIG>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          GPIO: OutputPin<Error = E1>
{
    fn write_register(&mut self, register: Register, data: u8) -> Result<(), <Self as IMU>::Error> {
        self.cs.set_low().map_err(|a| ImuError::NssError(a))?;
        self.spi.write(&[register as u8, data])?;
        self.cs.set_high().map_err(|a| ImuError::NssError(a))?;
        Ok(())
    }

    fn read_registers(&mut self, register_start: Register, buffer: &mut [u8]) -> Result<(), <Self as IMU>::Error> {
        buffer[0] = register_start as u8;
        self.cs.set_low().map_err(|a| ImuError::NssError(a))?;
        self.spi.transfer(buffer)?;
        self.cs.set_high().map_err(|a| ImuError::NssError(a))?;
        Ok(())
    }

    pub fn set_dlpf_cfg(&mut self, cfg: DLPF_CFG) {
        Self::write_register(self, Register::CONFIG, cfg as u8);
    }

    pub fn into_gyro_250_dps(mut self) -> Mpu6000<SPI, GPIO, Gyro250DpS, ACCEL_CONFIG> {
        Self::write_register(&mut self, Register::GYRO_CONFIG, 1);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: Gyro250DpS,
            accel_config: self.accel_config,
            gyro_conversion: 250f32,
            accel_conversion: self.accel_conversion,
        }
    }

    pub fn into_gyro_500_dps(mut self) -> Mpu6000<SPI, GPIO, Gyro500DpS, ACCEL_CONFIG> {
        Self::write_register(&mut self, Register::GYRO_CONFIG, 1 << 3);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: Gyro500DpS,
            accel_config: self.accel_config,
            gyro_conversion: 500f32,
            accel_conversion: self.accel_conversion,
        }
    }

    pub fn into_gyro_1000_dps(mut self) -> Mpu6000<SPI, GPIO, Gyro1000DpS, ACCEL_CONFIG> {
        Self::write_register(&mut self, Register::GYRO_CONFIG, 2 << 3);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: Gyro1000DpS,
            accel_config: self.accel_config,
            gyro_conversion: 1000f32,
            accel_conversion: self.accel_conversion,
        }
    }

    pub fn into_gyro_2000_dps(mut self) -> Mpu6000<SPI, GPIO, Gyro2000DpS, ACCEL_CONFIG> {
        Self::write_register(&mut self, Register::GYRO_CONFIG, 3 << 3);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: Gyro2000DpS,
            accel_config: self.accel_config,
            gyro_conversion: 2000f32,
            accel_conversion: self.accel_conversion,
        }
    }

    pub fn into_accel_2_g(mut self) -> Mpu6000<SPI, GPIO, GYRO_CONFIG, Accel2g> {
        Self::write_register(&mut self, Register::ACCEL_CONFIG, 0);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: self.gyro_config,
            accel_config: Accel2g,
            gyro_conversion: self.gyro_conversion,
            accel_conversion: 2f32,
        }
    }

    pub fn into_accel_4_g(mut self) -> Mpu6000<SPI, GPIO, GYRO_CONFIG, Accel4g> {
        Self::write_register(&mut self, Register::ACCEL_CONFIG, 1 << 3);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: self.gyro_config,
            accel_config: Accel4g,
            gyro_conversion: self.gyro_conversion,
            accel_conversion: 4f32,
        }
    }

    pub fn into_accel_8_g(mut self) -> Mpu6000<SPI, GPIO, GYRO_CONFIG, Accel8g> {
        Self::write_register(&mut self, Register::ACCEL_CONFIG, 2 << 3);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: self.gyro_config,
            accel_config: Accel8g,
            gyro_conversion: self.gyro_conversion,
            accel_conversion: 8f32,
        }
    }

    pub fn into_accel_16_g(mut self) -> Mpu6000<SPI, GPIO, GYRO_CONFIG, Accel16g> {
        Self::write_register(&mut self, Register::ACCEL_CONFIG, 3 << 3);
        Mpu6000 {
            spi: self.spi,
            cs: self.cs,
            gyro_config: self.gyro_config,
            accel_config: Accel16g,
            gyro_conversion: self.gyro_conversion,
            accel_conversion: 16f32,
        }
    }
}

impl<SPI, GPIO, GYRO_CONFIG, ACCEL_CONFIG, E, E1> IMU for Mpu6000<SPI, GPIO, GYRO_CONFIG, ACCEL_CONFIG>
    where SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
          GPIO: OutputPin<Error = E1>
{
    type Error = ImuError<E, E1>;

    fn set_sample_rate(&mut self, rate: u32) -> Result<(), Self::Error> {
        let smplrt_div = (8000 / rate - 1) as u8;
        Self::write_register(self, Register::SMPLRT_DIV, smplrt_div)
    }

    fn get_6dof(&mut self, gyro: &mut Vec3, accel: &mut Vec3) -> Result<(), Self::Error> {
        let mut buffer = [1u8; 15];
        Self::read_registers(self, Register::ACCEL_X_OUT_H, &mut buffer)?;
        accel.x = (((buffer[1] as u16) << 8) | (buffer[2] as u16)) as f32 / self.accel_conversion;
        accel.y = (((buffer[3] as u16) << 8) | (buffer[4] as u16)) as f32 / self.accel_conversion;
        accel.z = (((buffer[5] as u16) << 8) | (buffer[6] as u16)) as f32 / self.accel_conversion;
        gyro.x = (((buffer[9] as u16) << 8) | (buffer[10] as u16)) as f32 / self.gyro_conversion;
        gyro.y = (((buffer[11] as u16) << 8) | (buffer[12] as u16)) as f32 / self.gyro_conversion;
        gyro.z = (((buffer[13] as u16) << 8) | (buffer[14] as u16)) as f32 / self.gyro_conversion;
        Ok(())
    }
}

