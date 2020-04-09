pub use stm32f4xx_hal as hal;
use hal::usb::{Peripheral, UsbBus};
use usbd_serial::SerialPort;
use usb_device::prelude::UsbDevice;

pub type SerialDevType<'a> = SerialPort<'a, UsbBus<Peripheral>>;
pub type UsbDevType<'a> = UsbDevice<'a, UsbBus<Peripheral>>;
