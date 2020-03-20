#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use stm32f4xx_hal::{prelude::*, stm32};
use stm32f4xx_hal::usb::{Peripheral, UsbBus};
use stm32f4xx_hal::spi::{Spi, Mode, Polarity, Phase};
use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::fmt::Write;
mod drivers;
use crate::drivers::imu::mpu6000::*;
use crate::drivers::imu::imu::Vec3;
use crate::drivers::imu::imu::IMU;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

macro_rules! uprint {
    ($serial:expr, $($arg:tt)*) => {
        $serial.write_fmt(format_args!($($arg)*)).ok()
    };
}

macro_rules! uprintln {
    ($serial:expr, $fmt:expr) => {
        uprint!($serial, concat!($fmt, "\n"))
    };
    ($serial:expr, $fmt:expr, $($arg:tt)*) => {
        uprint!($serial, concat!($fmt, "\n"), $($arg)*)
    };
}

fn create_usb_device<'a>(bus: &'a UsbBusAllocator<UsbBus<Peripheral>>) -> UsbDevice<UsbBus<Peripheral>> {
    UsbDeviceBuilder::new(bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("Fake company")
        .product("Enumeration test")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build()
}

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {

    let rcc = dp.RCC.constrain();

    let _clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(168.mhz())
        .pclk1(24.mhz())
        .require_pll48clk()
        .freeze();

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let mut led = gpiob.pb5.into_push_pull_output();

    let mut delay = hal::delay::Delay::new(cp.SYST, _clocks);
    
    // Create MPU driver
    let cs = gpioa.pa4.into_push_pull_output();
    let pins = (
            gpioa.pa5.into_alternate_af5(),
            gpioa.pa6.into_alternate_af5(),
            gpioa.pa7.into_alternate_af5(),
        );

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition
    };

    let spi = Spi::spi1(
            dp.SPI1,
            pins,
            spi_mode,
            1_000_000.hz(),
            _clocks
        );

    let mut mpu = Mpu6000::create_default(spi, cs).into_gyro_250_dps().into_accel_2_g();
    mpu.set_dlpf_cfg(DLPF_CFG::BANDWIDTH_256);
    if let Ok(()) = mpu.set_sample_rate(8000) {
    } else {
        panic!();
    }

    let mut gyro = Vec3{x: 0.0, y: 0.0, z: 0.0};
    let mut accel = Vec3{x: 1.0, y: 1.0, z: 1.0};

    // Create USB driver
    let usb = Peripheral {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate_af10(),
        pin_dp: gpioa.pa12.into_alternate_af10(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = create_usb_device(&usb_bus);

    let x = accel.x;
    let mut buf = [0u8; 64];
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            if let Ok(()) = mpu.get_6dof(&mut gyro, &mut accel) {
                match serial.read(&mut buf) {
                    Ok(len) if len > 0 => {uprintln!(serial, "gyro, x: {}, y: {}, z: {}", x, accel.y, accel.z);}
                    _ => {}
                }
            }

        }
    }
    }
    loop {}
}
