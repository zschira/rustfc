#![no_std]
#![no_main]

#![feature(alloc)]
#![feature(lang_items)]
#![feature(const_mut_refs)]
#![feature(maybe_uninit_ref)]

// Plug in the allocator crate
extern crate alloc;
extern crate alloc_cortex_m;

extern crate panic_semihosting;

mod platform;
use crate::platform::hal as hal;

use hal::{prelude::*, stm32};
use hal::usb::{Peripheral, UsbBus};
use hal::spi::{Spi, Mode, Polarity, Phase};

use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::mem::MaybeUninit;
use core::ptr::null_mut;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use alloc::alloc::alloc;
use alloc::boxed::Box;
use cortex_m_rt as rt;
use rtfm::{Exclusive, Mutex};
use core::fmt::Write;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

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

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        bus: UsbBusAllocator<UsbBus<Peripheral>>,
    }

    #[init(spawn = [manage_usb])]
    fn init(cx: init::Context) -> init::LateResources {
        let cp = cx.core;
        let dp = cx.device;

        // Configure heap
        let start = rt::heap_start() as usize;
        let size = 4096;
        unsafe { ALLOCATOR.init(start, size) }

        let rcc = dp.RCC.constrain();

        let _clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(168.mhz())
            .pclk1(24.mhz())
            .require_pll48clk()
            .freeze();

        let gpioa = dp.GPIOA.split();

        // Create USB driver
        let usb = Peripheral {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
        };

        let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });
        cx.spawn.manage_usb();
        init::LateResources { bus: usb_bus }
    }

    #[task(resources=[bus])]
    fn manage_usb(cx: manage_usb::Context) {
        static mut serial_ptr: *mut u8 = null_mut::<u8>();
        static mut usb_device_ptr: *mut u8 = null_mut::<u8>();
        if(((*serial_ptr).is_null()) || ((*usb_device_ptr).is_null())) {
            create_usb(&cx.resources.bus, *serial_ptr as *mut SerialPort<UsbBus<Peripheral>>, *usb_device_ptr as *mut UsbDevice<UsbBus<Peripheral>>);
        }
        let serial = unsafe{ (*serial_ptr as *mut SerialPort<UsbBus<Peripheral>>).as_mut().unwrap() };
        let usb_dev= unsafe{ (*usb_device_ptr as *mut UsbDevice<UsbBus<Peripheral>>).as_mut().unwrap() };
        let mut buf = [0u8; 64];
        loop {
            if usb_dev.poll(&mut [serial]) {
                match serial.read(&mut buf) {
                    Ok(len) if len > 0 => { uprintln!(serial, "Hello World!"); }
                    _ => {}
                }
            }
        }
    }

    extern "C" {
        fn UART4();
    }
};

fn create_usb<'a>(usb_bus: &'a UsbBusAllocator<UsbBus<Peripheral>>, mut serial_ptr: *mut SerialPort<'a, UsbBus<Peripheral>>, mut device_ptr: *mut UsbDevice<'a, UsbBus<Peripheral>>) {
    serial_ptr = unsafe{ alloc(Layout::new::<SerialPort<UsbBus<Peripheral>>>()) as *mut SerialPort<UsbBus<Peripheral>> };
    device_ptr = unsafe{ alloc(Layout::new::<UsbDevice<UsbBus<Peripheral>>>()) as *mut UsbDevice<UsbBus<Peripheral>> };
    if let (Some(serial), Some(device)) = (
        unsafe{ serial_ptr.as_mut() },
        unsafe{ device_ptr.as_mut() },
    ) {
        *serial = SerialPort::new(usb_bus);
        *device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("Fake company")
        .product("Enumeration test")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();
    };
}

#[lang = "oom"]
#[no_mangle]
pub fn rust_oom(_: Layout) -> ! {
    loop {}
}
