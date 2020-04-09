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

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use alloc::boxed::Box;
use cortex_m_rt as rt;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();


pub struct UsbSerialDev<'a> {
    bus: Box<UsbBusAllocator<UsbBus<Peripheral>>>,
    serial: Box<SerialPort<'a, UsbBus<Peripheral>>>,
    device: Box<UsbDevice<'a, UsbBus<Peripheral>>>
}

impl<'a> UsbSerialDev<'a> {
    fn init(&'a mut self, usb: Peripheral) {
        self.bus = Box::new(UsbBus::new(usb, unsafe { &mut EP_MEMORY }));
        self.serial = Box::new(SerialPort::new(self.bus.as_ref()));
        self.device = Box::new(UsbDeviceBuilder::new(&self.bus, UsbVidPid(0x5824, 0x27dd))
        .manufacturer("Fake company")
        .product("Enumeration test")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build());
    }
}


static mut USB_DEV: MaybeUninit<UsbSerialDev> = MaybeUninit::<UsbSerialDev>::uninit();

#[rtfm::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_serial_dev: &'static MaybeUninit<UsbSerialDev<'static>>,
    }

    #[init]
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

        //let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });
        unsafe{ (*USB_DEV.as_mut_ptr()).init(usb); 
        init::LateResources { usb_serial_dev: &USB_DEV }}
    }

    #[task(resources=[usb_serial_dev])]
    fn manage_usb(cx: manage_usb::Context) {
    }

    extern "C" {
        fn UART4();
    }
};


#[lang = "oom"]
#[no_mangle]
pub fn rust_oom(_: Layout) -> ! {
    loop {}
}
