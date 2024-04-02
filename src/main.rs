#![no_std]
#![no_main]

//use hal::hal_02::digital::v2::ToggleableOutputPin;
use panic_halt as _;

use rtt_target::{rtt_init_print, rprintln};
use stm32f4xx_hal as hal;

use crate::hal::{
    gpio::{Edge, Input, PA0, PD12, Output},
    interrupt, pac,
    prelude::*,
    otg_fs::{UsbBus, USB},
};

use cortex_m_rt::entry;

use cortex_m::interrupt::{free, Mutex};
use usb_device::device::StringDescriptors;
use usb_device::prelude::*;
use usb_device::class_prelude::UsbBusAllocator;
use usbd_serial::{
    SerialPort,
    USB_CLASS_CDC
};

use core::{
    cell::RefCell,
    ops::DerefMut
};

static BUTTON: Mutex<RefCell<Option<PA0<Input>>>> = Mutex::new(RefCell::new(None));
static LED_HANDLER: Mutex<RefCell<Option<PD12<Output>>>> = Mutex::new(RefCell::new(None));

static G_USB_SERIAL: Mutex<RefCell<Option<SerialPort<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));

static G_USB_DEVICE: Mutex<RefCell<Option<UsbDevice<UsbBus<USB>>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    static mut USB_BUS: Option<UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> = None;
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];


    rtt_init_print!();
    rprintln!("Starting Controller");

    let mut dp = pac::Peripherals::take().unwrap();
    let mut syscfg = dp.SYSCFG.constrain();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .pclk1(24.MHz())
        .require_pll48clk()
        .freeze();

    let gpioa = dp.GPIOA.split();

    let mut board_btn = gpioa.pa0.into_pull_down_input();
    board_btn.make_interrupt_source(&mut syscfg);
    board_btn.enable_interrupt(&mut dp.EXTI);
    board_btn.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

    let gpiod = dp.GPIOD.split();
    let green = gpiod.pd12.into_push_pull_output();

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into(),
        pin_dp: gpioa.pa12.into(),
        hclk: clocks.hclk(),
    };
    rprintln!("USB set up");

    USB_BUS.replace(stm32f4xx_hal::otg_fs::UsbBusType::new(usb, EP_MEMORY));

    let local_usb_bus = USB_BUS.as_ref().unwrap();

    let serial = SerialPort::new(&local_usb_bus);
    rprintln!("Serial set up");

    let usb_dev = UsbDeviceBuilder::new(&local_usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();

    free(|cs| {
        BUTTON.borrow(cs).replace(Some(board_btn));
        LED_HANDLER.borrow(cs).replace(Some(green));

        G_USB_SERIAL.borrow(cs).replace(Some(serial));
        G_USB_DEVICE.borrow(cs).replace(Some(usb_dev));
    });
    rprintln!("Handlers registered ");

    pac::NVIC::unpend(hal::pac::Interrupt::EXTI0);
    pac::NVIC::unpend(hal::pac::Interrupt::OTG_FS);
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::OTG_FS);
        pac::NVIC::unmask(hal::pac::Interrupt::EXTI0);
    };
    rprintln!("Start loop");

    #[allow(clippy::empty_loop)]
    loop {

    }
}

#[allow(non_snake_case)]
#[interrupt]
fn EXTI0() {
    free(|cs| {
        rprintln!("EXTI0 Interrupt");
        let mut btn_ref = BUTTON.borrow(cs).borrow_mut();
        if let Some(ref mut btn) = btn_ref.deref_mut() {
            btn.clear_interrupt_pending_bit();
        }
        let mut grn_ref = LED_HANDLER.borrow(cs).borrow_mut();
        if let Some(ref mut grn) = grn_ref.deref_mut() {
            grn.toggle();
        }
    });
}

#[allow(non_snake_case)]
#[interrupt]
fn OTG_FS() {
    static mut USB_SERIAL: Option<SerialPort<UsbBus<USB>>> = None;
    static mut USB_DEVICE: Option<UsbDevice<UsbBus<USB>>> = None;

    let usb_dev = USB_DEVICE.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move USB device here, leaving a None in its place
            G_USB_DEVICE.borrow(cs).replace(None).unwrap()
        })
    });

    let serial = USB_SERIAL.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move USB serial device here, leaving a None in its place
            G_USB_SERIAL.borrow(cs).replace(None).unwrap()
        })
    });

    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {

                match core::str::from_utf8( &mut buf[0..count]) {
                    Err(e) => {
                        rprintln!("String is not valid UTF8: {}", e);
                    }
                    Ok(msg) => {
                        rprintln!("Received {} Bytes: {}", count, msg);
                    }
                }

                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }
                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {
                            rprintln!("Error during Write to serial device");
                        }
                    }
                }
            }
            Err(UsbError::WouldBlock) => { } // No Error
            Err(UsbError::BufferOverflow) => {
                rprintln!("Buffer overflowed");
            }
            _ => {
                rprintln!("Unknown error occured");
            }
        }
    }
}

