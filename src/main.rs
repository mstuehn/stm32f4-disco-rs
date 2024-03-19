#![no_std]
#![no_main]

use panic_halt as _;

use rtt_target::{rtt_init_print, rprintln};

use cortex_m_rt::entry;
use stm32f4xx_hal::gpio::ReadPin;
use stm32f4xx_hal::hal::digital::OutputPin;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::{pac, prelude::*};
use usb_device::device::StringDescriptors;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {

    rtt_init_print!();
    rprintln!("Starting Controller");

    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .pclk1(24.MHz())
        .require_pll48clk()
        .freeze();

    let gpioa = dp.GPIOA.split();
    let button = gpioa.pa0.into_input();

    let gpiod = dp.GPIOD.split();
    let mut green = gpiod.pd12.into_push_pull_output();
    green.set_high(); // Turn off

    let mut red = gpiod.pd14.into_push_pull_output();
    red.set_high(); // Turn off

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into(),
        pin_dp: gpioa.pa12.into(),
        hclk: clocks.hclk(),
    };
    rprintln!("USB set up");

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = SerialPort::new(&usb_bus);
    rprintln!("Serial set up");

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();

    loop {
        if button.is_low() {
            red.set_low();
        } else {
            red.set_high();
        }

        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                green.set_low(); // Turn on

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

        green.set_high(); // Turn off
    }
}
