#![no_std]

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use cortex_m_rt::entry;

pub use atsamd_hal as hal;
pub use hal::ehal;
pub use hal::pac;

use hal::clock::GenericClockController;
use hal::sercom::{
    v2::{
        uart::{self, BaudMode, Oversampling},
        Sercom0,
    },
    I2CMaster0,
};
use hal::time::Hertz;

hal::bsp_pins! {
    PA05 {
        name: d1
        aliases: {
            AlternateC: UartRx
        }
    }
    PA08 {
        name: d2
        aliases: {
            PushPullOutput: Led
        }
    }
    PA09 {
        name: d3
    }
    PA14 {
        name: d4
        aliases: {
            AlternateC: Sda
        }
    }
    PA15 {
        name: d5
        aliases: {
            AlternateC: Scl
        }
    }
    PA28 {
        /// RST pin
        name: d6
    }
    PA30 {
        name: d7
    }
    PA31 {
        name: d8
    }
    PA24 {
        name: d9
    }
    PA25 {
        name: d10
    }
    PA02 {
        name: d13
    }
    PA04 {
        name: d14
        aliases: {
            AlternateC: UartTx
        }
    }
}

pub type UartPads = uart::Pads<Sercom0, UartRx, UartTx>;

/// UART device for the labelled RX & TX pins
pub type Uart = uart::Uart<uart::Config<UartPads>, uart::Duplex>;

/// Convenience for setting up the D1 and D14 pins to
/// operate as UART RX/TX (respectively) running at the specified baud.
pub fn uart(
    clocks: &mut GenericClockController,
    baud: impl Into<Hertz>,
    sercom0: pac::SERCOM0,
    pm: &mut pac::PM,
    rx: impl Into<UartRx>,
    tx: impl Into<UartTx>,
) -> Uart {
    let gclk0 = clocks.gclk0();
    let clock = &clocks.sercom0_core(&gclk0).unwrap();
    let baud = baud.into();
    let pads = uart::Pads::default().rx(rx.into()).tx(tx.into());
    uart::Config::new(pm, sercom0, pads, clock.freq())
        .baud(baud, BaudMode::Fractional(Oversampling::Bits16))
        .enable()
}

/// I2C master for the labelled SDA & SCL pins
pub type I2C = I2CMaster0<Sda, Scl>;

/// Convenience for setting up the D4 and D5 pins to operate as I²C
/// SDA/SDL (respectively) running at the specified baud.
pub fn i2c_master(
    clocks: &mut GenericClockController,
    bus_speed: impl Into<Hertz>,
    sercom0: pac::SERCOM0,
    pm: &mut pac::PM,
    sda: impl Into<Sda>,
    scl: impl Into<Scl>,
) -> I2C {
    let gclk0 = clocks.gclk0();

    I2CMaster0::new(
        &clocks.sercom0_core(&gclk0).unwrap(),
        bus_speed.into(),
        sercom0,
        pm,
        sda.into(),
        scl.into(),
    )
}
