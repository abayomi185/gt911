#![no_std]
#![no_main]

use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Flex, Input, Io, Level, Output, Pull},
    i2c::I2C,
    peripherals::{Peripherals, I2C0},
    prelude::*,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    log::info!("Initializing touch controller");
    let i2c0: I2C<I2C0, Async> = I2C::new(
        peripherals.I2C0,
        io.pins.gpio33,
        io.pins.gpio32,
        400.kHz(),
        &clocks,
    );
    let touch_reset = Output::new(io.pins.gpio25, Level::Low);
    let mut touch_interrupt = Flex::new(io.pins.gpio22);

    touch_interrupt.set_low();

    let mut touch_controller = gt911::GT911Builder::new(i2c0, touch_reset, delay)
        .resolution(240 as u16, 320 as u16)
        .callback(|| {
            touch_interrupt.set_as_input(Pull::None);
            true
        })
        .build()
        .unwrap();
    log::info!("Touch controller initialised");

    log::info!("{:?}", touch_controller.get_resolution().unwrap());

    loop {}
}
