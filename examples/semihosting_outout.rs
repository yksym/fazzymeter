#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;
extern crate stm32f4xx_hal as hal;
use cortex_m_rt::entry;
use hal::serial::Serial;
use hal::gpio::gpioa;
use hal::prelude::*;
//use f3::hal::{prelude::*, serial::Serial, stm32f30x};
use cortex_m_semihosting::{hio, hprintln};
use core::fmt::Write;


use hal::nb::block;

#[entry]
fn main() -> ! {
    //let p = stm32f30x::Peripherals::take().unwrap();

    let mut stdout = match hio::hstdout() {
        Ok(fd)  => fd,
        Err(()) => panic!("cannot use stdout"),
    };

    let language = "Rust";
    let ranking = 1;

    write!(stdout, "{} on embedded is #{}!", language, ranking);

    hprintln!("Hello, world!").unwrap();
    //let p = hal::stm32::Peripherals::take().unwrap();
    //let mut rcc = p.RCC.constrain();
    //let clocks = rcc.cfgr.freeze();

    //let mut gpioa = p.GPIOA.split();
    //let tx = gpioa.pa2.into_alternate_af7();
    //let rx = gpioa.pa3.into_alternate_af7();

    //let serial = Serial::usart2(p.USART2, (tx, rx), hal::serial::config::Config::default(), clocks);
    //let (mut tx, mut rx) = serial.unwrap().split();

    loop {
        //let byte = block!(rx.read());

        //match byte {
        //    Ok(v) => block!(tx.write(v-1)).ok(),
        //    Err(v) => block!(tx.write(100)).ok(),
        //};
    }
}
