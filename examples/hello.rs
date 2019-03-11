////! Prints "Hello, world!" on the host console using semihosting
//
//#![no_main]
//#![no_std]
//
//
//use cortex_m_semihosting::{debug, hprintln};
//
//#[entry]
//fn main() -> ! {
//    hprintln!("Hello, world!").unwrap();
//
//    // exit QEMU
//    // NOTE do not run this on hardware; it can corrupt OpenOCD state
//    //debug::exit(debug::EXIT_SUCCESS);
//
//    loop {}
//}
#![no_main]
#![no_std]

extern crate panic_halt;
extern crate cortex_m;

extern crate stm32f4xx_hal as hal;

use hal::prelude::*;
use cortex_m_rt::entry;

use hal::gpio::{Output, PushPull};
use hal::gpio::gpioa::{PA5};

use hal::delay::Delay;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = hal::stm32::Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let mut gpioa = p.GPIOA.split();
    let mut pa5: PA5<Output<PushPull>> = 
            gpioa.pa5.into_push_pull_output();

    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        pa5.set_high();
        delay.delay_ms(1_000_u16);
        pa5.set_low();
        delay.delay_ms(1_000_u16);
    }
}
