#![no_main]
#![no_std]

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}

#[no_mangle]
pub extern "C" fn DefaultExceptionHandler() {
    loop {}
}

#[no_mangle]
pub extern "C" fn HardFault() {
    loop {}
}

#[no_mangle]
extern "C" fn main() -> ! {
    let x = RODATA;
    let y = unsafe { &mut BSS };
    let z = unsafe { &mut DATA };
    let mut _fl: f32 = 0.0;

    loop {
        *z += x[0] as u16;
        *y += 2;
        _fl += 1.1;
    }
}

static RODATA: &[u8] = b"Hello, world!";
static mut BSS: u8 = 0;
static mut DATA: u16 = 1;
