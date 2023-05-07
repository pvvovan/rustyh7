#![no_main]
#![no_std]

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}

#[no_mangle]
extern "C" fn main() -> ! {
    let x = RODATA;
    let y = unsafe { &mut BSS };
    let z = unsafe { &mut DATA };

    loop {
        *z += x[0] as u16;
        *y += 2;
    }
}

static RODATA: &[u8] = b"Hello, world!";
static mut BSS: u8 = 0;
static mut DATA: u16 = 1;
