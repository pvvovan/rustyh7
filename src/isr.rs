#[no_mangle]
pub extern "C" fn SysTick() {
    unsafe {
        CNT += 1;
        if CNT >= 100 {
            CNT = 0;
            SECONDS += 1;
        }
    }
}

static mut CNT: u32 = 0;
static mut SECONDS: u32 = 0;
