use core::arch::asm;

#[no_mangle]
extern "C" fn SysTick() {
    unsafe {
        CENTISEC += 1;
        if CENTISEC >= 100 {
            CENTISEC = 0;
            TOTALSEC += 1;
        }
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
}

static mut CENTISEC: u32 = 0;
static mut TOTALSEC: u32 = 0;

pub fn sys_seconds() -> u32 {
    core::sync::atomic::fence(core::sync::atomic::Ordering::Acquire);
    unsafe { TOTALSEC }
}

pub struct Atom {
    locked_owned: bool,
}

impl Atom {
    pub fn unlock(&mut self) {
        if self.locked_owned {
            self.locked_owned = false;
            core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            unsafe {
                asm!("CPSIE i");
            }
        }
    }
}

impl Drop for Atom {
    fn drop(&mut self) {
        self.unlock();
    }
}

pub fn lock() -> Atom {
    let primask: u32;
    unsafe {
        asm!("MRS {}, PRIMASK", out(reg) primask);
    }

    let mut already_disabled = false;
    if (primask & 0x0000_0001) > 0 {
        already_disabled = true;
    } else {
        unsafe {
            asm!("CPSID i");
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    }

    Atom {
        locked_owned: !already_disabled,
    }
}
