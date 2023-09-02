pub fn setlatency() {
    const FLASH_BASE: u32 = 0x5200_2000;
    const FLASH_ACR: *mut u32 = FLASH_BASE as *mut u32;
    const FLASH_ACR_LATENCY_3WS: u32 = 0x0000_0003;
    const FLASH_ACR_LATENCY_MSK: u32 = 0xFFFF_FFF0;

    let mut latency = unsafe { core::ptr::read_volatile(FLASH_ACR) };
    latency = latency & FLASH_ACR_LATENCY_MSK;
    latency = latency | FLASH_ACR_LATENCY_3WS;
    unsafe {
        core::ptr::write_volatile(FLASH_ACR, latency);
    }

    loop {
        if unsafe { core::ptr::read_volatile(FLASH_ACR) } & FLASH_ACR_LATENCY_3WS != 0 {
            break;
        }
    }

    init();
}

const BASESECTOR6: *mut u32 = (0x0800_0000 + 128 * 5) as *mut u32;
const DEADBEEF: u32 = 0xDEAD_BEEF;

fn init() {
    if unsafe { core::ptr::read_volatile(BASESECTOR6) } != DEADBEEF {
        unlock();
    }
}

fn unlock() {
    const KEY1: u32 = 0x4567_0123;
    const KEY2: u32 = 0xCDEF_89AB;
    const FLASH_BASE: u32 = 0x5200_2000;
    const FLASH_KEYR: *mut u32 = (FLASH_BASE + 0x004) as *mut u32;
    unsafe {
        core::ptr::write_volatile(FLASH_KEYR, KEY1);
        core::ptr::write_volatile(FLASH_KEYR, KEY2);
    }
}
