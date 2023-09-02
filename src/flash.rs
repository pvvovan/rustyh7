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
}
