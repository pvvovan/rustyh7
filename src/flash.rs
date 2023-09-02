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

const SECTOR: u32 = 5;
const BASESECTOR6: *mut u32 = (0x0800_0000 + 128 * 1024 * SECTOR) as *mut u32;
const VALUE_ADDR: *mut f32 = (0x0800_0000 + 128 * 1024 * SECTOR + 4) as *mut f32;
const DEADBEEF: u32 = 0xDEAD_BEEF;

fn init() {
    let pattern = unsafe { core::ptr::read_volatile(BASESECTOR6) };
    if pattern != DEADBEEF {
        unlock();
        erase(SECTOR);
        program(0.0);
        lock();
    }
}

const FLASH_BASE: u32 = 0x5200_2000;
const FLASH_CR: *mut u32 = (FLASH_BASE + 0x00C) as *mut u32;

fn unlock() {
    const KEY1: u32 = 0x4567_0123;
    const KEY2: u32 = 0xCDEF_89AB;
    const FLASH_KEYR: *mut u32 = (FLASH_BASE + 0x004) as *mut u32;
    unsafe {
        core::ptr::write_volatile(FLASH_KEYR, KEY1);
        core::ptr::write_volatile(FLASH_KEYR, KEY2);
    }
}

fn lock() {
    const FLASH_CR_LOCK: u32 = 1;
    unsafe {
        let mut cr = core::ptr::read_volatile(FLASH_CR);
        cr |= FLASH_CR_LOCK;
        core::ptr::write_volatile(FLASH_CR, cr);
    };
}

fn erase(sector: u32) {
    const FLASH_CR_SER: u32 = 1 << 2;
    const FLASH_CR_SNBPOS: u32 = 8;
    const FLASH_CR_SNB: u32 = 7 << FLASH_CR_SNBPOS;
    const FLASH_CR_START: u32 = 1 << 7;

    unsafe {
        let mut cr = core::ptr::read_volatile(FLASH_CR);
        cr |= FLASH_CR_SER;
        cr &= !FLASH_CR_SNB;
        cr |= sector << FLASH_CR_SNBPOS;
        cr |= FLASH_CR_START;
        core::ptr::write_volatile(FLASH_CR, cr);
        waitqw();
    }
}

unsafe fn waitqw() {
    const FLASH_SR: *mut u32 = (FLASH_BASE + 0x010) as *mut u32;
    const FLASH_SR_QW: u32 = 1 << 2;
    loop {
        if core::ptr::read_volatile(FLASH_SR) & FLASH_SR_QW == 0 {
            break;
        }
    }
}

fn program(value: f32) {
    const FLASH_CR_PG: u32 = 1 << 1;
    const FLASH_CR_FW: u32 = 1 << 6;
    unsafe {
        let mut cr = core::ptr::read_volatile(FLASH_CR);
        cr |= FLASH_CR_PG | FLASH_CR_FW;
        core::ptr::write_volatile(FLASH_CR, cr);
        core::ptr::write_volatile(BASESECTOR6, DEADBEEF);
        core::ptr::write_volatile(VALUE_ADDR, value);
        waitqw();
    }
}

pub fn read() -> f32 {
    let value = unsafe { core::ptr::read_volatile(VALUE_ADDR) };
    return value;
}

pub fn write(value: f32) {
    unlock();
    erase(SECTOR);
    program(value);
    lock();
}
