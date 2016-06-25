#![feature(core_intrinsics)]
extern crate core;
extern crate ahci;
extern crate uio;
extern crate mmap;
extern crate driverkit;

use mmap::*;
use ahci::*;
use ahci::fis::*;
use std::mem;
use driverkit::mem::DevMem;
use std::thread; // sleep()
use std::time::Duration; // sleep()

use driverkit::Volatile;
use driverkit::bitops::*;
use driverkit::timedops::wait_until;

const ATA_CMD_READ_DMA_EXT: u8 = 0x25;
const ATA_CMD_WRITE_DMA_EXT: u8 = 0x35;
const ATA_CMD_IDENTIFY: u8 = 0xEC;
const ATA_DEV_BUSY: u8 = 0x80;
const ATA_DEV_DRQ: u8 = 0x08;

pub fn round_up(val: usize, target: usize) -> usize {
    return (val + target - 1) / target * target;
}

pub struct AhciDisk {
    pub bar5: MemoryMap,
    pub dev: uio::UioDevice,
}

impl AhciDisk {
    pub fn from_uio(uio_num: usize) -> AhciDisk {
        let dev = uio::UioDevice::new(uio_num).unwrap();
        let bar = dev.map_resource(5).unwrap();

        AhciDisk {
            bar5: bar,
            dev: dev,
        }
    }

    pub fn get_hba_mut<'a>(&'a mut self) -> &'a mut HbaRaw {
        unsafe { mem::transmute::<*mut u8, &mut HbaRaw>(self.bar5.data()) }
    }

    pub fn get_hba(&self) -> &HbaRaw {
        unsafe { mem::transmute::<*mut u8, &HbaRaw>(self.bar5.data()) }
    }
}


#[repr(packed)]
pub struct HbaRaw {
    /// Host capability (0x00)
    pub cap: Volatile<u32>,
    /// Global host control (0x04)
    pub ghc: Volatile<u32>,
    /// Interrupt status (0x08)
    pub is: Volatile<u32>,
    /// Port implemented (0x0C)
    pub pi: Volatile<u32>,
    /// Version (0x10)
    pub vs: Volatile<u32>,
    /// Command completion coalescing control (0x14)
    pub ccc_ctl: Volatile<u32>,
    /// Command completion coalescing ports (0x18)
    pub ccc_pts: Volatile<u32>,
    /// Enclosure management location (0x1C)
    pub em_loc: Volatile<u32>,
    /// Enclosure management control (0x20)
    pub em_ctl: Volatile<u32>,
    /// Host capabilities extended (0x24)
    pub cap2: Volatile<u32>,
    /// BIOS/OS handoff control and status (0x28)
    pub bohc: Volatile<u32>,
    /// Reserved (0x2C - 0x9F)
    pub rsv: [u8; 116],
    /// Vendor specific registers (0xA0 - 0xFF)
    pub vendor: [u8; 96],
    /// Port control registers (0x100 - 0x10FF)
    ports: [HbaPortRaw; 32],
}

#[repr(packed)]
pub struct HbaPortRaw {
    /// command list base address, 1K-byte aligned (0x00)
    pub clb: Volatile<u64>,
    /// FIS base address, 256-byte aligned (0x08)
    pub fb: Volatile<u64>,
    /// interrupt status (0x10)
    pub is: Volatile<u32>,
    /// interrupt enable (0x14)
    pub ie: Volatile<u32>,
    /// command and status (0x18)
    pub cmd: Volatile<u32>,
    /// Reserved (0x1C)
    pub rsv0: Volatile<u32>,
    /// task file data (0x20)
    pub tfd: Volatile<u32>,
    /// signature (0x24)
    pub sig: Volatile<u32>,
    /// SATA status (SCR0:SStatus) (0x28)
    pub ssts: Volatile<u32>,
    /// SATA control (SCR2:SControl) (0x2C)
    pub sctl: Volatile<u32>,
    /// SATA error (SCR1:SError) (0x30)
    pub serr: Volatile<u32>,
    /// SATA active (SCR3:SActive) (0x34)
    pub sact: Volatile<u32>,
    /// command issue (0x38)
    pub ci: Volatile<u32>,
    /// SATA notification (SCR4:SNotification) (0x3C)
    pub sntf: Volatile<u32>,
    /// FIS-based switch control (0x40)
    pub fbs: Volatile<u32>,
    /// Port x Device Sleep (0x44)
    pub devslp: Volatile<u32>,
    /// Reserved (0x48 - 0x6F)
    pub rsv1: [Volatile<u32>; 10],
    /// Vendor specific (0x70 - 0x7F)
    pub vendor: [Volatile<u32>; 4],
}

const HOST_RESET: u32		= (1 << 0);  /* reset controller; self-clear */
const HOST_AHCI_EN: u32		= (1 << 31); /* AHCI enabled */
const HOST_IRQ_EN: u32		= (1 << 1);  /* global IRQ enable */


const PORT_CMD_LIST_ON: u32 =	(1 << 15); /* cmd list DMA engine running */
const PORT_CMD_FIS_ON: u32  =	(1 << 14); /* FIS DMA engine running */
const PORT_CMD_FIS_RX: u32  =	(1 << 4); /* Enable FIS receive DMA engine */
const PORT_CMD_CLO: u32     =	(1 << 3); /* Command list override */
const PORT_CMD_POWER_ON: u32=	(1 << 2); /* Power up device */
const PORT_CMD_SPIN_UP: u32 =	(1 << 1); /* Spin up device */
const PORT_CMD_START: u32   =	(1 << 0); /* Enable port DMA engine */
const PORT_CMD_ICC_ACTIVE: u32 =	(0x1 << 28); /* Put i/f in active state */


pub fn main() {
    println!("Hello from AHCI");
    let mut disk = AhciDisk::from_uio(0);
    let mut hba: &mut HbaRaw = disk.get_hba_mut();


    let mut cap_save = hba.cap.get();
    cap_save &= ((1<<28) | (1<<17));
    cap_save |= (1<<27);

    let tmp = hba.ghc.get();
    if tmp & HOST_RESET == 0 {
        hba.ghc.set(tmp | HOST_RESET);
    }

    std::thread::sleep(Duration::from_millis(1000));

    let tmp = hba.ghc.get();
    if tmp & HOST_RESET > 0 {
        unreachable!();
    }

    hba.ghc.set(HOST_AHCI_EN);
    hba.ghc.get(); // write_with_flish
    println!("ghc is: 0b{:x}", hba.ghc.get());

    hba.cap.set(cap_save); // ???

    hba.pi.set(0xf); // wtf?
    hba.pi.get(); // write_with_flush
    println!("pi is: 0b{:x}", hba.pi.get());

    let port_map = hba.pi.get();
    let cap = hba.cap.get();
    let n_ports: usize = (cap as usize & 0x1f) + 1;

    for i in 0..n_ports {
        let mut port = &hba.ports[i];
        println!("port[{}].sib: 0b{:x}", i, port.sig.get());
        println!("port[{}].tfd: 0b{:b}", i, port.tfd.get());

        let mut tmp = port.cmd.get();
        if tmp & (PORT_CMD_LIST_ON | PORT_CMD_FIS_ON | PORT_CMD_FIS_RX | PORT_CMD_START) > 0 {

            tmp &= !(PORT_CMD_LIST_ON | PORT_CMD_FIS_ON | PORT_CMD_FIS_RX | PORT_CMD_START);
            port.cmd.set(tmp);
            port.cmd.get();
            println!("cmd is: 0b{:x}", port.cmd.get());


			/* spec says 500 msecs for each bit, so
			 * this is slightly incorrect.
			 */
            std::thread::sleep(Duration::from_millis(500));
		}

        port.cmd.set(PORT_CMD_SPIN_UP);
        let mut j = 0;
        while j < 100 {
            std::thread::sleep(Duration::from_millis(10));
            let tmp = port.ssts.get();
            if (tmp & 0xf) == 0x3 {
                break;
            }
            j += 1;
        }

        let tmp = port.serr.get();
        println!("PORT_SCR_ERR 0x{:x}", tmp);
        port.serr.set(tmp);

        /* ack any pending irq events for this port */
        let tmp = port.is.get();
		println!("PORT_IRQ_STAT 0x{:x}", tmp);
		if tmp > 0 {
            port.is.set(tmp);
        }

        hba.is.set(1<<i);

		/* set irq mask (enables interrupts) */
		//writel(DEF_PORT_IRQ, port_mmio + PORT_IRQ_MASK);

        let mem = DevMem::alloc(1024*1024*2).unwrap();


        let tmp = port.ssts.get();
		println!("Port {} status: 0x{:x}", i, tmp);
		if (tmp & 0xf) == 0x03 {
            //probe_ent->link_port_map |= (0x01 << i);
            println!("updates linkmap...");


            let tmp = hba.ghc.get();
        	println!("HOST_CTL 0x{:x}", tmp);
        	hba.ghc.set(tmp | HOST_IRQ_EN);
        	let tmp = hba.ghc.get();
            println!("HOST_CTL 0x{:x}", tmp);

            port.clb.set(mem.physical_address());
            port.clb.get();
            println!("clb: 0x{:x}", port.clb.get());

            port.fb.set (mem.physical_address());
            port.fb.get();
            println!("fb: 0x{:x}", port.fb.get());

            port.cmd.set(
                PORT_CMD_ICC_ACTIVE | PORT_CMD_FIS_RX | PORT_CMD_POWER_ON | PORT_CMD_SPIN_UP | PORT_CMD_START
            );
            println!("cmd: 0x{:x}", port.cmd.get());
        }

    }


    loop {
        println!("port[0].tfd: 0b{:b}", hba.ports[0].tfd.get());
        std::thread::sleep(Duration::from_millis(500));
    }
    println!("");

}
