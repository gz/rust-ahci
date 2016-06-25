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

    pub fn get_hba_mut<'a>(&'a mut self) -> &'a mut Hba {
        unsafe { mem::transmute::<*mut u8, &mut Hba>(self.bar5.data()) }
    }

    pub fn get_hba(&self) -> &Hba {
        unsafe { mem::transmute::<*mut u8, &Hba>(self.bar5.data()) }
    }
}

pub struct AhciPort<'a> {
    /// The Port number.
    pub ident: usize,
    /// Number of command slots (from HBACapability).
    ncs: u32,
    /// PCI config space for the port.
    port: &'a mut HbaPort,
    /// Memory for command headers.
    clb_mem: DevMem,
    /// Memory for the FB.
    fb_mem: DevMem,
    ctba_mem: Vec<DevMem>,
}

impl<'a> AhciPort<'a> {


    pub fn new(idx: usize, ncs: u32, port: &'a mut HbaPort) -> AhciPort {
        assert!(port.is_idle());

        // Page 112:
        port.stop();
        //port.reset();

        // For each implemented port, system software shall allocate memory for
        // and program: PxCLB PxFB
        let clb_mem = DevMem::alloc(4096).unwrap(); // >1024 bytes
        port.clb.set(clb_mem.physical_address());
        println!("CLB Physical Address: {:x}", clb_mem.physical_address());

        let fb_mem = DevMem::alloc(4096).unwrap(); // >256 bytes
        port.fb.set(fb_mem.physical_address());
        println!("FB Physical Address: {:x}", fb_mem.physical_address());

        //port.cmd.enable_fis_receive();
        port.serr.clear();

        // Determine which events should cause an interrupt
        port.ie.enable_all();


        let mut ctba_mem = Vec::with_capacity(ncs as usize);
        for slot in 0..ncs {
            let size = round_up(mem::size_of::<CommandTable>(), 4096);
            let mem = DevMem::alloc(size).unwrap();

            let clb = clb_mem.data();
            let header: &mut CommandHeader =
                unsafe { &mut *(clb as *mut CommandHeader).offset(slot as isize) };
            header.ctba.set(mem.physical_address());

            ctba_mem.push(mem);
        }


        println!("port.tfd {:?}\n", port.tfd);
        println!("port.cmd {:?}\n", port.cmd);
        println!("Port {}: {:?}", idx, port.probe());

        while port.tfd.busy() || port.tfd.drq() || !port.is_present() {
            std::thread::sleep(Duration::from_millis(1000));
            println!("Before Activating Port {:?}\n", idx);

            println!("port.is: {:?}", port.is);
            println!("port.cmd {:?}", port.cmd);
            println!("port.sntf {:?}", port.sntf);
            println!("port.sact {:?}", port.sact);
            println!("port.serr: {:?}", port.serr);
            println!("port.tfd {:?}\n", port.tfd);
        }

        // println!("Before CMD override {:?}\n", idx);
        // port.cmd.set_command_list_override();
        // while port.cmd.command_list_override() {}
        // println!("After CMD override {:?}\n", idx);

        // Software shall not set PxCMD.ST to ‘1’ until it is determined
        // that a functional device is present on the port as determined
        // by PxTFD.STS.BSY = ‘0’, PxTFD.STS.DRQ = ‘0’, and PxSSTS.DET = 3h.
        if port.is_present() {
            println!("Activating Port {:?}", idx);
            port.start();
        } else {
            panic!("Can't activate shit.");
        }

        AhciPort {
            ident: idx,
            ncs: ncs,
            port: port,
            clb_mem: clb_mem,
            fb_mem: fb_mem,
            ctba_mem: ctba_mem,
        }
    }

    /// Determines if command slot is empty.
    ///
    /// An empty command slot has its respective bit cleared to ‘0’
    /// in both the PxCI and PxSACT registers.
    fn is_free_cmd_slot(&self, slot: u8) -> bool {
        !self.port.ci.commands_issued(slot) && !self.port.sact.device_status(slot)
    }

    /// Finds the first free slot in all available command slots.
    fn find_free_slot(&self) -> Option<u8> {
        for slot in 0..self.ncs {
            if self.is_free_cmd_slot(slot as u8) {
                return Some(slot as u8);
            }
        }
        None
    }


    pub fn dma(&mut self, write: bool, block: u64, sectors: u64, mem: DevMem) {
        let slot = self.find_free_slot().unwrap();
        println!("Found free slot: {:?}", slot);

        // Software builds a command FIS in system memory at location
        // PxCLB[CH(pFreeSlot)]:CFIS with
        // the command type.
        let clb = self.clb_mem.data();
        let header: &mut CommandHeader =
            unsafe { &mut *(clb as *mut CommandHeader).offset(slot as isize) };

        // PRDTL containing the number of entries in the PRD table:
        header.prdtl.set(1);
        // CFL set to the length of the command in the CFIS area:
        header.set_cfl((mem::size_of::<fis::H2DRegister>() / mem::size_of::<u32>()) as u8);
        // A bit set if it is an ATAPI command:
        // header.set_atapi();
        // W (Write) bit set if data is going to the device:
        if write {
            header.set_write();
        }
        assert!(self.ctba_mem[slot as usize].physical_address() == header.ctba.get());

        println!("self.ctba_mem[slot as usize].physical_address() 0x{:x}",
                 self.ctba_mem[slot as usize].physical_address());
        println!("self.ctba_mem[slot as usize].virtual_address() 0x{:x}",
                 self.ctba_mem[slot as usize].virtual_address());

        let table: &mut CommandTable =
            unsafe { &mut *(self.ctba_mem[slot as usize].data() as *mut CommandTable) };
        table.prdt[0] = RegionDescriptor::new(mem.physical_address(), 511, false);

        assert!(mem::size_of::<fis::H2DRegister>() < 64);

        // TOOD: count = 1?
        let ata_identify = H2DRegister::new(FisType::RegisterFis_H2D,
                                            true,
                                            ATA_CMD_IDENTIFY, 0, 0, 0, 0, 0, 0);

        let fis: &mut fis::H2DRegister =
            unsafe { &mut *(table.cfis.as_ptr() as *mut fis::H2DRegister) };
        *fis = ata_identify;
        println!("{:?}", fis);

        // println!("Waiting for device to be ready... {:?}", self.port.tfd);
        // while (!self.port.tfd.busy() && self.port.tfd.drq()) {}

        println!("Waiting for command to finish...");
        assert!(self.port.cmd.command_list_running());

        self.port.ci.set_commands_issued(slot);
        while self.port.ci.commands_issued(slot) {
            println!("port.is: {:?}", self.port.is);
            println!("port.cmd {:?}", self.port.cmd);
            println!("port.tfd {:?}", self.port.tfd);
            println!("port.sntf {:?}", self.port.sntf);
            println!("port.sact {:?}", self.port.sact);
            println!("port.serr: {:?}\n", self.port.serr);

            std::thread::sleep(Duration::from_millis(1000));
        }

        println!("We are done.");
    }
}

pub fn main() {
    println!("Hello from AHCI");
    let mut disk = AhciDisk::from_uio(0);
    let mut hba: &mut Hba = disk.get_hba_mut();
    println!("hba.ghc.ahci {:?}", hba.ghc.ahci());
    hba.ghc.set_ahci();

    // Determine how many command slots the HBA supports, by reading CAP.NCS.
    let ncs = hba.cap.command_slots();

    //hba.ghc.disable_interrupts();

    hba.ghc.reset();
    while hba.ghc.reset_pending() {}
    std::thread::sleep(Duration::from_millis(10));

    hba.ghc.enable_interrupts();

    println!("Reset the HBA!");
    println!("{:?}", hba.cap);
    println!("Hba size: {:?}", std::mem::size_of::<Hba>());
    println!("HbaPort size: {:?}", std::mem::size_of::<HbaPort>());
    println!("Hba ahci: {:?}", hba.ghc.ahci());
    println!("Implemented ports: {:?}", hba.pi);

    let fb_mem = DevMem::alloc(4096).unwrap(); // >256 bytes
    let clb_mem = DevMem::alloc(4096).unwrap(); // >256 bytes

    {
        let mut port = unsafe { hba.get_port_mut(0) };
        port.stop();
        assert!(!port.cmd.is_started());
        assert!(!port.cmd.command_list_running());
        assert!(!port.cmd.fis_receive());
        assert!(!port.cmd.fis_receive_running());

        println!("[1] Busy: {:?}", port.tfd.busy());
        println!("Present: {:?}", port.is_present());
        //port.reset();
        port.fb.set(fb_mem.physical_address());
        port.clb.set(0x0000000001000000);
        port.serr.clear();
        port.cmd.enable_fis_receive();
        port.serr.clear();
        port.ie.enable_all();
    }

    {
        let mut port = unsafe { hba.get_port_mut(1) };
        port.stop();
        assert!(!port.cmd.is_started());
        assert!(!port.cmd.command_list_running());
        assert!(!port.cmd.fis_receive());
        assert!(!port.cmd.fis_receive_running());

        println!("[1] Busy: {:?}", port.tfd.busy());
        println!("Present: {:?}", port.is_present());
        //port.reset();
        port.fb.set(0x0000000001000000);
        port.clb.set(0x0000000001000000);
        port.serr.clear();
        port.cmd.enable_fis_receive();
        port.serr.clear();
        port.ie.enable_all();
    }

    {
        let mut port = unsafe { hba.get_port_mut(2) };
        port.stop();
        assert!(!port.cmd.is_started());
        assert!(!port.cmd.command_list_running());
        assert!(!port.cmd.fis_receive());
        assert!(!port.cmd.fis_receive_running());

        println!("[1] Busy: {:?}", port.tfd.busy());
        println!("Present: {:?}", port.is_present());
        //port.reset();
        port.fb.set(0x0000000001000000);
        port.clb.set(0x0000000001000000);
        port.serr.clear();
        port.cmd.enable_fis_receive();
        port.serr.clear();
        port.ie.enable_all();
    }

    let mut port = unsafe { hba.get_port_mut(0) };
    println!("Port 0 busy: {:?}", port.tfd.busy());
    let mut i = 0;
    while port.tfd.busy() {
        port.cmd.disable_fis_receive();
        while port.cmd.fis_receive_running() {}
        port.cmd.enable_fis_receive();

        port.serr.clear();
        std::thread::sleep(Duration::from_millis(1));
    }
    println!("XXX Busy: {:?}", port.tfd.busy());


    //let mut ports = Vec::new();
    //ports.push(AhciPort::new(0, ncs, port));
}
