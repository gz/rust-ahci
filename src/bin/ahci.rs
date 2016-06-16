extern crate ahci;
extern crate uio;
extern crate mmap;
extern crate driverkit;

use mmap::*;
use ahci::*;
use std::mem;
use driverkit::mem::DevMem;

pub struct AhciDisk {
    pub bar5: MemoryMap,
    pub dev: uio::UioDevice,
}

impl AhciDisk {
    pub fn from_uio(uio_num: usize) -> AhciDisk {
        let dev = uio::UioDevice::new(uio_num).unwrap();
        let bar = dev.map_resource(5).unwrap();


        AhciDisk { bar5: bar, dev: dev }
    }

    pub fn get_hba_mut<'a>(&'a mut self) -> &'a mut Hba {
        unsafe { mem::transmute::<*mut u8, &mut Hba>(self.bar5.data()) }
    }

    pub fn get_hba(&self) -> &Hba {
        unsafe { mem::transmute::<*mut u8, &Hba>(self.bar5.data()) }
    }
}

pub struct AhciPort<'a> {
    pub ident: usize,
    pub port: &'a mut HbaPort,
    pub clb_mem: DevMem,
    pub fb_mem: DevMem,
}

impl<'a> AhciPort<'a> {

    pub fn new(idx: usize, port: &'a mut HbaPort) -> AhciPort {
        // Page 112:
        println!("Port {}: {:?}", idx, port.probe());
        port.stop();

        // If PxCMD.ST, PxCMD.CR, PxCMD.FRE and
        // PxCMD.FR are all cleared, the port is in an idle state.
        assert!(!port.cmd.command_list_running());
        assert!(!port.cmd.fis_receive());
        assert!(!port.cmd.fis_receive_running());
        assert!(!port.cmd.is_started());

        // For each implemented port, system software shall allocate memory for
        // and program: PxCLB PxFB
        let clb_mem = DevMem::alloc(4096).unwrap(); // >1024 bytes
        let fb_mem = DevMem::alloc(4096).unwrap(); // >256 bytes

        port.clb = clb_mem.physical_address();
        port.fb = fb_mem.physical_address();

        // Determine which events should cause an interrupt
        port.ie.enable_task_file_error();
        port.ie.enable_host_bus_fatal_error();
        port.ie.enable_host_bus_data_error();
        port.ie.enable_interface_fatal_error();
        port.ie.enable_interface_non_fatal_error();
        port.ie.enable_overflow();
        port.ie.enable_incorrect_port_multiplier();
        port.ie.enable_phyrdy_change_irq();
        port.ie.enable_device_mechanical_presence();
        port.ie.enable_port_change_irq();
        port.ie.enable_descriptor_processed();
        port.ie.enable_unknown_fis_irq();
        port.ie.enable_device_bits_fis_irq();
        port.ie.enable_pio_setup_irq();
        port.ie.enable_d2h_register_fis_interrupt();

        // Software shall not set PxCMD.ST to ‘1’ until it is determined
        // that a functional device is present on the port as determined
        // by PxTFD.STS.BSY = ‘0’, PxTFD.STS.DRQ = ‘0’, and PxSSTS.DET = 3h.
        if port.is_present() {
            println!("Activating Port {:?}", idx);
            port.start();
        }

        AhciPort { ident: idx, port: port, clb_mem: clb_mem, fb_mem: fb_mem }
    }


    fn dma(&mut self, write: bool, block: u64, sectors: u64, mem: DevMem) -> Result<usize> {

        //self.clb_mem.data() as *mut H
        /*
        let header = CommandHeader::new();
        if write {
            header.set_write();
        }*/

    }

}




pub fn main() {
    println!("Hello from AHCI");
    let mut disk = AhciDisk::from_uio(0);
    let mut hba: &mut Hba = disk.get_hba_mut();

    // Determine how many command slots the HBA supports, by reading CAP.NCS.

    println!("{:?}", hba.cap);
    hba.ghc.set_ahci();

    println!("Implemented ports: {:?}", hba.pi);
    let mut ports = Vec::new();

    let mut port = unsafe { hba.get_port_mut(0) };
    ports.push(AhciPort::new(0, port));

}
