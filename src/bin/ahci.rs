extern crate ahci;
extern crate uio;
extern crate mmap;

use mmap::*;
use uio::linux;
use ahci::*;
use std::mem;

pub struct AhciDisk {
    pub bar5: MemoryMap,
    pub dev: linux::UioDevice,
}

impl AhciDisk {
    pub fn from_uio(uio_num: usize) -> AhciDisk {
        let dev = linux::UioDevice::new(uio_num).unwrap();
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

pub fn main() {
    println!("Hello from AHCI");
    let mut disk = AhciDisk::from_uio(0);
    let mut hba: &mut Hba = disk.get_hba_mut();
    // Determine how many command slots the HBA supports, by reading CAP.NCS.
    let ncs = hba.cap.command_slots();

    println!("{:?}", hba.cap);
    println!("before set ahci {:?}", hba.ghc);
    hba.ghc.set_ahci();
    println!("after set ahci {:?}", hba.ghc);

    println!("Implemented ports: {:?}", hba.pi);
    for i in 0..31 {
        if hba.pi.is_implemented(i) {
            // Page 112:
            let mut port = hba.get_port_mut(0);
            println!("Port {}: {:?}", i, port.probe());
            port.stop();

            // If PxCMD.ST, PxCMD.CR, PxCMD.FRE and
            // PxCMD.FR are all cleared, the port is in an idle state.
            assert!(!port.cmd.command_list_running());
            assert!(!port.cmd.fis_receive());
            assert!(!port.cmd.fis_receive_running());
            assert!(!port.cmd.is_started());


            // For each implemented port, system software shall allocate memory for
            // and program: PxCLB PxFB
            //println!("port.clb {:?}", port.clb);
            //println!("port.fb {:?}", port.fb);

            // mmap 256 bytes for port.fb
            // mmap 1024 bytes for port.clb

            port.clb = 0x0;
            port.fb = 0x0;

            //port.reset();
        }
    }

    let mut port = hba.get_port_mut(0);
    for i in 0..31 {
        if !port.ci.commands_issued(i) {
            println!("Found free slot: {:?}", i);

        }
    }

}
