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
    let mut hba = disk.get_hba_mut();
    println!("ports: {:?}", hba.pi);

    let mut port = hba.get_port_mut(0);
    println!("{:?}", port.probe());
    port.reset();
}
