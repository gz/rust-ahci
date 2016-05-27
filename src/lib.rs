extern crate mmap;
extern crate uio;

use mmap::*;
use uio::linux;

pub struct AhciDisk {
    bar5: MemoryMap,
    dev: linux::UioDevice,
}

impl AhciDisk {
    pub fn from_uio(uio_num: usize) -> AhciDisk {
        let dev = linux::UioDevice::new(uio_num).unwrap();
        let bar = dev.map_resource(5).unwrap();
        AhciDisk { bar5: bar, dev: dev }
    }
}
