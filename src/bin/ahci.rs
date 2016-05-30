extern crate ahci;

use ahci::*;
use std::mem;

pub fn main() {
    println!("Hello from AHCI");
    let disk = AhciDisk::from_uio(0);

    let bar = disk.bar5;
    let port = unsafe { mem::transmute::<*mut u8, &Hba>(bar.data()) };
    println!("{:?}", port);
}
