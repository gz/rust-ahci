extern crate ahci;

use ahci::*;

pub fn main() {
    println!("Hello from AHCI");
    let disk = AhciDisk::from_uio(0);
}
