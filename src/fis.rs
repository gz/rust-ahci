pub enum FisType {
    /// Register FIS – Host to Device
    RegisterFis_H2D = 0x27,
    /// Register FIS – Device to Host
    RegisterFis_D2H = 0x34,
    /// DMA Activate FIS – Device to Host
    DmaActivate_D2H = 0x39,
    /// DMA Setup FIS – Bi-directional
    DmaSetup = 0x41,
    /// Data FIS – Bi-directional
    Data = 0x46,
    /// BIST Activate FIS – Bi-directional
    BISTActivate = 0x58,
    /// PIO Setup FIS – Device to Host
    PIOSetup_D2H = 0x5F,
    /// Set Device Bits FIS – Device to Host
    SetDeviceBits_D2H = 0xA1,
    /// Reserved for future Serial ATA definition
    Reserved1 = 0xA6,
    /// Reserved for future Serial ATA definition
    Reserved2 = 0xB8,
    /// Reserved for future Serial ATA definition
    Reserved3 = 0xBF,
    /// Vendor specific
    Vendor1 = 0xC7,
    /// Vendor specific
    Vendor2 = 0xD4,
    /// Reserved for future Serial ATA definition
    Reserved4 = 0xD9,
}

/// FIS Register - Host to Device (10.3.4)
#[repr(packed)]
#[derive(Debug)]
pub struct H2DRegister {
    /// FIS Type - Set to a value of 27h.
    /// Defines the rest of the FIS fields.
    /// Defines the length of the FIS as five Dwords.
    typ: u8,
    /// Bit 8 is set to one when the register transfer is due to an update of the Command register.
    flags: u8,
    /// Contains the contents of the Command register of the Shadow Register Block.
    command: u8,
    /// Contains the contents of the Features (7:0) register of the Shadow Register Block.
    features_low: u8,
    /// Contains the contents of the LBA register (23:0) of the Shadow Register Block
    /// and the contents of the Device register of the Shadow Register Block.
    lba_low_device: u32,
    /// Contains the contents of the LBA register of the Shadow Register Block.
    /// and the contents of the Features (15:8) register of the Shadow Register Block.
    lba_high_features: u32,
    /// Contains the contents of the Sector Count register of the Shadow Register Block.
    count: u16,
    /// Isochronous Command Completion (ICC) contains a value is set by the host to inform device of a time limit.
    icc: u8,
    /// Contains the contents of the Device Control register of the Shadow Register Block.
    control: u8,
    reserved: u32,
}

impl H2DRegister {
    pub fn new(t: FisType,
               c: bool,
               command: u8,
               features: u16,
               lba: u64,
               device: u8,
               icc: u8,
               count: u16,
               control: u8)
               -> H2DRegister {

        let flags = match c {
            true => 1 << 7,
            false => 0,
        };

        let lba_low = (lba & 0xffffff) as u32; // bits 23:0
        let lba_high = ((lba & 0xffffff << 24) >> 24) as u32;

        let lba_low_device = ((device as u32) << 24) | lba_low;
        let lba_high_features = (features as u32 & 0xff00) << 16 | lba_high;

        H2DRegister {
            typ: t as u8,
            flags: flags,
            command: command,
            features_low: (features & 0xff) as u8,
            lba_low_device: lba_low_device,
            lba_high_features: lba_high_features,
            count: count,
            icc: icc,
            control: control,
            reserved: 0,
        }
    }
}


/// FIS Register - Device to Host
#[repr(packed)]
#[derive(Debug)]
pub struct D2HRegister {
    typ: u8,
    rrir_port: u8,
    status: u8,
    error: u8,
    lba_low_device: u32,
    lba_up: u32,
    count: u16,
    reserved1: u16,
    reserved2: u32,
}
