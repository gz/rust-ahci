#![feature(concat_idents)]

extern crate mmap;
extern crate uio;
//extern crate pci;

use std::fmt;
//use std::ops;
use mmap::*;
use uio::linux;

const HBA_SIG_ATA: u32 = 0x00000101;
const HBA_SIG_ATAPI: u32 = 0xEB140101;
const HBA_SIG_PM: u32 = 0x96690101;
const HBA_SIG_SEMB: u32 = 0xC33C0101;

const HBA_SSTS_PRESENT: u32 = 0x3;

macro_rules! is_bit_set {
    ($field:expr, $bit:expr) => (
        $field & (1 << $bit) > 0
    )
}

macro_rules! bit_get {
    ($doc:meta, $fun:ident, $bit:expr) => (
        #[$doc]
        pub fn $fun(&self) -> bool {
            is_bit_set!(self.0, $bit)
        }
    )
}

macro_rules! bit_set {
    ($doc:meta, $fun:ident, $bit:expr) => (
        #[$doc]
        pub fn $fun(&mut self) {
            self.0 |= 1 << $bit;
        }
    )
}

macro_rules! bit_clear {
    ($doc:meta, $fun:ident, $bit:expr) => (
        #[$doc]
        pub fn $fun(&mut self) {
            self.0 |= 0 << $bit;
        }
    )
}


fn bits_get(r: u32, from: usize, to: usize) -> u32 {
    assert!(from <= 31);
    assert!(to <= 31);
    assert!(from <= to);

    let mask = match to {
        31 => u32::max_value(),
        _ => (1 << (to+1)) - 1,
    };

    (r & mask) >> from
}

#[derive(Debug)]
pub struct HBACapability(pub u32);

impl HBACapability {
    bit_get!(doc = "Supports 64-bit Addressing", has_64bit_addressing, 31);
    bit_get!(doc = "Supports Native Command Queuing", has_native_command_queing, 30);
    bit_get!(doc = "Supports SNotification Register", has_snotification_register, 29);
    bit_get!(doc = "Supports Mechanical Presence Switch", has_mechanical_presence_switch, 28);
    bit_get!(doc = "Supports Staggered Spin-up", has_staggered_spin_up, 27);
    bit_get!(doc = "Supports Aggressive Link Power Management", has_aggressive_link_power_mgmt, 26);
    bit_get!(doc = "Supports Activity LED", has_activity_led, 25);
    bit_get!(doc = "Supports Command List Override", has_command_list_override, 24);
    bit_get!(doc = "Supports AHCI mode only", has_ahci_mode_only, 18);
    bit_get!(doc = "Supports Port Multiplier", has_port_multiplier, 17);
    bit_get!(doc = "FIS-based Switching Supported", has_fis_switching, 16);
    bit_get!(doc = "PIO Multiple DRQ Block", has_multiple_drq_blocks, 15);
    bit_get!(doc = "Slumber State Capable", has_slumber_state, 14);
    bit_get!(doc = "Partial State Capable", has_partial_state, 13);
    bit_get!(doc = "Command Completion Coalescing Supported", has_cmd_completion_coalescing, 7);
    bit_get!(doc = "Enclosure Management Supported", has_enclosure_mgmt, 6);
    bit_get!(doc = "Supports External SATA", has_external_stat, 5);

    /// Number of Command Slots
    pub fn command_slots(&self) -> u32 {
        bits_get(self.0, 8, 12)
    }

    /// Interface Speed Support
    pub fn interface_speed(&self) -> u32 {
        bits_get(self.0, 20, 23)
    }

    /// Number of Ports (NP)
    pub fn ports(&self) -> u32 {
        bits_get(self.0, 0, 4)
    }
}

#[derive(Debug)]
pub struct GlobalHBAControl(u32);

impl GlobalHBAControl {
    bit_get!(doc = "AHCI Enable", ae, 31);
    bit_set!(doc = "AHCI Enable", set_ae, 31);
}

#[derive(Debug)]
pub struct HBAVersion(u32);

impl HBAVersion {

    /// Major Version Number
    pub fn major(&self) -> u16 {
        bits_get(self.0, 16, 31) as u16
    }

    /// Minor Version Number
    pub fn minor(&self) -> u16 {
        bits_get(self.0, 0, 15) as u16
    }
}

#[derive(Debug)]
pub struct InterruptStatus(u32);

impl InterruptStatus {

    /// Interrupt Pending Status
    pub fn is_pending(&self, port: usize) -> bool {
        is_bit_set!(self.0, port)
    }

    pub fn clear(&mut self, port: usize) {
        self.0 |= 0 << port;
    }
}

#[derive(Debug)]
pub struct ImplementedPorts(u32);

impl ImplementedPorts {

    /// Port Implemented
    pub fn is_implemented(&self, port: usize) -> bool {
        is_bit_set!(self.0, port)
    }

}

#[derive(Debug)]
pub struct CommandCompletionControl(u32);

impl CommandCompletionControl {

    /// Timeout Value
    pub fn timeout(&self) -> u16 {
        bits_get(self.0, 16, 31) as u16
    }

    /// Set Timeout Value
    pub fn set_timeout(&self, tv: u16) {
        unreachable!()
    }

    /// Command Completions
    pub fn command_completions(&self) -> u8 {
        bits_get(self.0, 8, 15) as u8
    }

    /// Set Command Completions
    pub fn set_command_completions(&self, cc: u8) {
        unreachable!()
    }

    /// Interrupt
    pub fn interupt(&self) -> u8 {
        bits_get(self.0, 3, 7) as u8
    }

    /// Set Interrupt
    pub fn set_interupt(&self, irq: u8) {
        unreachable!()
    }

    bit_get!(doc = "Is CCC enabled?", enabled, 0);
    bit_set!(doc = "Enable CCC", enable, 0);
    bit_clear!(doc = "Disable CCC", disable, 0);
}

#[derive(Debug)]
pub struct CommandCompletionPorts(u32);

impl CommandCompletionPorts {

    /// Is port part of the command completion coalescing feature?
    pub fn has_cc(&self, port: u32) -> bool {
        is_bit_set!(self.0, port)
    }

    /// Is port part of the command completion coalescing feature?
    pub fn set_cc(&mut self, port: u32) {
        self.0 |= 1 << port;
    }

}

#[derive(Debug)]
pub struct EnclosureLocation(u32);

impl EnclosureLocation {

    /// Offset
    pub fn offset(&self) -> u16 {
        bits_get(self.0, 16, 31) as u16
    }

    /// Buffer size
    pub fn buffer_size(&self) -> u16 {
        bits_get(self.0, 0, 15) as u16
    }


}

#[derive(Debug)]
pub struct EnclosureControl(u32);

impl EnclosureControl {
    bit_get!(doc = "Port Multiplier Support?", has_port_multiplier, 27);
    bit_get!(doc = "Activity LED Hardware Driven", has_hardware_activity_led, 26);
    bit_get!(doc = "Transmit Only", has_transmit_only, 25);
    bit_get!(doc = "Single Message Buffer", has_single_message_buffer, 24);
    bit_get!(doc = "SGPIO Enclosure Management Messages", has_sgpio, 19);
    bit_get!(doc = "SES-2 Enclosure Management Messages", has_ses2, 18);
    bit_get!(doc = "SAF-TE Enclosure Management Messages", has_safte, 17);
    bit_get!(doc = "LED Message Types", led_types, 16);

    bit_set!(doc = "Reset", reset, 9);
    bit_set!(doc = "Transmit Message", transmit_message, 8);

    bit_get!(doc = "Message Received", message_received, 0);
    bit_set!(doc = "Message Received", set_message_received, 0);
}

#[derive(Debug)]
pub struct ExtendedCaps(u32);

impl ExtendedCaps {
    bit_get!(doc = "DevSleep Entrance from Slumber Only", has_deso, 5);
    bit_get!(doc = "Supports Aggressive Device Sleep Management", has_agressive_sleep_management, 4);
    bit_get!(doc = "Supports Device Sleep", has_device_sleep, 3);
    bit_get!(doc = "Automatic Partial to Slumber Transitions", has_automatic_slumber, 2);
    bit_get!(doc = "NVMHCI Present", has_nvmhci, 1);
    bit_get!(doc = "BIOS/OS Handoff", has_bios_os_handoff, 0);
}

#[derive(Debug)]
pub struct BiosHandOffStatus(u32);

impl BiosHandOffStatus {
    bit_get!(doc = "BIOS Busy", is_bios_busy, 4);
    bit_set!(doc = "Set BIOS Busy", set_bios_busy, 4);
    bit_clear!(doc = "Clear BIOS Busy", clear_bios_busy, 4);

    bit_get!(doc = "OS Ownership Change", change_ownership_to_os, 3);
    bit_set!(doc = "Clear OS Ownership Change Status", clear_change_ownership_to_os, 3);

    bit_get!(doc = "SMI on OS Ownership Change", has_smi_on_ownership_change, 2);
    bit_set!(doc = "SMI on OS Ownership Change Enable", enable_smi_on_ownership_change, 2);
    bit_clear!(doc = "SMI on OS Ownership Change Disable", disable_smi_on_ownership_change, 2);

    bit_get!(doc = "OS Owned Semaphore", os_owned, 1);
    bit_set!(doc = "OS Owned Semaphore", set_os_owned, 1);
    bit_clear!(doc = "OS Owned Semaphore", clear_os_owned, 1);

    bit_get!(doc = "BIOS Owned Semaphore", bios_owned, 1);
    bit_set!(doc = "BIOS Owned Semaphore", set_bios_owned, 1);
    bit_clear!(doc = "BIOS Owned Semaphore", clear_bios_owned, 1);
}

#[repr(packed)]
pub struct Hba {
    /// Host capability (0x00)
    pub cap: HBACapability,
    /// Global host control (0x04)
    pub ghc: GlobalHBAControl,
    /// Interrupt status (0x08)
    pub is: InterruptStatus,
    /// Port implemented (0x0C)
    pub pi: ImplementedPorts,
    /// Version (0x10)
    pub vs: HBAVersion,
    /// Command completion coalescing control (0x14)
    pub ccc_ctl: CommandCompletionControl,
    /// Command completion coalescing ports (0x18)
    pub ccc_pts: CommandCompletionPorts,
    /// Enclosure management location (0x1C)
    pub em_loc: EnclosureLocation,
    /// Enclosure management control (0x20)
    pub em_ctl: EnclosureControl,
    /// Host capabilities extended (0x24)
    pub cap2: ExtendedCaps,
    /// BIOS/OS handoff control and status (0x28)
    pub bohc: BiosHandOffStatus,
    /// Reserved (0x2C - 0x9F)
    pub rsv: [u8; 116],
    /// Vendor specific registers (0xA0 - 0xFF)
    pub vendor: [u8; 96],
    /// Port control registers (0x100 - 0x10FF)
    pub ports: [HbaPort; 32],
}

impl fmt::Debug for Hba {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(write!(f, "HBA:\n"));
        try!(write!(f, "cap: {}\n", self.cap.0));
        try!(write!(f, "ghc: {}\n", self.ghc.0));
        try!(write!(f, "is: {}\n", self.is.0));
        try!(write!(f, "pi: {}\n", self.pi.0));
        try!(write!(f, "vs: {}\n", self.vs.0));
        try!(write!(f, "ccc_ctl: {}\n", self.ccc_ctl.0));
        try!(write!(f, "ccc_pts: {}\n", self.ccc_pts.0));
        try!(write!(f, "em_loc: {}\n", self.em_loc.0));
        try!(write!(f, "em_ctl: {}\n", self.em_ctl.0));
        try!(write!(f, "cap2: {}\n", self.cap2.0));
        write!(f, "bohc: {}\n", self.bohc.0)
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct HbaPort {
    /// command list base address, 1K-byte aligned (0x00)
    pub clb: u64,
    /// FIS base address, 256-byte aligned (0x08)
    pub fb: u64,
    /// interrupt status (0x10)
    pub is: u32,
    /// interrupt enable (0x14)
    pub ie: u32,
    /// command and status (0x18)
    pub cmd: u32,
    /// Reserved (0x1C)
    pub rsv0: u32,
    /// task file data (0x20)
    pub tfd: u32,
    /// signature (0x24)
    pub sig: u32,
    /// SATA status (SCR0:SStatus) (0x28)
    pub ssts: u32,
    /// SATA control (SCR2:SControl) (0x2C)
    pub sctl: u32,
    /// SATA error (SCR1:SError) (0x30)
    pub serr: u32,
    /// SATA active (SCR3:SActive) (0x34)
    pub sact: u32,
    /// command issue (0x38)
    pub ci: u32,
    /// SATA notification (SCR4:SNotification) (0x3C)
    pub sntf: u32,
    /// FIS-based switch control (0x40)
    pub fbs: u32,
    /// Reserved (0x44 - 0x6F)
    pub rsv1: [u32; 11],
    /// Vendor specific (0x70 - 0x7F)
    pub vendor: [u32; 4],
}

#[derive(Debug)]
pub enum HbaPortType {
    None,
    Unknown(u32),
    SATA,
    SATAPI,
    PM,
    SEMB,
}


impl HbaPort {
    pub fn probe(&self) -> HbaPortType {
        if self.ssts == HBA_SSTS_PRESENT {
            let sig = self.sig;
            match sig {
                HBA_SIG_ATA => HbaPortType::SATA,
                HBA_SIG_ATAPI => HbaPortType::SATAPI,
                HBA_SIG_PM => HbaPortType::PM,
                HBA_SIG_SEMB => HbaPortType::SEMB,
                _ => HbaPortType::Unknown(sig),
            }
        } else {
            HbaPortType::None
        }
    }
}

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
}
