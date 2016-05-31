use std::fmt;

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
pub struct HbaInterruptStatus(u32);

impl HbaInterruptStatus {

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
    pub is: HbaInterruptStatus,
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

impl Hba {
    pub fn get_port(&self, port: usize) -> &HbaPort {
        assert!(port < 32);
        assert!(port < self.cap.ports() as usize + 1);
        assert!(self.pi.is_implemented(port));

        &self.ports[port]
    }

    pub fn get_port_mut(&mut self, port: usize) -> &mut HbaPort {
        assert!(port < 32);
        assert!(port < self.cap.ports() as usize + 1);
        assert!(self.pi.is_implemented(port));

        &mut self.ports[port]
    }

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
    pub is: PortInterruptStatus,
    /// interrupt enable (0x14)
    pub ie: InterruptEnable,
    /// command and status (0x18)
    pub cmd: CommandAndStatus,
    /// Reserved (0x1C)
    pub rsv0: u32,
    /// task file data (0x20)
    pub tfd: TaskFileData,
    /// signature (0x24)
    pub sig: PortSignature,
    /// SATA status (SCR0:SStatus) (0x28)
    pub ssts: SerialAtaStatus,
    /// SATA control (SCR2:SControl) (0x2C)
    pub sctl: SerialAtaControl,
    /// SATA error (SCR1:SError) (0x30)
    pub serr: SerialAtaError,
    /// SATA active (SCR3:SActive) (0x34)
    pub sact: SerialAtaActive,
    /// command issue (0x38)
    pub ci: CommandsIssued,
    /// SATA notification (SCR4:SNotification) (0x3C)
    pub sntf: SerialAtaNotification,
    /// FIS-based switch control (0x40)
    pub fbs: FisSwitchingControl,
    /// Port x Device Sleep (0x44)
    pub devslp: DeviceSleep,
    /// Reserved (0x48 - 0x6F)
    pub rsv1: [u32; 10],
    /// Vendor specific (0x70 - 0x7F)
    pub vendor: [u32; 4],
}

const HBA_SSTS_PRESENT: u8 = 0x3;
const HBA_SIG_ATA: u32 = 0x00000101;
const HBA_SIG_ATAPI: u32 = 0xEB140101;
const HBA_SIG_PM: u32 = 0x96690101;
const HBA_SIG_SEMB: u32 = 0xC33C0101;

#[derive(Debug)]
pub enum HbaPortType {
    None,
    SATA,
    SATAPI,
    PM,
    SEMB,
    Unknown(u32),
}

impl HbaPort {

    pub fn start(&mut self) {
        while self.cmd.command_list_running() {}
        self.cmd.start();
    }

    pub fn stop(&mut self) {
        self.cmd.stop();
        while self.cmd.command_list_running() {}
    }

    pub fn reset(&mut self) {
        self.stop();

        self.sctl.set_device_detection_init(0x1);
        // TODO: wait 1 ms
        self.sctl.set_device_detection_init(0x3);
        self.serr.0 = u32::max_value();

        self.start();
    }

    pub fn probe(&self) -> HbaPortType {
        if self.ssts.device_detection() == HBA_SSTS_PRESENT {
            match self.sig.0 {
                HBA_SIG_ATA => HbaPortType::SATA,
                HBA_SIG_ATAPI => HbaPortType::SATAPI,
                HBA_SIG_PM => HbaPortType::PM,
                HBA_SIG_SEMB => HbaPortType::SEMB,
                _ => HbaPortType::Unknown(self.sig.0),
            }
        } else {
            HbaPortType::None
        }
    }

}

#[derive(Debug)]
pub struct PortInterruptStatus(u32);

impl PortInterruptStatus {
    bit_get!(doc = "Cold Port Detect Status", cold_port_detected, 31);
    bit_clear!(doc = "Clear Cold Port Detect Status", clear_cold_port_detected, 31);

    bit_get!(doc = "Task File Error Status", task_file_error, 30);
    bit_clear!(doc = "Clear Task File Error Status", clear_task_file_error, 30);

    bit_get!(doc = "Host Bus Fatal Error Status", host_bus_fatal_error, 29);
    bit_clear!(doc = "Clear Host Bus Fatal Error Status", clear_host_bus_fatal_error, 29);

    bit_get!(doc = "Host Bus Data Error Status", host_bus_data_error, 28);
    bit_clear!(doc = "Clear Host Bus Data Error Status", clear_host_bus_data_error, 28);

    bit_get!(doc = "Interface Fatal Error Status", interface_fatal_error, 27);
    bit_clear!(doc = "Clear Interface Fatal Error Status", clear_interface_fatal_error, 27);

    bit_get!(doc = "Interface Non-fatal Error Status", interface_non_fatal_error, 26);
    bit_clear!(doc = "Clear Interface Non-fatal Error Status", clear_interface_non_fatal_error, 26);

    bit_get!(doc = "Overflow Status", overflow, 24);
    bit_clear!(doc = "Clear Overflow Status", clear_overflow, 24);

    bit_get!(doc = "Incorrect Port Multiplier Status", incorrect_port_multiplier, 23);
    bit_clear!(doc = "Clear Incorrect Port Multiplier Status", clear_incorrect_port_multiplier, 23);

    bit_get!(doc = "PhyRdy Change Status", phy_ready, 22);

    bit_get!(doc = "Device Mechanical Presence Status", device_mechanical_presence, 7);
    bit_clear!(doc = "Clear Device Mechanical Presence Status", clear_device_mechanical_presence, 7);

    bit_get!(doc = "Port Connect Change Status", port_connect_change, 6);

    bit_get!(doc = "Descriptor Processed", descriptor_processed, 5);
    bit_clear!(doc = "Clear Descriptor Processed", clear_descriptor_processed, 5);

    bit_get!(doc = "Unknown FIS Interrupt", unknown_fis_interrupt, 4);

    bit_get!(doc = "Received a Set Device Bits Interrupt", set_device_bit_interrupt, 3);
    bit_clear!(doc = "Clear Set Device Bits Interrupt", clear_set_device_bit_interrupt, 3);

    bit_get!(doc = "DMA Setup FIS Interrupt", dma_setup_fis_interrupt, 2);
    bit_clear!(doc = "Clear DMA Setup FIS Interrupt", clear_dma_setup_fis_interrupt, 2);

    bit_get!(doc = "PIO Setup FIS Interrupt", pio_setup_fis_interrupt, 1);
    bit_clear!(doc = "Clear PIO Setup FIS Interrupt", clear_pio_setup_fis_interrupt, 1);

    bit_get!(doc = "Device to Host Register FIS Interrupt", d2h_register_fis_interrupt, 0);
    bit_clear!(doc = "Clear Device to Host Register FIS Interrupt", clear_d2h_register_fis_interrupt, 0);
}

#[derive(Debug)]
pub struct InterruptEnable(u32);

impl InterruptEnable {
    bit_get!(doc = "Cold Presence Detect", cold_presence_detect, 31);
    bit_set!(doc = "Enable Cold Presence Detect", enable_cold_presence_detect, 31);
    bit_clear!(doc = "Disable Cold Presence Detect", disable_cold_presence_detect, 31);

    bit_get!(doc = "Task File Error", task_file_error, 30);
    bit_set!(doc = "Enable Task File Error", enable_task_file_error, 30);
    bit_clear!(doc = "Disable Task File Error", disable_task_file_error, 30);

    bit_get!(doc = "Host Bus Fatal Error", host_bus_fatal_error, 29);
    bit_set!(doc = "Enable Host Bus Fatal Error", enable_host_bus_fatal_error, 29);
    bit_clear!(doc = "Disable Host Bus Fatal Error", disable_host_bus_fatal_error, 29);

    bit_get!(doc = "Host Bus data Error", host_bus_data_error, 28);
    bit_set!(doc = "Enable Host Bus data Error", enable_host_bus_data_error, 28);
    bit_clear!(doc = "Disable Host Bus data Error", disable_host_bus_data_error, 28);

    bit_get!(doc = "Interface Fatal Error", interface_fatal_error, 27);
    bit_set!(doc = "Enable Interface Fatal Error", enable_interface_fatal_error, 27);
    bit_clear!(doc = "Disable Interface Fatal Error", disable_interface_fatal_error, 27);

    bit_get!(doc = "Interface Non-fatal Error", interface_non_fatal_error, 26);
    bit_set!(doc = "Enable Interface Non-fatal Error", enable_interface_non_fatal_error, 26);
    bit_clear!(doc = "Disable Interface Non-fatal Error", disable_interface_non_fatal_error, 26);

    bit_get!(doc = "Overflow", overflow, 24);
    bit_set!(doc = "Enable Overflow", enable_overflow, 24);
    bit_clear!(doc = "Disable Overflow", disable_overflow, 24);

    bit_get!(doc = "Incorrect Port Multiplier", incorrect_port_multiplier, 23);
    bit_set!(doc = "Enable Incorrect Port Multiplier", enable_incorrect_port_multiplier, 23);
    bit_clear!(doc = "Disable Incorrect Port Multiplier", disable_incorrect_port_multiplier, 23);

    bit_get!(doc = "PyRdy Change Interrupt", pyrdy_change_irq, 22);
    bit_set!(doc = "Enable PyRdy Change Interrupt", enable_pyrdy_change_irq, 22);
    bit_clear!(doc = "Disable PyRdy Change Interrupt", disable_pyrdy_change_irq, 22);

    bit_get!(doc = "Device Mechanical Presence", device_mechanical_presence, 7);
    bit_set!(doc = "Enable Device Mechanical Presence", enable_device_mechanical_presence, 7);
    bit_clear!(doc = "Disable Device Mechanical Presence", disable_device_mechanical_presence, 7);

    bit_get!(doc = "Port Change Interrupt", port_change_irq, 6);
    bit_set!(doc = "Enable Port Change Interrupt", enable_port_change_irq, 6);
    bit_clear!(doc = "Disable Port Change Interrupt", disable_port_change_irq, 6);

    bit_get!(doc = "Descriptor Processed Interrupt", descriptor_processed, 5);
    bit_set!(doc = "Enable Descriptor Processed Interrupt", enable_descriptor_processed, 5);
    bit_clear!(doc = "Disable Descriptor Processed Interrupt", disable_descriptor_processed, 5);

    bit_get!(doc = "Unknown FIS Interrupt", unknown_fis_irq, 4);
    bit_set!(doc = "Enable Unknown FIS Interrupt", enable_unknown_fis_irq, 4);
    bit_clear!(doc = "Disable Unknown FIS Interrupt", disable_unknown_fis_irq, 4);

    bit_get!(doc = "Set Device Bits FIS Interrupt", device_bits_fis_irq, 3);
    bit_set!(doc = "Enable Set Device Bits FIS Interrupt", enable_device_bits_fis_irq, 3);
    bit_clear!(doc = "Disable Set Device Bits FIS Interrupt", disable_device_bits_fis_irq, 3);

    bit_get!(doc = "DMA Setup FIS Interrupt", dma_setup_fis_irq, 2);
    bit_set!(doc = "Enable DMA Setup FIS Interrupt", enable_dma_setup_fis_irq, 2);
    bit_clear!(doc = "Disable DMA Setup FIS Interrupt", disable_dma_setup_fis_irq, 2);

    bit_get!(doc = "PIO Setup FIS Interrupt", pio_setup_irq, 1);
    bit_set!(doc = "Enable PIO Setup FIS Interrupt", enable_pio_setup_irq, 1);
    bit_clear!(doc = "Disable PIO Setup FIS Interrupt", disable_pio_setup_irq, 1);

    bit_get!(doc = "Device to host Register FIS Interrupt", d2h_register_fis_interrupt, 0);
    bit_set!(doc = "Enable Device to host Register FIS Interrupt", enable_d2h_register_fis_interrupt, 0);
    bit_clear!(doc = "Disable Device to host Register FIS Interrupt", disable_d2h_register_fis_interrupt, 0);
}

#[derive(Debug)]
pub struct CommandAndStatus(u32);

impl CommandAndStatus {

    /// Interface Communication Control
    pub fn icc(&self) -> u8 {
        bits_get(self.0, 28, 31) as u8
    }

    /// Set Interface Communication Control
    pub fn set_icc(&self) {
        unreachable!();
    }

    bit_get!(doc = "Aggressive Slumber / Partial (ASP)", aggressive_slumber, 27);
    bit_set!(doc = "Set Aggressive Slumber / Partial (ASP)", set_aggressive_slumber, 27);

    bit_get!(doc = "Aggressive Link Power Management Enable (ALPE)", aggressive_link_power_mgmt, 26);
    bit_set!(doc = "Set Aggressive Link Power Management Enable (ALPE)", set_aggressive_link_power_mgmt, 26);

    bit_get!(doc = "Drive LED on ATAPI Enable (DLAE)", drive_led_on_atapi, 25);
    bit_set!(doc = "Set Drive LED on ATAPI Enable (DLAE)", set_drive_led_on_atapi, 25);

    bit_get!(doc = "Device is ATAPI", device_is_atapi, 24);
    bit_set!(doc = "Set Device is ATAPI", set_device_is_atapi, 24);

    bit_get!(doc = "Automatic Partial to Slumber Transitions Enabled (APSTE)", automatic_partial_slumber_transitions, 23);
    bit_set!(doc = "Set Automatic Partial to Slumber Transitions Enabled (APSTE)", set_automatic_partial_slumber_transitions, 23);

    bit_get!(doc = "FIS-based Switching Capable Port", fis_switching_port, 22);
    bit_get!(doc = "External SATA Port", external_sata_port, 21);
    bit_get!(doc = "Cold Presence Detection", cold_presence_detection, 20);
    bit_get!(doc = "Mechanical Presence Switch Attached to Port", mechanical_presence_switch_attached, 19);
    bit_get!(doc = "Hot Plug Capable Port", hot_plug_capable_port, 18);

    bit_get!(doc = "Port Multiplier Attached", port_multiplier_attached, 17);
    bit_set!(doc = "Set Port Multiplier Attached", set_port_multiplier_attached, 17);
    bit_clear!(doc = "Clear Port Multiplier Attached", clear_port_multiplier_attached, 17);

    bit_get!(doc = "Cold Presence State", cold_presence, 16);
    bit_get!(doc = "Command List Running (CR)", command_list_running, 15);
    bit_get!(doc = "FIS Receive Running", fis_receive_running, 14);
    bit_get!(doc = "Mechanical Presence Switch State", mechanical_presence_switch_state, 13);

    /// Current Command Slot
    pub fn ccs(&self) -> u8 {
        bits_get(self.0, 8, 12) as u8
    }

    bit_get!(doc = "FIS Receive", fis_receive, 4);
    bit_set!(doc = "Set FIS Receive", enable_fis_receive, 4);
    bit_clear!(doc = "Clear FIS Receive", disable_fis_receive, 4);

    bit_get!(doc = "Command List Override", command_list_override, 3);
    bit_set!(doc = "Set Command List Override", clear_command_list_override, 3);

    bit_get!(doc = "Power On Device", power_on_device, 2);
    bit_set!(doc = "Set Power On Device", enable_power_on_device, 2);
    bit_clear!(doc = "Clear Power On Device", disable_power_on_device, 2);

    bit_get!(doc = "Spin-Up Device", spin_up_device, 1);
    bit_set!(doc = "Set Spin-Up Device", enable_spin_up_device, 1);
    bit_clear!(doc = "Clear Spin-Up Device", disable_spin_up_device, 1);

    bit_get!(doc = "Is Started?", is_started, 0);
    bit_set!(doc = "Set Start", start, 0);
    bit_clear!(doc = "Stop", stop, 0);
}

#[derive(Debug)]
pub struct TaskFileData(u32);

impl TaskFileData {
    pub fn error(&self) -> u8 {
        bits_get(self.0, 8, 15) as u8
    }

    pub fn status(&self) -> u8 {
        bits_get(self.0, 0, 7) as u8
    }
}

#[derive(Debug)]
pub struct PortSignature(u32);

impl PortSignature {
    pub fn lba(&self) -> u32 {
        bits_get(self.0, 8, 31) << 8
    }

    pub fn sectors(&self) -> u8 {
        bits_get(self.0, 0, 7) as u8
    }
}

#[derive(Debug)]
pub struct SerialAtaStatus(u32);

impl SerialAtaStatus {
    pub fn power_management(&self) -> u8 {
        // TOOD: enum
        bits_get(self.0, 8, 11) as u8
    }

    pub fn current_speed(&self) -> u8 {
        // TODO: enum
        bits_get(self.0, 4, 7) as u8
    }

    pub fn device_detection(&self) -> u8 {
        // TODO: enum
        bits_get(self.0, 0, 3) as u8
    }
}


#[derive(Debug)]
pub struct SerialAtaControl(u32);

impl SerialAtaControl {
    pub fn allowed_ipm_transitions(&self) -> u8 {
        // TODO: Should be enum
        bits_get(self.0, 8, 11) as u8
    }

    pub fn set_allowed_ipm_transitions(&self, transitions: u8) {
        unreachable!();
    }

    pub fn allowed_speed(&self) -> u8 {
        // TODO: Should be enum
        bits_get(self.0, 4, 7) as u8
    }

    pub fn set_allowed_speed(&self, speed: u8) {
        unreachable!();
    }

    pub fn device_detection_init(&self) -> u8 {
        bits_get(self.0, 0, 3) as u8
    }

    pub fn set_device_detection_init(&self, mode: u8) {
        unreachable!();
    }
}

#[derive(Debug)]
pub struct SerialAtaError(u32);

impl SerialAtaError {

    pub fn diagnostics(&self) -> u16 {
        bits_get(self.0, 16, 31) as u16
    }

    pub fn error(&self) -> u16 {
        bits_get(self.0, 0, 15) as u16
    }

}

#[derive(Debug)]
pub struct SerialAtaActive(u32);

impl SerialAtaActive {

    pub fn device_status(&self, slot: u8) -> bool {
        is_bit_set!(self.0, slot)
    }

    pub fn set_device_status(&self, slot: u8) {
        unreachable!();
    }

}


#[derive(Debug)]
pub struct CommandsIssued(u32);

impl CommandsIssued {

    pub fn commands_issued(&self, slot: u8) -> bool {
        is_bit_set!(self.0, slot)
    }

    pub fn set_commands_issued(&self, slot: u8) {
        unreachable!();
    }

}

#[derive(Debug)]
pub struct SerialAtaNotification(u32);

impl SerialAtaNotification {

    /// PM Notify (PMN)
    ///
    /// This field indicates whether a particular device with the corresponding
    /// PM Port number issued a Set Device Bits FIS to the host with the Notification bit set.
    pub fn pm_notify(&self, port: u8) -> bool {
        is_bit_set!(self.0, port)
    }

    pub fn clear_pm_notify(&mut self, port: u8) {
        self.0 |= 1 << port;
    }
}


#[derive(Debug)]
pub struct FisSwitchingControl(u32);

impl FisSwitchingControl {

    /// Device With Error
    pub fn device_with_error(&self) -> u8 {
        bits_get(self.0, 16, 19) as u8
    }

    /// Active Device Optimization
    pub fn active_device_optimization(&self) -> u8 {
        bits_get(self.0, 12, 15) as u8
    }

    /// Device To Issue
    pub fn device_to_issue(&self) -> u8 {
        bits_get(self.0, 8, 11) as u8
    }

    pub fn set_device_to_issue(&self, port: u8) {
        unreachable!()
    }

    bit_get!(doc = "Single Device Error", single_device_error, 2);
    bit_set!(doc = "Device Error Clear", device_clear_error, 1);

    bit_get!(doc = "Enabled", is_enabled, 0);
    bit_set!(doc = "Enable FIS-based switching", enable, 0);
    bit_clear!(doc = "Disable FIS-based switching", disable, 0);
}

#[derive(Debug)]
pub struct DeviceSleep(u32);

impl DeviceSleep {

    /// DITO Multiplier
    pub fn dito_multiplier(&self) -> u8 {
        bits_get(self.0, 25, 28) as u8
    }

    /// Device Sleep Idle Timeout (DITO)
    pub fn dito(&self) -> u8 {
        bits_get(self.0, 15, 24) as u8
    }

    /// Minimum Device Sleep Assertion Time (MDAT)
    pub fn mdat(&self) -> u8 {
        bits_get(self.0, 10, 14) as u8
    }

    /// Device Sleep Exit Timeout (DETO)
    pub fn deto(&self) -> u8 {
        bits_get(self.0, 2, 9) as u8
    }

    bit_get!(doc = "Device Sleep Present (DSP)", has_device_sleep, 1);

    bit_get!(doc = "Aggressive Device Sleep (ADSE)", aggressive_device_sleep, 0);
    bit_set!(doc = "Aggressive Device Sleep Enable (ADSE)", enable_aggressive_device_sleep, 0);
    bit_clear!(doc = "Aggressive Device Sleep Disable (ADSE)", disable_aggressive_device_sleep, 0);
}
