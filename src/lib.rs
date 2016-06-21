#[macro_use]
extern crate driverkit;
extern crate core;

pub mod fis;

use std::fmt;
use std::mem;
use std::thread; // sleep()
use std::time::Duration; // sleep()

use driverkit::Volatile;
use driverkit::bitops::*;
use driverkit::timedops::wait_until;

#[repr(packed)]
pub struct HBACapability(Volatile<u32>);

impl HBACapability {
    bit_get_fn!(doc = "Supports 64-bit Addressing", has_64bit_addressing, 31);
    bit_get_fn!(doc = "Supports Native Command Queuing",
                has_native_command_queing,
                30);
    bit_get_fn!(doc = "Supports SNotification Register",
                has_snotification_register,
                29);
    bit_get_fn!(doc = "Supports Mechanical Presence Switch",
                has_mechanical_presence_switch,
                28);
    bit_get_fn!(doc = "Supports Staggered Spin-up",
                has_staggered_spin_up,
                27);
    bit_get_fn!(doc = "Supports Aggressive Link Power Management",
                has_aggressive_link_power_mgmt,
                26);
    bit_get_fn!(doc = "Supports Activity LED", has_activity_led, 25);
    bit_get_fn!(doc = "Supports Command List Override",
                has_command_list_override,
                24);
    bit_get_fn!(doc = "Supports AHCI mode only", has_ahci_mode_only, 18);
    bit_get_fn!(doc = "Supports Port Multiplier", has_port_multiplier, 17);
    bit_get_fn!(doc = "FIS-based Switching Supported", has_fis_switching, 16);
    bit_get_fn!(doc = "PIO Multiple DRQ Block", has_multiple_drq_blocks, 15);
    bit_get_fn!(doc = "Slumber State Capable", has_slumber_state, 14);
    bit_get_fn!(doc = "Partial State Capable", has_partial_state, 13);
    bit_get_fn!(doc = "Command Completion Coalescing Supported",
                has_cmd_completion_coalescing,
                7);
    bit_get_fn!(doc = "Enclosure Management Supported",
                has_enclosure_mgmt,
                6);
    bit_get_fn!(doc = "Supports External SATA", has_external_stat, 5);

    /// Number of Command Slots (NCS)
    pub fn command_slots(&self) -> u32 {
        bits_get(&self.0, 8, 12)
    }

    /// Interface Speed Support (ISS)
    pub fn interface_speed(&self) -> u32 {
        bits_get(&self.0, 20, 23)
    }

    /// Number of Ports (NP)
    pub fn ports(&self) -> u32 {
        bits_get(&self.0, 0, 4)
    }
}

impl fmt::Debug for HBACapability {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(write!(f, "HBACapability:\n"));
        try!(write!(f,
                    "\tSupports 64-bit Addressing: {}\n",
                    self.has_64bit_addressing()));
        try!(write!(f,
                    "\tSupports Native Command Queuing: {}\n",
                    self.has_native_command_queing()));
        try!(write!(f,
                    "\tSupports SNotification Register: {}\n",
                    self.has_snotification_register()));
        try!(write!(f,
                    "\tSupports Mechanical Presence Switch: {}\n",
                    self.has_mechanical_presence_switch()));
        try!(write!(f,
                    "\tSupports Staggered Spin-up: {}\n",
                    self.has_staggered_spin_up()));
        try!(write!(f,
                    "\tSupports Aggressive Link Power Management: {}\n",
                    self.has_aggressive_link_power_mgmt()));
        try!(write!(f, "\tSupports Activity LED: {}\n", self.has_activity_led()));
        try!(write!(f,
                    "\tSupports Command List Override: {}\n",
                    self.has_command_list_override()));
        try!(write!(f,
                    "\tSupports AHCI mode only: {}\n",
                    self.has_ahci_mode_only()));
        try!(write!(f,
                    "\tSupports Port Multiplier: {}\n",
                    self.has_port_multiplier()));
        try!(write!(f,
                    "\tFIS-based Switching Supported: {}\n",
                    self.has_fis_switching()));
        try!(write!(f,
                    "\tPIO Multiple DRQ Block: {}\n",
                    self.has_multiple_drq_blocks()));
        try!(write!(f, "\tSlumber State Capable: {}\n", self.has_slumber_state()));
        try!(write!(f, "\tPartial State Capable: {}\n", self.has_partial_state()));
        try!(write!(f,
                    "\tCommand Completion Coalescing Supported: {}\n",
                    self.has_cmd_completion_coalescing()));
        try!(write!(f,
                    "\tEnclosure Management Supported: {}\n",
                    self.has_enclosure_mgmt()));
        try!(write!(f,
                    "\tSupports External SATA: {}\n",
                    self.has_external_stat()));
        try!(write!(f, "\tNumber of Command Slots: {}\n", self.command_slots()));
        try!(write!(f, "\tInterface Speed Support: {}\n", self.interface_speed()));
        write!(f, "\tPorts: {}\n", self.ports())
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct GlobalHBAControl(Volatile<u32>);

impl GlobalHBAControl {
    bit_get_fn!(doc = "AHCI Enable", ahci, 31);
    bit_set_fn!(doc = "AHCI Enable", set_ahci, 31);

    bit_get_fn!(doc = "MSI Revert to Single Message (MRSM)", mrsm, 2);

    bit_get_fn!(doc = "Are Interrupts Enabled?", interrupts_enabled, 1);
    bit_set_fn!(doc = "Interrupt Enable", enable_interrupts, 1);
    bit_clear_fn!(doc = "Interrupt Disable", disable_interrupts, 1);

    bit_get_fn!(doc = "HBA Reset (HR) in progress", reset_pending, 0);
    bit_set_fn!(doc = "HBA Reset (HR):", reset, 0);
}

#[derive(Debug)]
#[repr(packed)]
pub struct HBAVersion(Volatile<u32>);

impl HBAVersion {
    /// Major Version Number
    pub fn major(&self) -> u16 {
        bits_get(&self.0, 16, 31) as u16
    }

    /// Minor Version Number
    pub fn minor(&self) -> u16 {
        bits_get(&self.0, 0, 15) as u16
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct HbaInterruptStatus(Volatile<u32>);

impl HbaInterruptStatus {
    /// Interrupt Pending Status
    pub fn is_pending(&self, port: usize) -> bool {
        is_bit_set!(self.0, port)
    }

    pub fn clear(&mut self, port: usize) {
        self.0.set(self.0.get() & !(1 << port));
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct ImplementedPorts(Volatile<u32>);

impl ImplementedPorts {
    /// Port Implemented
    pub fn is_implemented(&self, port: usize) -> bool {
        is_bit_set!(self.0, port)
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct CommandCompletionControl(Volatile<u32>);

impl CommandCompletionControl {
    /// Timeout Value
    pub fn timeout(&self) -> u16 {
        bits_get(&self.0, 16, 31) as u16
    }

    /// Set Timeout Value
    pub fn set_timeout(&self, tv: u16) {
        unreachable!()
    }

    /// Command Completions
    pub fn command_completions(&self) -> u8 {
        bits_get(&self.0, 8, 15) as u8
    }

    /// Set Command Completions
    pub fn set_command_completions(&self, cc: u8) {
        unreachable!()
    }

    /// Interrupt
    pub fn interupt(&self) -> u8 {
        bits_get(&self.0, 3, 7) as u8
    }

    /// Set Interrupt
    pub fn set_interupt(&self, irq: u8) {
        unreachable!()
    }

    bit_get_fn!(doc = "Is CCC enabled?", enabled, 0);
    bit_set_fn!(doc = "Enable CCC", enable, 0);
    bit_clear_fn!(doc = "Disable CCC", disable, 0);
}

#[derive(Debug)]
#[repr(packed)]
pub struct CommandCompletionPorts(Volatile<u32>);

impl CommandCompletionPorts {
    /// Is port part of the command completion coalescing feature?
    pub fn has_cc(&self, port: u32) -> bool {
        is_bit_set!(self.0, port)
    }

    /// Is port part of the command completion coalescing feature?
    pub fn set_cc(&mut self, port: u32) {
        self.0.set(self.0.get() | 1 << port);
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct EnclosureLocation(Volatile<u32>);

impl EnclosureLocation {
    /// Offset
    pub fn offset(&self) -> u16 {
        bits_get(&self.0, 16, 31) as u16
    }

    /// Buffer size
    pub fn buffer_size(&self) -> u16 {
        bits_get(&self.0, 0, 15) as u16
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct EnclosureControl(Volatile<u32>);

impl EnclosureControl {
    bit_get_fn!(doc = "Port Multiplier Support?", has_port_multiplier, 27);
    bit_get_fn!(doc = "Activity LED Hardware Driven",
                has_hardware_activity_led,
                26);
    bit_get_fn!(doc = "Transmit Only", has_transmit_only, 25);
    bit_get_fn!(doc = "Single Message Buffer", has_single_message_buffer, 24);
    bit_get_fn!(doc = "SGPIO Enclosure Management Messages", has_sgpio, 19);
    bit_get_fn!(doc = "SES-2 Enclosure Management Messages", has_ses2, 18);
    bit_get_fn!(doc = "SAF-TE Enclosure Management Messages", has_safte, 17);
    bit_get_fn!(doc = "LED Message Types", led_types, 16);

    bit_set_fn!(doc = "Reset", reset, 9);
    bit_set_fn!(doc = "Transmit Message", transmit_message, 8);

    bit_get_fn!(doc = "Message Received", message_received, 0);
    bit_set_fn!(doc = "Message Received", set_message_received, 0);
}

#[derive(Debug)]
#[repr(packed)]
pub struct ExtendedCaps(Volatile<u32>);

impl ExtendedCaps {
    bit_get_fn!(doc = "DevSleep Entrance from Slumber Only", has_deso, 5);
    bit_get_fn!(doc = "Supports Aggressive Device Sleep Management",
                has_agressive_sleep_management,
                4);
    bit_get_fn!(doc = "Supports Device Sleep", has_device_sleep, 3);
    bit_get_fn!(doc = "Automatic Partial to Slumber Transitions",
                has_automatic_slumber,
                2);
    bit_get_fn!(doc = "NVMHCI Present", has_nvmhci, 1);
    bit_get_fn!(doc = "BIOS/OS Handoff", has_bios_os_handoff, 0);
}

#[derive(Debug)]
#[repr(packed)]
pub struct BiosHandOffStatus(Volatile<u32>);

impl BiosHandOffStatus {
    bit_get_fn!(doc = "BIOS Busy", is_bios_busy, 4);
    bit_set_fn!(doc = "Set BIOS Busy", set_bios_busy, 4);
    bit_clear_fn!(doc = "Clear BIOS Busy", clear_bios_busy, 4);

    bit_get_fn!(doc = "OS Ownership Change", change_ownership_to_os, 3);
    bit_set_fn!(doc = "Clear OS Ownership Change Status",
                clear_change_ownership_to_os,
                3);

    bit_get_fn!(doc = "SMI on OS Ownership Change",
                has_smi_on_ownership_change,
                2);
    bit_set_fn!(doc = "SMI on OS Ownership Change Enable",
                enable_smi_on_ownership_change,
                2);
    bit_clear_fn!(doc = "SMI on OS Ownership Change Disable",
                  disable_smi_on_ownership_change,
                  2);

    bit_get_fn!(doc = "OS Owned Semaphore", os_owned, 1);
    bit_set_fn!(doc = "OS Owned Semaphore", set_os_owned, 1);
    bit_clear_fn!(doc = "OS Owned Semaphore", clear_os_owned, 1);

    bit_get_fn!(doc = "BIOS Owned Semaphore", bios_owned, 1);
    bit_set_fn!(doc = "BIOS Owned Semaphore", set_bios_owned, 1);
    bit_clear_fn!(doc = "BIOS Owned Semaphore", clear_bios_owned, 1);
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
    ports: [HbaPort; 32],
}

impl Hba {
    pub fn get_port(&self, port: usize) -> &HbaPort {
        assert!(port < 32);
        assert!(port < self.cap.ports() as usize + 1);
        assert!(self.pi.is_implemented(port));

        &self.ports[port]
    }

    /// Get a mutable reference to a port.
    ///
    /// # Unsafe
    ///   * Ensure that this is called only once per port.
    pub unsafe fn get_port_mut(&mut self, port: usize) -> &mut HbaPort {
        assert!(port < 32);
        assert!(port < self.cap.ports() as usize + 1);
        assert!(self.pi.is_implemented(port));

        mem::transmute(&mut self.ports[port])
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
    pub clb: Volatile<u64>,
    /// FIS base address, 256-byte aligned (0x08)
    pub fb: Volatile<u64>,
    /// interrupt status (0x10)
    pub is: PortInterruptStatus,
    /// interrupt enable (0x14)
    pub ie: InterruptEnable,
    /// command and status (0x18)
    pub cmd: CommandAndStatus,
    /// Reserved (0x1C)
    pub rsv0: Volatile<u32>,
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
    pub rsv1: [Volatile<u32>; 10],
    /// Vendor specific (0x70 - 0x7F)
    pub vendor: [Volatile<u32>; 4],
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

    pub fn is_idle(&self) -> bool {
        // If PxCMD.ST, PxCMD.CR, PxCMD.FRE and
        // PxCMD.FR are all cleared, the port is in an idle state.
        !port.cmd.command_list_running() &&
        !port.cmd.is_started() &&
        !port.cmd.fis_receive() &&
        !port.cmd.fis_receive_running() &&

    }

    pub fn start(&mut self) {
        wait_until(|| !self.cmd.command_list_running(),
                   Duration::from_millis(500))
            .unwrap();

        self.cmd.start();
        // self.cmd.enable_fis_receive();
    }

    pub fn stop(&mut self) {
        self.cmd.stop();
        wait_until(|| !self.cmd.command_list_running(),
                   Duration::from_millis(500))
            .expect("Timout while waiting for command list processing to stop");

        if self.cmd.fis_receive() {
            self.cmd.disable_fis_receive();
            wait_until(|| !self.cmd.fis_receive_running(),
                       Duration::from_millis(500))
                .expect("Timeout waiting for FIS receive to stop");
        }

        assert!(!self.cmd.fis_receive_running());
        assert!(!self.cmd.is_started());
    }

    pub fn reset(&mut self) {

        self.sctl.set_device_detection_init(0x1);
        thread::sleep(Duration::from_millis(2));
        self.sctl.set_device_detection_init(0x0);
        while self.ssts.device_detection() != 0x3 {}

        self.serr.clear();
    }

    pub fn is_present(&self) -> bool {
        self.ssts.device_detection() == HBA_SSTS_PRESENT && !self.tfd.busy() && !self.tfd.drq()
    }

    pub fn probe(&self) -> HbaPortType {
        if self.ssts.device_detection() == HBA_SSTS_PRESENT {
            match self.sig.0.get() {
                HBA_SIG_ATA => HbaPortType::SATA,
                HBA_SIG_ATAPI => HbaPortType::SATAPI,
                HBA_SIG_PM => HbaPortType::PM,
                HBA_SIG_SEMB => HbaPortType::SEMB,
                _ => HbaPortType::Unknown(self.sig.0.get()),
            }
        } else {
            HbaPortType::None
        }
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct PortInterruptStatus(Volatile<u32>);

impl PortInterruptStatus {
    bit_get_fn!(doc = "Cold Port Detect Status", cold_port_detected, 31);
    bit_clear_fn!(doc = "Clear Cold Port Detect Status",
                  clear_cold_port_detected,
                  31);

    bit_get_fn!(doc = "Task File Error Status", task_file_error, 30);
    bit_clear_fn!(doc = "Clear Task File Error Status",
                  clear_task_file_error,
                  30);

    bit_get_fn!(doc = "Host Bus Fatal Error Status",
                host_bus_fatal_error,
                29);
    bit_clear_fn!(doc = "Clear Host Bus Fatal Error Status",
                  clear_host_bus_fatal_error,
                  29);

    bit_get_fn!(doc = "Host Bus Data Error Status", host_bus_data_error, 28);
    bit_clear_fn!(doc = "Clear Host Bus Data Error Status",
                  clear_host_bus_data_error,
                  28);

    bit_get_fn!(doc = "Interface Fatal Error Status",
                interface_fatal_error,
                27);
    bit_clear_fn!(doc = "Clear Interface Fatal Error Status",
                  clear_interface_fatal_error,
                  27);

    bit_get_fn!(doc = "Interface Non-fatal Error Status",
                interface_non_fatal_error,
                26);
    bit_clear_fn!(doc = "Clear Interface Non-fatal Error Status",
                  clear_interface_non_fatal_error,
                  26);

    bit_get_fn!(doc = "Overflow Status", overflow, 24);
    bit_clear_fn!(doc = "Clear Overflow Status", clear_overflow, 24);

    bit_get_fn!(doc = "Incorrect Port Multiplier Status",
                incorrect_port_multiplier,
                23);
    bit_clear_fn!(doc = "Clear Incorrect Port Multiplier Status",
                  clear_incorrect_port_multiplier,
                  23);

    bit_get_fn!(doc = "PhyRdy Change Status", phy_ready, 22);

    bit_get_fn!(doc = "Device Mechanical Presence Status",
                device_mechanical_presence,
                7);
    bit_clear_fn!(doc = "Clear Device Mechanical Presence Status",
                  clear_device_mechanical_presence,
                  7);

    bit_get_fn!(doc = "Port Connect Change Status", port_connect_change, 6);

    bit_get_fn!(doc = "Descriptor Processed", descriptor_processed, 5);
    bit_clear_fn!(doc = "Clear Descriptor Processed",
                  clear_descriptor_processed,
                  5);

    bit_get_fn!(doc = "Unknown FIS Interrupt", unknown_fis_interrupt, 4);

    bit_get_fn!(doc = "Received a Set Device Bits Interrupt",
                set_device_bit_interrupt,
                3);
    bit_clear_fn!(doc = "Clear Set Device Bits Interrupt",
                  clear_set_device_bit_interrupt,
                  3);

    bit_get_fn!(doc = "DMA Setup FIS Interrupt", dma_setup_fis_interrupt, 2);
    bit_clear_fn!(doc = "Clear DMA Setup FIS Interrupt",
                  clear_dma_setup_fis_interrupt,
                  2);

    bit_get_fn!(doc = "PIO Setup FIS Interrupt", pio_setup_fis_interrupt, 1);
    bit_clear_fn!(doc = "Clear PIO Setup FIS Interrupt",
                  clear_pio_setup_fis_interrupt,
                  1);

    bit_get_fn!(doc = "Device to Host Register FIS Interrupt",
                d2h_register_fis_interrupt,
                0);
    bit_clear_fn!(doc = "Clear Device to Host Register FIS Interrupt",
                  clear_d2h_register_fis_interrupt,
                  0);
}

#[derive(Debug)]
#[repr(packed)]
pub struct InterruptEnable(Volatile<u32>);

impl InterruptEnable {
    pub fn enable_all(&mut self) {
        self.0.set(u32::max_value());
    }

    pub fn disable_all(&mut self) {
        self.0.set(0);
    }

    bit_get_fn!(doc = "Cold Presence Detect", cold_presence_detect, 31);
    bit_set_fn!(doc = "Enable Cold Presence Detect",
                enable_cold_presence_detect,
                31);
    bit_clear_fn!(doc = "Disable Cold Presence Detect",
                  disable_cold_presence_detect,
                  31);

    bit_get_fn!(doc = "Task File Error", task_file_error, 30);
    bit_set_fn!(doc = "Enable Task File Error", enable_task_file_error, 30);
    bit_clear_fn!(doc = "Disable Task File Error", disable_task_file_error, 30);

    bit_get_fn!(doc = "Host Bus Fatal Error", host_bus_fatal_error, 29);
    bit_set_fn!(doc = "Enable Host Bus Fatal Error",
                enable_host_bus_fatal_error,
                29);
    bit_clear_fn!(doc = "Disable Host Bus Fatal Error",
                  disable_host_bus_fatal_error,
                  29);

    bit_get_fn!(doc = "Host Bus data Error", host_bus_data_error, 28);
    bit_set_fn!(doc = "Enable Host Bus data Error",
                enable_host_bus_data_error,
                28);
    bit_clear_fn!(doc = "Disable Host Bus data Error",
                  disable_host_bus_data_error,
                  28);

    bit_get_fn!(doc = "Interface Fatal Error", interface_fatal_error, 27);
    bit_set_fn!(doc = "Enable Interface Fatal Error",
                enable_interface_fatal_error,
                27);
    bit_clear_fn!(doc = "Disable Interface Fatal Error",
                  disable_interface_fatal_error,
                  27);

    bit_get_fn!(doc = "Interface Non-fatal Error",
                interface_non_fatal_error,
                26);
    bit_set_fn!(doc = "Enable Interface Non-fatal Error",
                enable_interface_non_fatal_error,
                26);
    bit_clear_fn!(doc = "Disable Interface Non-fatal Error",
                  disable_interface_non_fatal_error,
                  26);

    bit_get_fn!(doc = "Overflow", overflow, 24);
    bit_set_fn!(doc = "Enable Overflow", enable_overflow, 24);
    bit_clear_fn!(doc = "Disable Overflow", disable_overflow, 24);

    bit_get_fn!(doc = "Incorrect Port Multiplier",
                incorrect_port_multiplier,
                23);
    bit_set_fn!(doc = "Enable Incorrect Port Multiplier",
                enable_incorrect_port_multiplier,
                23);
    bit_clear_fn!(doc = "Disable Incorrect Port Multiplier",
                  disable_incorrect_port_multiplier,
                  23);

    bit_get_fn!(doc = "PyRdy Change Interrupt", phyrdy_change_irq, 22);
    bit_set_fn!(doc = "Enable PyRdy Change Interrupt",
                enable_phyrdy_change_irq,
                22);
    bit_clear_fn!(doc = "Disable PyRdy Change Interrupt",
                  disable_phyrdy_change_irq,
                  22);

    bit_get_fn!(doc = "Device Mechanical Presence",
                device_mechanical_presence,
                7);
    bit_set_fn!(doc = "Enable Device Mechanical Presence",
                enable_device_mechanical_presence,
                7);
    bit_clear_fn!(doc = "Disable Device Mechanical Presence",
                  disable_device_mechanical_presence,
                  7);

    bit_get_fn!(doc = "Port Change Interrupt", port_change_irq, 6);
    bit_set_fn!(doc = "Enable Port Change Interrupt",
                enable_port_change_irq,
                6);
    bit_clear_fn!(doc = "Disable Port Change Interrupt",
                  disable_port_change_irq,
                  6);

    bit_get_fn!(doc = "Descriptor Processed Interrupt",
                descriptor_processed,
                5);
    bit_set_fn!(doc = "Enable Descriptor Processed Interrupt",
                enable_descriptor_processed,
                5);
    bit_clear_fn!(doc = "Disable Descriptor Processed Interrupt",
                  disable_descriptor_processed,
                  5);

    bit_get_fn!(doc = "Unknown FIS Interrupt", unknown_fis_irq, 4);
    bit_set_fn!(doc = "Enable Unknown FIS Interrupt",
                enable_unknown_fis_irq,
                4);
    bit_clear_fn!(doc = "Disable Unknown FIS Interrupt",
                  disable_unknown_fis_irq,
                  4);

    bit_get_fn!(doc = "Set Device Bits FIS Interrupt",
                device_bits_fis_irq,
                3);
    bit_set_fn!(doc = "Enable Set Device Bits FIS Interrupt",
                enable_device_bits_fis_irq,
                3);
    bit_clear_fn!(doc = "Disable Set Device Bits FIS Interrupt",
                  disable_device_bits_fis_irq,
                  3);

    bit_get_fn!(doc = "DMA Setup FIS Interrupt", dma_setup_fis_irq, 2);
    bit_set_fn!(doc = "Enable DMA Setup FIS Interrupt",
                enable_dma_setup_fis_irq,
                2);
    bit_clear_fn!(doc = "Disable DMA Setup FIS Interrupt",
                  disable_dma_setup_fis_irq,
                  2);

    bit_get_fn!(doc = "PIO Setup FIS Interrupt", pio_setup_irq, 1);
    bit_set_fn!(doc = "Enable PIO Setup FIS Interrupt",
                enable_pio_setup_irq,
                1);
    bit_clear_fn!(doc = "Disable PIO Setup FIS Interrupt",
                  disable_pio_setup_irq,
                  1);

    bit_get_fn!(doc = "Device to host Register FIS Interrupt",
                d2h_register_fis_interrupt,
                0);
    bit_set_fn!(doc = "Enable Device to host Register FIS Interrupt",
                enable_d2h_register_fis_interrupt,
                0);
    bit_clear_fn!(doc = "Disable Device to host Register FIS Interrupt",
                  disable_d2h_register_fis_interrupt,
                  0);
}

#[repr(packed)]
pub struct CommandAndStatus(Volatile<u32>);

impl CommandAndStatus {
    /// Interface Communication Control
    pub fn icc(&self) -> u8 {
        bits_get(&self.0, 28, 31) as u8
    }

    /// Set Interface Communication Control
    pub fn set_icc(&self) {
        unreachable!();
    }

    bit_get_fn!(doc = "Aggressive Slumber / Partial (ASP)",
                aggressive_slumber,
                27);
    bit_set_fn!(doc = "Set Aggressive Slumber / Partial (ASP)",
                set_aggressive_slumber,
                27);

    bit_get_fn!(doc = "Aggressive Link Power Management Enable (ALPE)",
                aggressive_link_power_mgmt,
                26);
    bit_set_fn!(doc = "Set Aggressive Link Power Management Enable (ALPE)",
                set_aggressive_link_power_mgmt,
                26);

    bit_get_fn!(doc = "Drive LED on ATAPI Enable (DLAE)",
                drive_led_on_atapi,
                25);
    bit_set_fn!(doc = "Set Drive LED on ATAPI Enable (DLAE)",
                set_drive_led_on_atapi,
                25);

    bit_get_fn!(doc = "Device is ATAPI", device_is_atapi, 24);
    bit_set_fn!(doc = "Set Device is ATAPI", set_device_is_atapi, 24);

    bit_get_fn!(doc = "Automatic Partial to Slumber Transitions Enabled (APSTE)",
                automatic_partial_slumber_transitions,
                23);
    bit_set_fn!(doc = "Set Automatic Partial to Slumber Transitions Enabled (APSTE)",
                set_automatic_partial_slumber_transitions,
                23);

    bit_get_fn!(doc = "FIS-based Switching Capable Port",
                fis_switching_port,
                22);
    bit_get_fn!(doc = "External SATA Port", external_sata_port, 21);
    bit_get_fn!(doc = "Cold Presence Detection", cold_presence_detection, 20);
    bit_get_fn!(doc = "Mechanical Presence Switch Attached to Port",
                mechanical_presence_switch_attached,
                19);
    bit_get_fn!(doc = "Hot Plug Capable Port", hot_plug_capable_port, 18);

    bit_get_fn!(doc = "Port Multiplier Attached",
                port_multiplier_attached,
                17);
    bit_set_fn!(doc = "Set Port Multiplier Attached",
                set_port_multiplier_attached,
                17);
    bit_clear_fn!(doc = "Clear Port Multiplier Attached",
                  clear_port_multiplier_attached,
                  17);

    bit_get_fn!(doc = "Cold Presence State", cold_presence, 16);
    bit_get_fn!(doc = "Command List Running (CR)", command_list_running, 15);
    bit_get_fn!(doc = "FIS Receive Running (FR)", fis_receive_running, 14);
    bit_get_fn!(doc = "Mechanical Presence Switch State",
                mechanical_presence_switch_state,
                13);

    /// Current Command Slot
    pub fn ccs(&self) -> u8 {
        bits_get(&self.0, 8, 12) as u8
    }

    bit_get_fn!(doc = "FIS Receive (FRE)", fis_receive, 4);
    bit_set_fn!(doc = "Set FIS Receive", enable_fis_receive, 4);
    bit_clear_fn!(doc = "Clear FIS Receive", disable_fis_receive, 4);

    bit_get_fn!(doc = "Command List Override", command_list_override, 3);
    bit_set_fn!(doc = "Set Command List Override",
                set_command_list_override,
                3);

    bit_get_fn!(doc = "Power On Device", power_on_device, 2);
    bit_set_fn!(doc = "Set Power On Device", enable_power_on_device, 2);
    bit_clear_fn!(doc = "Clear Power On Device", disable_power_on_device, 2);

    bit_get_fn!(doc = "Spin-Up Device", spin_up_device, 1);
    bit_set_fn!(doc = "Set Spin-Up Device", enable_spin_up_device, 1);
    bit_clear_fn!(doc = "Clear Spin-Up Device", disable_spin_up_device, 1);

    bit_get_fn!(doc = "Is Started? (ST)", is_started, 0);
    bit_set_fn!(doc = "Set Start", start, 0);
    bit_clear_fn!(doc = "Stop", stop, 0);
}

impl fmt::Debug for CommandAndStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(write!(f, "CommandAndStatus:\n"));
        try!(write!(f, "\tICC: {}\n", self.icc()));
        try!(write!(f,
                    "\tAggressive Slumber / Partial (ASP): {}\n",
                    self.aggressive_slumber()));
        try!(write!(f,
                    "\tAggressive Link Power Management Enable (ALPE): {}\n",
                    self.aggressive_link_power_mgmt()));
        try!(write!(f,
                    "\tDrive LED on ATAPI Enable (DLAE): {}\n",
                    self.drive_led_on_atapi()));
        try!(write!(f, "\tDevice is ATAPI: {}\n", self.device_is_atapi()));
        try!(write!(f,
                    "\tAutomatic Partial to Slumber Transitions Enabled (APSTE): {}\n",
                    self.automatic_partial_slumber_transitions()));
        try!(write!(f,
                    "\tFIS-based Switching Capable Port: {}\n",
                    self.fis_switching_port()));
        try!(write!(f, "\tExternal SATA Port: {}\n", self.external_sata_port()));
        try!(write!(f,
                    "\tCold Presence Detection: {}\n",
                    self.cold_presence_detection()));
        try!(write!(f,
                    "\tMechanical Presence Switch Attached to Port: {}\n",
                    self.mechanical_presence_switch_attached()));
        try!(write!(f,
                    "\tHot Plug Capable Port: {}\n",
                    self.hot_plug_capable_port()));
        try!(write!(f,
                    "\tPort Multiplier Attached: {}\n",
                    self.port_multiplier_attached()));
        try!(write!(f, "\tCold Presence State: {}\n", self.cold_presence()));
        try!(write!(f,
                    "\tCommand List Running (CR): {}\n",
                    self.command_list_running()));
        try!(write!(f, "\tCurrent Command Slot (CCS): {}\n", self.ccs()));
        try!(write!(f,
                    "\tFIS Receive Running (FR): {}\n",
                    self.fis_receive_running()));
        try!(write!(f,
                    "\tMechanical Presence Switch State: {}\n",
                    self.mechanical_presence_switch_state()));
        try!(write!(f, "\tFIS Receive (FRE): {}\n", self.fis_receive()));
        try!(write!(f,
                    "\tCommand List Override: {}\n",
                    self.command_list_override()));
        try!(write!(f, "\tPower On Device: {}\n", self.power_on_device()));
        try!(write!(f, "\tSpin-Up Device: {}\n", self.spin_up_device()));
        write!(f, "\tIs Started? (ST): {}\n", self.is_started())
    }
}

#[repr(packed)]
pub struct TaskFileData(Volatile<u32>);

impl TaskFileData {
    pub fn error(&self) -> u8 {
        bits_get(&self.0, 8, 15) as u8
    }

    bit_get_fn!(doc = "Indicates the interface is busy.", busy, 7);
    bit_get_fn!(doc = "Data Transfer Requested.", drq, 3);
    bit_get_fn!(doc = "Indicates an error during the transfer.", err, 0);

    pub fn status(&self) -> u8 {
        bits_get(&self.0, 0, 7) as u8
    }
}

impl fmt::Debug for TaskFileData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(write!(f, "TaskFileData:\n"));
        try!(write!(f, "\tErrors: 0b{:b}\n", self.error()));
        write!(f, "\tStatus: 0b{:b}\n", self.status())
    }
}


#[derive(Debug)]
#[repr(packed)]
pub struct PortSignature(Volatile<u32>);

impl PortSignature {
    pub fn lba(&self) -> u32 {
        bits_get(&self.0, 8, 31) << 8
    }

    pub fn sectors(&self) -> u8 {
        bits_get(&self.0, 0, 7) as u8
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct SerialAtaStatus(Volatile<u32>);

impl SerialAtaStatus {
    pub fn power_management(&self) -> u8 {
        // TOOD: enum
        bits_get(&self.0, 8, 11) as u8
    }

    pub fn current_speed(&self) -> u8 {
        // TODO: enum
        bits_get(&self.0, 4, 7) as u8
    }

    pub fn device_detection(&self) -> u8 {
        // TODO: enum
        bits_get(&self.0, 0, 3) as u8
    }
}


#[derive(Debug)]
#[repr(packed)]
pub struct SerialAtaControl(Volatile<u32>);

impl SerialAtaControl {
    pub fn allowed_ipm_transitions(&self) -> u8 {
        // TODO: Should be enum
        bits_get(&self.0, 8, 11) as u8
    }

    pub fn set_allowed_ipm_transitions(&self, transitions: u8) {
        unreachable!();
    }

    pub fn allowed_speed(&self) -> u8 {
        // TODO: Should be enum
        bits_get(&self.0, 4, 7) as u8
    }

    pub fn set_allowed_speed(&self, speed: u8) {
        unreachable!();
    }

    pub fn device_detection_init(&self) -> u8 {
        bits_get(&self.0, 0, 3) as u8
    }

    pub fn set_device_detection_init(&mut self, mode: u8) {
        bits_set(&mut self.0, 0, 3, mode as u32);
    }
}

#[repr(packed)]
pub struct SerialAtaError(Volatile<u32>);

impl SerialAtaError {
    pub fn diagnostics(&self) -> u16 {
        bits_get(&self.0, 16, 31) as u16
    }

    bit_get_fn!(doc = "Exchange (X): Indicates that a change in device presence has been detected",
                exchange,
                26);
    bit_get_fn!(doc = "Unknown FIS Type (F)", unknown_fis_type, 25);
    bit_get_fn!(doc = "Transport state transition error (T)",
                transport_state_transition_error,
                24);
    bit_get_fn!(doc = "Link Sequence Error (S)", link_sequence_error, 23);
    bit_get_fn!(doc = "Handshake Error (H)", handshake_error, 22);
    bit_get_fn!(doc = "CRC Error (C)", crc_error, 21);
    bit_get_fn!(doc = "Disparity Error (D)", disparity_error, 20);
    bit_get_fn!(doc = "10B to 8B Decode Error (B)", decode_error, 19);
    bit_get_fn!(doc = "Comm Wake (W)", comm_wake, 18);
    bit_get_fn!(doc = "Phy Internal Error (I)", phy_internal_error, 17);
    bit_get_fn!(doc = "PhyRdy Change (N)", phyrdy_change, 16);

    pub fn error(&self) -> u16 {
        bits_get(&self.0, 0, 15) as u16
    }

    bit_get_fn!(doc = "Internal Error (E)", internal_error, 11);
    bit_get_fn!(doc = "Protocol Error (P)", protocol_error, 10);
    bit_get_fn!(doc = "Persistent Communication or Data Integrity Error (C)",
                comm_data_error,
                9);
    bit_get_fn!(doc = "Transient Data Integrity Error (T)",
                transient_data_integrity_error,
                8);
    bit_get_fn!(doc = "Recovered Communications Error (M)",
                recovered_communication_error,
                1);
    bit_get_fn!(doc = "Recovered Data Integrity Error (I)",
                recovered_data_integrity_error,
                0);

    pub fn clear(&mut self) {
        self.0.set(u32::max_value());
    }
}

impl fmt::Debug for SerialAtaError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        try!(write!(f, "SerialAtaError:\n"));

        try!(write!(f, "- Diagnostics:\n"));
        try!(write!(f, "\tExchange (X): {}\n", self.exchange()));
        try!(write!(f, "\tUnknown FIS Type (F): {}\n", self.unknown_fis_type()));
        try!(write!(f,
                    "\tTransport state (T): {}\n",
                    self.transport_state_transition_error()));
        try!(write!(f,
                    "\tLink Sequence Error (S): {}\n",
                    self.link_sequence_error()));
        try!(write!(f, "\tHandshake Error (H): {}\n", self.handshake_error()));
        try!(write!(f, "\tCRC Error (C): {}\n", self.crc_error()));
        try!(write!(f, "\tDisparity Error (D): {}\n", self.disparity_error()));
        try!(write!(f, "\t10B to 8B Decode Error (B): {}\n", self.decode_error()));
        try!(write!(f, "\tComm Wake (W): {}\n", self.comm_wake()));
        try!(write!(f,
                    "\tPhy Internal Error (I): {}\n",
                    self.phy_internal_error()));
        try!(write!(f, "\tPhyRdy Change (N): {}\n", self.phyrdy_change()));

        try!(write!(f, "- Error:\n"));
        try!(write!(f, "\tInternal Error (E): {}\n", self.internal_error()));
        try!(write!(f, "\tProtocol Error (P): {}\n", self.protocol_error()));
        try!(write!(f,
                    "\tPersistent Communication or Data Integrity Error (C): {}\n",
                    self.comm_data_error()));
        try!(write!(f,
                    "\tTransient Data Integrity Error (T): {}\n",
                    self.transient_data_integrity_error()));
        try!(write!(f,
                    "\tRecovered Communications Error (M): {}\n",
                    self.recovered_communication_error()));
        write!(f,
               "\tRecovered Data Integrity Error (I): {}\n",
               self.recovered_data_integrity_error())
    }
}



#[derive(Debug)]
#[repr(packed)]
pub struct SerialAtaActive(Volatile<u32>);

impl SerialAtaActive {
    pub fn device_status(&self, slot: u8) -> bool {
        is_bit_set!(self.0, slot)
    }

    pub fn set_device_status(&self, slot: u8) {
        unreachable!();
    }
}


#[derive(Debug)]
#[repr(packed)]
pub struct CommandsIssued(Volatile<u32>);

impl CommandsIssued {
    pub fn commands_issued(&self, slot: u8) -> bool {
        is_bit_set!(self.0, slot)
    }

    pub fn set_commands_issued(&mut self, slot: u8) {
        self.0.set(self.0.get() | 1 << slot);
    }
}

#[derive(Debug)]
#[repr(packed)]
pub struct SerialAtaNotification(Volatile<u32>);

impl SerialAtaNotification {
    /// PM Notify (PMN)
    ///
    /// This field indicates whether a particular device with the corresponding
    /// PM Port number issued a Set Device Bits FIS to the host with the Notification bit set.
    pub fn pm_notify(&self, port: u8) -> bool {
        is_bit_set!(self.0, port)
    }

    pub fn clear_pm_notify(&mut self, port: u8) {
        self.0.set(self.0.get() | 1 << port);
    }
}


#[derive(Debug)]
#[repr(packed)]
pub struct FisSwitchingControl(Volatile<u32>);

impl FisSwitchingControl {
    /// Device With Error
    pub fn device_with_error(&self) -> u8 {
        bits_get(&self.0, 16, 19) as u8
    }

    /// Active Device Optimization
    pub fn active_device_optimization(&self) -> u8 {
        bits_get(&self.0, 12, 15) as u8
    }

    /// Device To Issue
    pub fn device_to_issue(&self) -> u8 {
        bits_get(&self.0, 8, 11) as u8
    }

    pub fn set_device_to_issue(&self, port: u8) {
        unreachable!()
    }

    bit_get_fn!(doc = "Single Device Error", single_device_error, 2);
    bit_set_fn!(doc = "Device Error Clear", device_clear_error, 1);

    bit_get_fn!(doc = "Enabled", is_enabled, 0);
    bit_set_fn!(doc = "Enable FIS-based switching", enable, 0);
    bit_clear_fn!(doc = "Disable FIS-based switching", disable, 0);
}

#[derive(Debug)]
#[repr(packed)]
pub struct DeviceSleep(Volatile<u32>);

impl DeviceSleep {
    /// DITO Multiplier
    pub fn dito_multiplier(&self) -> u8 {
        bits_get(&self.0, 25, 28) as u8
    }

    /// Device Sleep Idle Timeout (DITO)
    pub fn dito(&self) -> u8 {
        bits_get(&self.0, 15, 24) as u8
    }

    /// Minimum Device Sleep Assertion Time (MDAT)
    pub fn mdat(&self) -> u8 {
        bits_get(&self.0, 10, 14) as u8
    }

    /// Device Sleep Exit Timeout (DETO)
    pub fn deto(&self) -> u8 {
        bits_get(&self.0, 2, 9) as u8
    }

    bit_get_fn!(doc = "Device Sleep Present (DSP)", has_device_sleep, 1);

    bit_get_fn!(doc = "Aggressive Device Sleep (ADSE)",
                aggressive_device_sleep,
                0);
    bit_set_fn!(doc = "Aggressive Device Sleep Enable (ADSE)",
                enable_aggressive_device_sleep,
                0);
    bit_clear_fn!(doc = "Aggressive Device Sleep Disable (ADSE)",
                  disable_aggressive_device_sleep,
                  0);
}

#[derive(Debug)]
#[repr(packed)]
pub struct CommandHeader {
    flags: Volatile<u16>,
    /// Physical Region Descriptor Table Length (PRDTL)
    pub prdtl: Volatile<u16>,
    /// Physical Region Descriptor Byte Count (PRDBC)
    pub prdbc: Volatile<u32>,
    /// Command Table Descriptor Base Address (CTBA)
    pub ctba: Volatile<u64>,
    reserved: [u32; 4],
}

impl CommandHeader {
    /// Create an empty Command Header
    pub fn new() -> CommandHeader {
        CommandHeader {
            flags: Volatile::with_value(0),
            prdtl: Volatile::with_value(0),
            prdbc: Volatile::with_value(0),
            ctba: Volatile::with_value(0),
            reserved: [0, 0, 0, 0],
        }
    }

    /// Command FIS Length (CFL)
    pub fn set_cfl(&mut self, length: u8) {
        assert!(length != 0 && length != 1 && length <= 16);
        bits_set_16(&mut self.flags, 0, 4, length as u16)
    }

    /// Set Port Multiplier Port (PMP)
    pub fn set_pmp(&mut self, pmp: u8) {
        bits_set_16(&mut self.flags, 12, 15, pmp as u16)
    }

    /// ATAPI (A)
    pub fn set_atapi(&mut self) {
        self.flags.set(self.flags.get() | 1 << 5);
    }

    /// Write (W)
    pub fn set_write(&mut self) {
        self.flags.set(self.flags.get() | 1 << 6);
    }

    /// Prefetchable (P)
    pub fn set_prefetchable(&mut self) {
        self.flags.set(self.flags.get() | 1 << 7);
    }

    /// Reset (R)
    pub fn set_reset(&mut self) {
        self.flags.set(self.flags.get() | 1 << 8);
    }

    /// BIST (B)
    pub fn set_bist(&mut self) {
        self.flags.set(self.flags.get() | 1 << 9);
    }

    /// Clear Busy upon R_OK (C)
    pub fn set_clear_busy(&mut self) {
        self.flags.set(self.flags.get() | 1 << 10);
    }
}

#[repr(packed)]
pub struct RegionDescriptor {
    /// Data base address
    dba: u64,
    /// Reserved
    reserved: u32,
    /// Byte count and Interrupt bit (31)
    dbc: u32,
}

impl RegionDescriptor {
    pub fn new(dba: u64, dbc: u32, interrupt: bool) -> RegionDescriptor {
        assert!((dba & 0x1) == 0);
        assert!((dbc & 0x1) > 0 && dbc <= 1024 * 1024 * 4);

        let dbc_val = match interrupt {
            true => (1 << 31) | dbc,
            false => dbc,
        };

        RegionDescriptor {
            dba: dba,
            reserved: 0,
            dbc: dbc_val,
        }
    }
}

#[repr(packed)]
pub struct CommandTable {
    /// Command FIS
    pub cfis: [u8; 64],
    /// ATAPI command
    pub acmd: [u8; 16],
    /// Reserved
    reserved: [u8; 48],
    /// Physical Descriptior Region Table entries
    /// Can be up to 65536 entries, we limit this to 248 for now
    /// (which limits the size of CommandTable to a single page).
    pub prdt: [RegionDescriptor; 248],
}
