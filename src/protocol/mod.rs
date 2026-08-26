pub mod registers;

use derive_more::Display;
pub use registers::Register;
mod instruction;
pub use instruction::{
    BROADCAST_ID, BulkComm, BulkCommEntries, BulkCommEntry, MAX_READ_REGISTERS, Packet, PacketKind, RegisterWrite,
    RegisterWrites, STATUS_FLAG, register_writes,
};
mod motor_error;
pub use motor_error::ErrorFlags;
pub use motor_error::{ERROR_FLAGS, WARNING_FLAGS};
mod response;

pub use response::Response;
mod bulk_write_data;
pub use bulk_write_data::BulkWriteData;

pub(crate) const PACKET_ID: usize = 2;
pub(crate) const PACKET_LEN: usize = 3;
pub(crate) const PACKET_ERROR: usize = 4;

/// Bytes per register value on the wire (4 little-endian bytes).
pub(crate) const REGISTER_BYTES: usize = 4;

/// The largest parameter block a frame can carry, in bytes.
///
/// `LEN` is one byte and counts the instruction/error byte and the checksum.
pub const MAX_PARAMETER_COUNT: usize = u8::MAX as usize - 2;

/// The largest frame the protocol can describe, in bytes.
///
/// `FF FF | id | len | inst | parameters | checksum` with a full parameter
/// block. Smaller buffers cannot carry every legal frame: reading all 31 config
/// registers replies with 130 bytes, and writing them sends 161.
pub const MAX_PACKET_SIZE: usize = PACKET_ERROR + 1 + MAX_PARAMETER_COUNT + 1;

/// The instructions supported by the BEAR protocol.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[non_exhaustive]
pub enum Instruction {
    /// Ping a motor to check if it is present on the bus.
    Ping = 0x01,
    /// Read a status register.
    ReadStat = 0x02,
    /// Write a status register.
    WriteStat = 0x03,
    /// Read a config register.
    ReadCfg = 0x04,
    /// Write a config register.
    WriteCfg = 0x05,
    /// Save config registers to flash so they persist across reboots.
    SaveCfg = 0x06,
    /// Set the absolute position of a motor with a backup battery.
    SetAbsPos = 0x08,
    /// Bulk read/write multiple motors in a single packet. See [`crate::Bus::bulk_read_write`].
    BulkComm = 0x12,
}

/// Registers used to set motor configuration
#[derive(Debug, Clone, Copy, strum::EnumIter, PartialEq, Eq, PartialOrd, Ord, Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[non_exhaustive]
pub enum ConfigRegister {
    /// ID register, u32, read and write
    Id = 0x00,
    /// MODE register, u32, read and write
    Mode = 0x01,
    /// BAUDRATE register, u32, read and write
    BaudRate = 0x02,
    /// HOMING OFFSET register, f32, read and write
    HomingOffset = 0x03,
    /// P GAIN ID register, f32, read and write
    PGainId = 0x04,
    /// I GAIN ID register, f32, read and write
    IGainId = 0x05,
    /// D GAIN ID register, f32, read and write
    DGainId = 0x06,
    /// // register, f32, read and write
    PGainIq = 0x07,
    /// I GAIN IQ register, f32, read and write
    IGainIq = 0x08,
    /// D GAIN IQ register, f32, read and write
    DGainIq = 0x09,
    /// P GAIN VEL register, f32, read and write
    PGainVel = 0x0A,
    /// I GAIN VEL register, f32, read and write
    IGainVel = 0x0B,
    /// D GAIN VEL register, f32, read and write
    DGainVel = 0x0C,
    /// P GAIN POS register, f32, read and write
    PGainPos = 0x0D,
    /// I GAIN POS register, f32, read and write
    IGainPos = 0x0E,
    /// D GAIN POS register, f32, read and write
    DGainPos = 0x0F,
    /// P GAIN FORCE register, f32, read and write
    PGainForce = 0x10,
    /// I GAIN FORCE register, f32, read and write
    IGainForce = 0x11,
    /// D GAIN FORCE register, f32, read and write
    DGainForce = 0x12,
    /// LIMIT ACCELERATION MAX register, f32, read and write
    LimitAccMax = 0x13,
    /// LIMIT I MAX register, f32, read and write
    LimitIMax = 0x14,
    /// LIMIT VEL MAX register, f32, read and write
    LimitVelMax = 0x15,
    /// LIMIT POS MIN register, f32, read and write
    LimitPosMin = 0x16,
    /// LIMIT POS MAX register, f32, read and write
    LimitPosMax = 0x17,
    /// MIN VOLTAGE register, f32, read and write
    MinVoltage = 0x18,
    /// MAX VOLTAGE register, f32, read and write
    MaxVoltage = 0x19,
    // LOW_VOLTAGE_WARNING = 0x1A,
    /// WATCHDOG TIMEOUT register, f32, read and write
    WatchdogTimeout = 0x1A,
    /// TEMP LIMIT LOW register, f32, read and write
    TempLimitLow = 0x1B, // Motor will start to limit power
    /// TEMP LIMIT HIGH register, f32, read and write
    TempLimitHigh = 0x1C, // Motor will shutdown

    /// RETURN TIME DELAY register, u32, read and write
    ReturnTimeDelay = 0x1E,
}

impl ConfigRegister {
    pub(crate) const READ_INST: u8 = Instruction::ReadCfg as u8;
    pub(crate) const WRITE_INST: u8 = Instruction::WriteCfg as u8;

    /// Size of the motor's config table, in registers.
    ///
    /// The number of addressable indices, not the number of variants above: the
    /// firmware bounds-checks a read against its whole table and answers any
    /// index below this, including `0x1D`, which is reserved and has no variant
    /// here. A device emulating a motor has to accept the same range.
    pub const COUNT: usize = 31;
}

/// A config table index, as it appears on the wire.
///
/// [`Bus::read_config`] and [`Bus::write_config`] take `impl Into<ConfigAddr>`:
/// a [`ConfigRegister`] covers the standard table, and a `u8` reaches anything
/// else. A register index is a full byte on the wire, and a device may
/// implement vendor registers above the standard table -- [`ConfigRegister`] is
/// `#[non_exhaustive]` precisely because it does not claim to be the whole
/// address space.
///
/// The device decides what is valid: an index it does not implement gets no
/// reply, which surfaces as a timeout.
///
/// The wrapper is what keeps a [`StatusRegister`] out of a config call:
/// `bus.read_config(id, 0x1D)` reaches a reserved index, while
/// `bus.read_config(id, StatusRegister::PresentPos)` does not compile.
///
/// [`Bus::read_config`]: crate::Bus::read_config
/// [`Bus::write_config`]: crate::Bus::write_config
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[display("{_0:#04x}")]
pub struct ConfigAddr(pub u8);

impl From<ConfigRegister> for ConfigAddr {
    fn from(register: ConfigRegister) -> Self {
        Self(register as u8)
    }
}

impl From<u8> for ConfigAddr {
    fn from(index: u8) -> Self {
        Self(index)
    }
}

/// Status Registers
#[derive(Debug, Clone, Copy, strum::EnumIter, PartialEq, Eq, PartialOrd, Ord, Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[non_exhaustive]
pub enum StatusRegister {
    /// TORQUE ENABLE register, f32, read and write
    TorqueEnable = 0x00, // Enable output
    /// HOMING COMPLETE register, f32, read and write
    HomingComplete = 0x01,
    /// GOAL I D register, f32, read and write
    GoalId = 0x02,
    /// GOAL I Q register, f32, read and write
    GoalIq = 0x03,
    /// GOAL VEL register, f32, read and write
    GoalVel = 0x04,
    /// GOAL POS register, f32, read and write
    GoalPos = 0x05,
    /// PRESENT I D register, f32, read only
    PresentId = 0x06,
    /// PRESENT I Q register, f32, read only
    PresentIq = 0x07,
    /// PRESENT VEL register, f32, read only
    PresentVel = 0x08,
    /// PRESENT POS register, f32, read only
    PresentPos = 0x09,
    /// INPUT VOLTAGE register, f32, read only
    InputVoltage = 0x0A,
    /// WINDING TEMP register, f32, read only
    WindingTemp = 0x0B,
    /// POWERSTAGE TEMP register, f32, read only
    PowerstageTemp = 0x0C,
    /// IC TEMP register, f32, read only
    IcTemp = 0x0D,
}

impl StatusRegister {
    pub(crate) const READ_INST: u8 = Instruction::ReadStat as u8;
    pub(crate) const WRITE_INST: u8 = Instruction::WriteStat as u8;

    /// Size of the motor's status table, in registers.
    ///
    /// The number of addressable indices, not the number of variants above.
    /// `0x0E` and `0x0F` have no variant here because they are not useful to a
    /// client, but the firmware still answers them, so a device emulating a
    /// motor has to accept the same range.
    pub const COUNT: usize = 16;
}

/// A status table index, as it appears on the wire.
///
/// The status-table counterpart of [`ConfigAddr`]: [`Bus::read_status`] and
/// [`Bus::write_status`] take `impl Into<StatusAddr>`, so a [`StatusRegister`]
/// covers the standard table and a `u8` reaches any other index the device
/// implements.
///
/// [`Bus::read_status`]: crate::Bus::read_status
/// [`Bus::write_status`]: crate::Bus::write_status
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[display("{_0:#04x}")]
pub struct StatusAddr(pub u8);

impl From<StatusRegister> for StatusAddr {
    fn from(register: StatusRegister) -> Self {
        Self(register as u8)
    }
}

impl From<u8> for StatusAddr {
    fn from(index: u8) -> Self {
        Self(index)
    }
}
