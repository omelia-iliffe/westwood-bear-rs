pub mod registers;

use derive_more::Display;
pub use registers::Register;
mod motor_error;
pub use motor_error::ErrorFlags;
pub use motor_error::{ERROR_FLAGS, WARNING_FLAGS};
mod response;

pub use response::Response;

pub(crate) const PACKET_ID: usize = 2;
pub(crate) const PACKET_LEN: usize = 3;
pub(crate) const PACKET_ERROR: usize = 4;

/// The Instructions supported by the Bear Protocol.
/// Currently BulkComm is unsupported by this crate.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Instruction {
    Ping = 0x01,
    ReadStat = 0x02,
    WriteStat = 0x03,
    ReadCfg = 0x04,
    WriteCfg = 0x05,
    SaveCfg = 0x06,
    BulkComm = 0x12,
}

/// Registers used to set motor configuration
#[derive(Debug, Clone, Copy, strum::EnumIter, PartialEq, Eq, PartialOrd, Ord, Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
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
}

impl ConfigRegister {
    pub(crate) const READ_INST: u8 = Instruction::ReadCfg as u8;
    pub(crate) const WRITE_INST: u8 = Instruction::WriteCfg as u8;
}

/// Status Registers
#[derive(Debug, Clone, Copy, strum::EnumIter, PartialEq, Eq, PartialOrd, Ord, Display)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
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
    /// ERROR STATUS register, f32, read only
    ErrorStatus = 0x0E,
    /// WARNING STATUS register, f32, read only
    WarningStatus = 0x0F,
}

impl StatusRegister {
    pub(crate) const READ_INST: u8 = Instruction::ReadStat as u8;
    pub(crate) const WRITE_INST: u8 = Instruction::WriteStat as u8;
}
