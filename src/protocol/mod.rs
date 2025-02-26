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
#[derive(Debug)]
#[repr(u8)]
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
#[repr(u8)]
pub enum ConfigRegister {
    Id = 0x00,
    Mode = 0x01,
    BaudRate = 0x02,
    HomingOffset = 0x03,
    // Gains for Id current loop
    PGainId = 0x04,
    IGainId = 0x05,
    DGainId = 0x06,
    // Gains for Iq current loop
    PGainIq = 0x07,
    IGainIq = 0x08,
    DGainIq = 0x09,
    // Gains for velocity loop
    PGainVel = 0x0A,
    IGainVel = 0x0B,
    DGainVel = 0x0C,
    // Gains for position loop
    PGainPos = 0x0D,
    IGainPos = 0x0E,
    DGainPos = 0x0F,
    // Gains for direct force loop
    PGainForce = 0x10,
    IGainForce = 0x11,
    DGainForce = 0x12,
    // Limits
    LimitAccMax = 0x13,
    LimitIMax = 0x14,
    LimitVelMax = 0x15,
    LimitPosMin = 0x16,
    LimitPosMax = 0x17,
    MinVoltage = 0x18,
    MaxVoltage = 0x19,
    // LOW_VOLTAGE_WARNING = 0x1A,
    WatchdogTimeout = 0x1A,
    TempLimitLow = 0x1B,  // Motor will start to limit power
    TempLimitHigh = 0x1C, // Motor will shutdown
}

impl ConfigRegister {
    pub(crate) const READ_INST: u8 = Instruction::ReadCfg as u8;
    pub(crate) const WRITE_INST: u8 = Instruction::WriteCfg as u8;
}

/// Status Registers
#[derive(Debug, Clone, Copy, strum::EnumIter, PartialEq, Eq, PartialOrd, Ord, Display)]
#[repr(u8)]
pub enum StatusRegister {
    TorqueEnable = 0x00, // Enable output
    HomingComplete = 0x01,
    GoalId = 0x02,
    GoalIq = 0x03,
    GoalVel = 0x04,
    GoalPos = 0x05,
    PresentId = 0x06,
    PresentIq = 0x07,
    PresentVel = 0x08,
    PresentPos = 0x09,
    InputVoltage = 0x0A,
    WindingTemp = 0x0B,
    PowerstageTemp = 0x0C,
    IcTemp = 0x0D,
    ErrorStatus = 0x0E,
    WarningStatus = 0x0F,
}

impl StatusRegister {
    pub(crate) const READ_INST: u8 = Instruction::ReadStat as u8;
    pub(crate) const WRITE_INST: u8 = Instruction::WriteStat as u8;
}
