use crate::error::{BufferTooSmallError, InvalidMessage, TransferError, WriteError};
use crate::{Instruction, Response};

pub trait Register {
    /// The inner type that the can be read or written to the register
    type Inner;
    const READ_INST: Instruction;
    /// The address that register data is read from or written to
    const ADDR: u8;

    /// Decode the value from the given buffer.
    fn decode(buffer: &[u8]) -> Result<Self::Inner, InvalidMessage>;
}

pub trait WritableRegister: Register {
    /// The number of bytes of the register
    const ENCODED_SIZE: u8;

    const WRITE_INST: Instruction;
    /// Encode the value into the given buffer.
    fn encode(value: Self::Inner, buffer: &mut [u8]) -> Result<(), BufferTooSmallError>;
}

const fn to_u8(input: usize) -> u8 {
    assert!(input <= u8::MAX as usize);
    input as u8
}
macro_rules! register {
    (@REGISTER $register:ident : $r_inst:expr, $addr:expr, $inner:ty) => {
        #[derive(Debug, Clone, PartialEq)]
        #[doc = concat!("[`",stringify!($register),"`] register at address `",stringify!($addr), "`")]
        #[doc = concat!("[`",stringify!($register),"`] is of type [`", stringify!($inner), "`]")]
        pub struct $register;

        impl Register for $register {
            type Inner = $inner;
            const ADDR: u8 = $addr as u8;

            const READ_INST: Instruction = $r_inst;

            fn decode(buffer: &[u8]) -> Result<Self::Inner, InvalidMessage> {
                const N: usize = core::mem::size_of::<$inner>();
                crate::error::InvalidParameterCount::check(buffer.len(), N)?;
                let value = Self::Inner::from_le_bytes(buffer[0..N].try_into().unwrap());
                Ok(value)
            }
        }
        impl<SerialPort, Buffer> crate::Bus<SerialPort, Buffer>
            where SerialPort: crate::SerialPort,
                    Buffer: AsMut<[u8]> + AsRef<[u8]> {
            paste::item!{
                pub fn [<read_ $register:snake>](&mut self, id: u8) -> Result<Response<$inner>, TransferError<SerialPort::Error>> {
                    self.read::<$register>(id)
                }
            }
        }
    };

    (@WRITABLE $register:ident : $r_inst:expr, $w_inst:expr, $addr:expr, $inner:ty) => {
        register!(@REGISTER $register: $r_inst, $addr, $inner);
        impl WritableRegister for $register {
            const ENCODED_SIZE: u8 = to_u8(core::mem::size_of::<Self::Inner>());

            const WRITE_INST: Instruction = $w_inst;

            fn encode(data: Self::Inner, buffer: &mut [u8]) -> Result<(), BufferTooSmallError> {
                const N: usize = core::mem::size_of::<$inner>();
                crate::error::BufferTooSmallError::check(N, buffer.len())?;
                buffer[..N].copy_from_slice(&data.to_le_bytes());
                Ok(())
            }
        }
        impl<SerialPort, Buffer> crate::Bus<SerialPort, Buffer>
            where SerialPort: crate::SerialPort,
                    Buffer: AsMut<[u8]> + AsRef<[u8]> {
            paste::item!{
                pub fn [<write_ $register:snake>](&mut self, id: u8, data: $inner) -> Result<(), WriteError<SerialPort::Error>> {
                    self.write::<$register>(id, data)
                }
            }
        }
    };
    (ConfigRegister::$register:ident, $inner:ty, RW) => {
        register!(@WRITABLE $register: Instruction::ReadCfg, Instruction::WriteCfg, crate::protocol::ConfigRegister::$register, $inner);
    };
    (StatusRegister::$register:ident, $inner:ty, RW) => {
        register!(@WRITABLE $register: Instruction::ReadStat, Instruction::WriteStat, crate::protocol::StatusRegister::$register, $inner);
    };
    (ConfigRegister::$register:ident, $inner:ty, RO) => {
        register!(@REGISTER $register: Instruction::ReadStat, crate::protocol::ConfigRegister::$register, $inner);
    };
    (StatusRegister::$register:ident, $inner:ty, R0) => {
        register!(@REGISTER $register: Instruction::ReadCfg, crate::protocol::StatusRegister::$register, $inner);
    };

}

pub mod config {
    use super::*;
    register!(ConfigRegister::Id, u32, RW);
    register!(ConfigRegister::Mode, u32, RW);
    register!(ConfigRegister::BaudRate, u32, RW);
    register!(ConfigRegister::HomingOffset, f32, RW);
    register!(ConfigRegister::PGainId, f32, RW);
    register!(ConfigRegister::IGainId, f32, RW);
    register!(ConfigRegister::DGainId, f32, RW);
    register!(ConfigRegister::PGainIq, f32, RW);
    register!(ConfigRegister::IGainIq, f32, RW);
    register!(ConfigRegister::DGainIq, f32, RW);
    register!(ConfigRegister::PGainVel, f32, RW);
    register!(ConfigRegister::IGainVel, f32, RW);
    register!(ConfigRegister::DGainVel, f32, RW);
    register!(ConfigRegister::PGainPos, f32, RW);
    register!(ConfigRegister::IGainPos, f32, RW);
    register!(ConfigRegister::DGainPos, f32, RW);
    register!(ConfigRegister::PGainForce, f32, RW);
    register!(ConfigRegister::IGainForce, f32, RW);
    register!(ConfigRegister::DGainForce, f32, RW);
    register!(ConfigRegister::LimitAccMax, f32, RW);
    register!(ConfigRegister::LimitIMax, f32, RW);
    register!(ConfigRegister::LimitVelMax, f32, RW);
    register!(ConfigRegister::LimitPosMin, f32, RW);
    register!(ConfigRegister::LimitPosMax, f32, RW);
    register!(ConfigRegister::MinVoltage, f32, RW);
    register!(ConfigRegister::MaxVoltage, f32, RW);
    register!(ConfigRegister::WatchdogTimeout, u32, RW);
    register!(ConfigRegister::TempLimitLow, f32, RW);
    register!(ConfigRegister::TempLimitHigh, f32, RW);
}

pub mod status {
    use super::*;
    register!(StatusRegister::TorqueEnable, u32, RW);
    register!(StatusRegister::HomingComplete, f32, RW);
    register!(StatusRegister::GoalId, f32, RW);
    register!(StatusRegister::GoalIq, f32, RW);
    register!(StatusRegister::GoalVel, f32, RW);
    register!(StatusRegister::GoalPos, f32, RW);
    register!(StatusRegister::PresentId, f32, RW);
    register!(StatusRegister::PresentIq, f32, RW);
    register!(StatusRegister::PresentVel, f32, RW);
    register!(StatusRegister::PresentPos, f32, RW);
    register!(StatusRegister::InputVoltage, f32, RW);
    register!(StatusRegister::WindingTemp, f32, RW);
    register!(StatusRegister::PowerstageTemp, f32, RW);
    register!(StatusRegister::IcTemp, f32, RW);
    register!(StatusRegister::ErrorStatus, f32, RW);
    register!(StatusRegister::WarningStatus, f32, RW);
}
