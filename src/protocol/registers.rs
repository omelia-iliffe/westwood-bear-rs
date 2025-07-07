//! The Bear protocol uses register to read and write data to the motors
//! Config regsiters can only be written to when the motors torque is disable. Config registers persist on reboot and can be updated (using [`crate::Bus::save_config`]).
//!
//! Status registers are not persistant and writable registers can be updated when torque is enabled.
//!
//! This module also contains [`Register`] and [`WritableRegister`] traits used for working with registers generically.
use crate::error::{BufferTooSmallError, InvalidMessage};

/// Implemented by each register, this trait is used with [`crate::Bus::read`].
pub trait Register {
    /// The register name, useful for debugging but currently unused.
    const NAME: &'static str;
    /// The inner type that can be read or written to the register
    type Inner;
    /// The instruction used for reading this register
    const READ_INST: u8;
    /// The address that register data is read from or written to
    const ADDRESS: u8;

    /// Decode the value from the given buffer.
    fn decode(buffer: &[u8]) -> Result<Self::Inner, InvalidMessage>;
}

/// Implemented by all writeable registers, this trait is used with [`crate::Bus::write`].
pub trait WritableRegister: Register {
    /// The instruction used for writing to this register
    const WRITE_INST: u8;
    /// The number of bytes of the register
    const ENCODED_SIZE: u8;

    /// Encode the value into the given buffer.
    fn encode(value: Self::Inner, buffer: &mut [u8]) -> Result<(), BufferTooSmallError>;
    /// Encode the value into bytes
    fn encode_bytes(data: Self::Inner) -> [u8; 4];
}

const fn to_u8(input: usize) -> u8 {
    assert!(input <= u8::MAX as usize);
    input as u8
}
macro_rules! register {
    (@REGISTER $register:ident : $r_type:ty, $addr:expr, $inner:ty) => {
        #[derive(Debug, Clone, PartialEq)]
        #[doc = concat!("[`",stringify!($register),"`] register at address `",stringify!($addr), "`")]
        #[doc = concat!("[`",stringify!($register),"`] is of type [`", stringify!($inner), "`]")]
        pub struct $register;

        impl Register for $register {
            const NAME: &'static str = stringify!($register);
            type Inner = $inner;
            const ADDRESS: u8 = $addr as u8;
            const READ_INST: u8 = <$r_type>::READ_INST;

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
                #[doc = "read the `" $register "` from a specific motor."]
                pub fn [<read_ $register:snake>](&mut self, id: u8) -> Result<Response<$inner>, TransferError<SerialPort::Error>> {
                    self.read::<$register>(id)
                }
            }
        }
    };

    (@WRITABLE $register:ident : $r_type:ty, $addr:expr, $inner:ty) => {
        register!(@REGISTER $register: $r_type, $addr, $inner);
        impl WritableRegister for $register {
            const ENCODED_SIZE: u8 = to_u8(core::mem::size_of::<Self::Inner>());

            const WRITE_INST: u8 = <$r_type>::WRITE_INST;

            fn encode(data: Self::Inner, buffer: &mut [u8]) -> Result<(), BufferTooSmallError> {
                const N: usize = core::mem::size_of::<$inner>();
                crate::error::BufferTooSmallError::check(N, buffer.len())?;
                buffer[..N].copy_from_slice(&data.to_le_bytes());
                Ok(())
            }

            fn encode_bytes(data: Self::Inner) -> [u8; core::mem::size_of::<Self::Inner>()] {
                data.to_le_bytes()
            }
        }
        impl<SerialPort, Buffer> crate::Bus<SerialPort, Buffer>
            where SerialPort: crate::SerialPort,
                    Buffer: AsMut<[u8]> + AsRef<[u8]> {
            paste::item!{
                #[doc = "write a `" $inner "` to the `" $register "` of a specific motor."]
                pub fn [<write_ $register:snake>](&mut self, id: u8, data: $inner) -> Result<(), WriteError<SerialPort::Error>> {
                    self.write::<$register>(id, data)
                }
            }
        }
    };
    (ConfigRegister::$register:ident, $inner:ty, RW) => {
        register!(@WRITABLE  $register: ConfigRegister, ConfigRegister::$register, $inner);
    };
    (StatusRegister::$register:ident, $inner:ty, RW) => {
        register!(@WRITABLE $register: StatusRegister, StatusRegister::$register, $inner);
    };
    (ConfigRegister::$register:ident, $inner:ty, RO) => {
        register!(@REGISTER $register: ConfigRegister, ConfigRegister::$register, $inner);
    };
    (StatusRegister::$register:ident, $inner:ty, RO) => {
        register!(@REGISTER $register: StatusRegister, StatusRegister::$register, $inner);
    };

}

/// Structs representing each config register.
/// Use with [`crate::Bus::read`] and [`crate::Bus::write`]
pub mod config {
    use super::*;
    use crate::error::{BufferTooSmallError, InvalidMessage, TransferError, WriteError};
    use crate::protocol::ConfigRegister;
    use crate::Response;
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

/// Structs representing each status register.
/// Use with [`crate::Bus::read`] and [`crate::Bus::write`]
pub mod status {
    use super::*;
    use crate::error::{BufferTooSmallError, InvalidMessage, TransferError, WriteError};
    use crate::protocol::StatusRegister;
    use crate::Response;
    register!(StatusRegister::TorqueEnable, u32, RW);
    register!(StatusRegister::HomingComplete, f32, RW);
    register!(StatusRegister::GoalId, f32, RW);
    register!(StatusRegister::GoalIq, f32, RW);
    register!(StatusRegister::GoalVel, f32, RW);
    register!(StatusRegister::GoalPos, f32, RW);
    register!(StatusRegister::PresentId, f32, RO);
    register!(StatusRegister::PresentIq, f32, RO);
    register!(StatusRegister::PresentVel, f32, RO);
    register!(StatusRegister::PresentPos, f32, RO);
    register!(StatusRegister::InputVoltage, f32, RO);
    register!(StatusRegister::WindingTemp, f32, RO);
    register!(StatusRegister::PowerstageTemp, f32, RO);
    register!(StatusRegister::IcTemp, f32, RO);
    register!(StatusRegister::ErrorStatus, f32, RO);
    register!(StatusRegister::WarningStatus, u32, RO);
}
