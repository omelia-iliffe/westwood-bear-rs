#![allow(non_camel_case_types)]
use crate::error::{BufferTooSmallError, InvalidMessage};

#[derive(Debug)]
pub enum RegisterType {
    Config,
    Status,
}
pub trait Register {
    /// The inner type that the can be read or written to the register
    type Inner;
    /// The type of Register, either [`RegisterType::Config`] or [`RegisterType::Status`]
    const REG_TYPE: RegisterType;
    /// The address that register data is read from or written to
    const ADDR: u8;
    /// The number of bytes of the register
    const ENCODED_SIZE: u8;

    /// Encode the value into the given buffer.
    fn encode(value: Self::Inner, buffer: &mut [u8]) -> Result<(), BufferTooSmallError>;

    /// Decode the value from the given buffer.
    fn decode(buffer: &[u8]) -> Result<Self::Inner, InvalidMessage>;
}

const fn to_u8(input: usize) -> u8 {
    assert!(input <= u8::MAX as usize);
    input as u8
}
macro_rules! register {
    (@REGISTER $reg:ident : $r_type:expr, $addr:expr, $inner:ty) => {
        #[derive(Debug, Clone, PartialEq)]
        #[doc = concat!("[`",stringify!($reg),"`] register at address `",stringify!($addr), "`")]
        #[doc = concat!("[`",stringify!($reg),"`] is of type [`", stringify!($inner), "`]")]
        pub struct $reg;

        impl Register for $reg {
            type Inner = $inner;
            const REG_TYPE: RegisterType = $r_type;
            const ADDR: u8 = $addr as u8;
            const ENCODED_SIZE: u8 = to_u8(core::mem::size_of::<Self::Inner>());

            fn encode(data: Self::Inner, buffer: &mut [u8]) -> Result<(), BufferTooSmallError> {
                const N: usize = core::mem::size_of::<$inner>();
                crate::error::BufferTooSmallError::check(N, buffer.len())?;
                buffer[..N].copy_from_slice(&data.to_le_bytes());
                Ok(())
            }

            fn decode(buffer: &[u8]) -> Result<Self::Inner, InvalidMessage> {
                const N: usize = core::mem::size_of::<$inner>();
                crate::error::InvalidParameterCount::check(buffer.len(), N)?;
                let value = Self::Inner::from_le_bytes(buffer[0..N].try_into().unwrap());
                Ok(value)
            }
        }
    };
    (ConfigRegister::$register:ident, $inner:ty) => {
        register!(@REGISTER $register: RegisterType::Config, crate::protocol::ConfigRegister::$register, $inner);
    };
    (StatusRegister::$register:ident, $inner:ty) => {
        register!(@REGISTER $register: RegisterType::Status, crate::protocol::StatusRegister::$register, $inner);
    };
}

register!(ConfigRegister::ID, u32);
register!(ConfigRegister::Mode, u32);
register!(ConfigRegister::BaudRate, u32);
register!(ConfigRegister::HomingOffset, f32);
register!(ConfigRegister::PGainId, f32);
register!(ConfigRegister::IGainId, f32);
register!(ConfigRegister::DGainId, f32);
register!(ConfigRegister::PGainIq, f32);
register!(ConfigRegister::IGainIq, f32);
register!(ConfigRegister::DGainIq, f32);
register!(ConfigRegister::PGainVel, f32);
register!(ConfigRegister::IGainVel, f32);
register!(ConfigRegister::DGainVel, f32);
register!(ConfigRegister::PGainPos, f32);
register!(ConfigRegister::IGainPos, f32);
register!(ConfigRegister::DGainPos, f32);
register!(ConfigRegister::PGainForce, f32);
register!(ConfigRegister::IGainForce, f32);
register!(ConfigRegister::DGainForce, f32);
register!(ConfigRegister::LimitAccMax, f32);
register!(ConfigRegister::LimitIMax, f32);
register!(ConfigRegister::LimitVelMax, f32);
register!(ConfigRegister::LimitPosMin, f32);
register!(ConfigRegister::LimitPosMax, f32);
register!(ConfigRegister::MinVoltage, f32);
register!(ConfigRegister::MaxVoltage, f32);
register!(ConfigRegister::WatchdogTimeout, u32);
register!(ConfigRegister::TempLimitLow, f32);
register!(ConfigRegister::TempLimitHigh, f32);

register!(StatusRegister::TorqueEnable, u32);
register!(StatusRegister::HomingComplete, f32);
register!(StatusRegister::GoalId, f32);
register!(StatusRegister::GoalIq, f32);
register!(StatusRegister::GoalVel, f32);
register!(StatusRegister::GoalPos, f32);
register!(StatusRegister::PresentId, f32);
register!(StatusRegister::PresentIq, f32);
register!(StatusRegister::PresentVel, f32);
register!(StatusRegister::PresentPos, f32);
register!(StatusRegister::InputVoltage, f32);
register!(StatusRegister::WindingTemp, f32);
register!(StatusRegister::PowerstageTemp, f32);
register!(StatusRegister::IcTemp, f32);
register!(StatusRegister::ErrorStatus, f32);
register!(StatusRegister::WarningStatus, f32);
