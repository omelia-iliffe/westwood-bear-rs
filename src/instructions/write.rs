use super::super::Bus;
use crate::error::WriteError;
use crate::registers::WritableRegister;
use crate::{ConfigAddr, Instruction, StatusAddr};

#[super::super::bisync]
impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: super::super::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub(crate) async fn write_raw(
        &mut self,
        motor_id: u8,
        instruction_id: u8,
        register: u8,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_packet(motor_id, instruction_id, data.len() + 1, |buffer| {
            buffer[0] = register;
            buffer[1..].copy_from_slice(data);
            Ok(())
        }).await
    }
    /// Write a config register to a specific motor.
    ///
    /// Takes a [`ConfigRegister`](crate::ConfigRegister) for the standard table,
    /// or a `u8` for any other index the device implements. See [`ConfigAddr`].
    ///
    /// `data` is the encoded value: 4 little-endian bytes, `f32` or `u32`
    /// according to the register. Nothing acknowledges a write on this protocol,
    /// so read the register back if you need to know it landed.
    pub async fn write_config(
        &mut self,
        motor_id: u8,
        register: impl Into<ConfigAddr>,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, Instruction::WriteCfg as u8, register.into().0, data).await
    }

    /// Write a status register to a specific motor.
    ///
    /// Takes a [`StatusRegister`](crate::StatusRegister) for the standard table,
    /// or a `u8` for any other index the device implements. See [`StatusAddr`].
    ///
    /// `data` is encoded as for [`Self::write_config`].
    pub async fn write_status(
        &mut self,
        motor_id: u8,
        register: impl Into<StatusAddr>,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, Instruction::WriteStat as u8, register.into().0, data).await
    }

    /// Write a register to a specific motor.
    ///
    /// The register is specificed as a generic parameter, ie `Bus::write<config::TorqueEnable>`,
    /// and are avaliable in the [`crate::protocol::registers::config`] and [`crate::protocol::registers::status`] modules.
    pub async fn write<R: WritableRegister>(
        &mut self,
        motor_id: u8,
        data: R::Inner,
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, R::WRITE_INST, R::ADDRESS, &R::encode_bytes(data)).await
    }
}
