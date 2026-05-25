use super::super::Bus;
use crate::error::WriteError;
use crate::registers::WritableRegister;
use crate::{ConfigRegister, Instruction, StatusRegister};

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
    /// Write a [`ConfigRegister`] to a specific motor
    ///
    /// The data parameter is an encoded byte slice. Encoding is either a f32 or u32 depending on the register.
    pub async fn write_config(
        &mut self,
        motor_id: u8,
        config_register: ConfigRegister,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, Instruction::WriteCfg as u8, config_register as u8, data).await   }

    /// Write a [`StatusRegister`] to a specific motor
    ///
    /// The data parameter is an encoded byte slice. Encoding is either a f32 or u32 depending on the register.
    pub async fn write_status(
        &mut self,
        motor_id: u8,
        status_register: StatusRegister,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, Instruction::WriteStat as u8, status_register as u8, data).await   }

    /// Write a register to a specific motor.
    ///
    /// The register is specificed as a generic parameter, ie `Bus::write<config::TorqueEnable>`,
    /// and are avaliable in the [`crate::protocol::registers::config`] and [`crate::protocol::registers::status`] modules.
    pub async fn write<R: WritableRegister>(
        &mut self,
        motor_id: u8,
        data: R::Inner,
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, R::WRITE_INST, R::ADDRESS, &R::encode_bytes(data)).await   }
}
