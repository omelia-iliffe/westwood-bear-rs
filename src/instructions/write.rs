use crate::bus::Bus;
use crate::error::WriteError;
use crate::registers::WritableRegister;
use crate::{ConfigRegister, Instruction, StatusRegister};

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub(crate) fn write_raw(
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
        })
    }
    pub fn write_config(
        &mut self,
        motor_id: u8,
        config_register: ConfigRegister,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, Instruction::WriteCfg as u8, config_register as u8, data)
    }
    pub fn write_status(
        &mut self,
        motor_id: u8,
        status_register: StatusRegister,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, Instruction::WriteStat as u8, status_register as u8, data)
    }
    pub fn write<R: WritableRegister>(
        &mut self,
        motor_id: u8,
        data: R::Inner,
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_raw(motor_id, R::WRITE_INST, R::ADDRESS, &R::encode_bytes(data))
    }
}
