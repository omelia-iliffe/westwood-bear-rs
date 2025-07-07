use crate::bus::Bus;
use crate::error::TransferError;
use crate::protocol::Response;
use crate::{ConfigRegister, StatusRegister};

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub(crate) fn read_raw(
        &mut self,
        motor_id: u8,
        instruction_id: u8,
        addr: u8,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.transfer_single(motor_id, instruction_id, 1, 5, |buffer| {
            buffer[0] = addr;
            Ok(())
        })
    }

    /// Read a [`ConfigRegister`] from a specific motor
    pub fn read_config(
        &mut self,
        motor_id: u8,
        config_register: ConfigRegister,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, ConfigRegister::READ_INST, config_register as u8)
    }

    /// Read a [`StatusRegister`] from a specific motor
    pub fn read_status(
        &mut self,
        motor_id: u8,
        status_register: StatusRegister,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, StatusRegister::READ_INST, status_register as u8)
    }

    /// Read a register from a specific motor.
    ///
    /// The register is specificed as a generic parameter, ie `Bus::read<status::PresentPos>`,
    /// and are avaliable in the [`crate::protocol::registers::config`] and [`crate::protocol::registers::status`] modules.
    pub fn read<R: crate::Register>(
        &mut self,
        motor_id: u8,
    ) -> Result<Response<R::Inner>, TransferError<SerialPort::Error>> {
        let r = self.read_raw(motor_id, R::READ_INST, R::ADDRESS)?;
        let r = Response {
            motor_id: r.motor_id,
            warning: r.warning,
            data: R::decode(r.data)?,
        };
        Ok(r)
    }
}
