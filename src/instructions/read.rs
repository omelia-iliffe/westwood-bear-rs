use super::super::Bus;
use crate::error::TransferError;
use crate::protocol::Response;
use crate::{ConfigAddr, ConfigRegister, StatusAddr, StatusRegister};

#[super::super::bisync]
impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: super::super::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub(crate) async fn read_raw(
        &mut self,
        motor_id: u8,
        instruction_id: u8,
        addr: u8,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.transfer_single(motor_id, instruction_id, 1, 5, |buffer| {
            buffer[0] = addr;
            Ok(())
        }).await
    }

    /// Read a config register from a specific motor.
    ///
    /// Takes a [`ConfigRegister`] for the standard table, or a `u8` for any
    /// other index the device implements. See [`ConfigAddr`].
    pub async fn read_config(
        &mut self,
        motor_id: u8,
        register: impl Into<ConfigAddr>,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, ConfigRegister::READ_INST, register.into().0).await
    }

    /// Read a status register from a specific motor.
    ///
    /// Takes a [`StatusRegister`] for the standard table, or a `u8` for any
    /// other index the device implements. See [`StatusAddr`].
    pub async fn read_status(
        &mut self,
        motor_id: u8,
        register: impl Into<StatusAddr>,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, StatusRegister::READ_INST, register.into().0).await
    }

    /// Read a register from a specific motor.
    ///
    /// The register is specificed as a generic parameter, ie `Bus::read<status::PresentPos>`,
    /// and are avaliable in the [`crate::protocol::registers::config`] and [`crate::protocol::registers::status`] modules.
    pub async fn read<R: crate::Register>(
        &mut self,
        motor_id: u8,
    ) -> Result<Response<R::Inner>, TransferError<SerialPort::Error>> {
        let r = self.read_raw(motor_id, R::READ_INST, R::ADDRESS).await?;
        let r = Response {
            motor_id: r.motor_id,
            warning: r.warning,
            data: R::decode(r.data)?,
        };
        Ok(r)
    }
}
