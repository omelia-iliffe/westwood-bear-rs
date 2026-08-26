use super::super::Bus;
use crate::error::TransferError;
use crate::protocol::Response;
use crate::{ConfigRegister, StatusRegister};

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

    /// Read a status register by raw index.
    ///
    /// [`Self::read_status`] takes a [`StatusRegister`], which covers the
    /// standard table. A register index is a full byte on the wire, though, and
    /// a device may implement vendor registers above that table -- the enum is
    /// `#[non_exhaustive]` precisely because it does not claim to be the whole
    /// address space. This is how a client reaches them.
    ///
    /// The device decides what is valid: an index it does not implement gets no
    /// reply, which surfaces here as a timeout.
    pub async fn read_status_raw(
        &mut self,
        motor_id: u8,
        register: u8,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, StatusRegister::READ_INST, register).await
    }

    /// Read a config register by raw index. See [`Self::read_status_raw`].
    pub async fn read_config_raw(
        &mut self,
        motor_id: u8,
        register: u8,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, ConfigRegister::READ_INST, register).await
    }

    /// Read a [`ConfigRegister`] from a specific motor
    pub async fn read_config(
        &mut self,
        motor_id: u8,
        config_register: ConfigRegister,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, ConfigRegister::READ_INST, config_register as u8).await
    }

    /// Read a [`StatusRegister`] from a specific motor
    pub async fn read_status(
        &mut self,
        motor_id: u8,
        status_register: StatusRegister,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.read_raw(motor_id, StatusRegister::READ_INST, status_register as u8).await
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
