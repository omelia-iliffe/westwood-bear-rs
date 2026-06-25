//! Bulk read/write of *status* registers across multiple motors in a single packet.
//!
//! The bulk instruction ([`Instruction::BulkComm`], `0x12`) is broadcast to all motors on the bus
//! (using the broadcast ID `0xFE`). It addresses a list of motors at once, reading and/or writing the
//! same set of [`StatusRegister`]s for every motor, with per-motor write values.
//!
//! Bulk is only available for status registers (the firmware reads/writes its status table for `0x12`).
//!
//! Because each motor reply borrows the bus' shared read buffer (and the next reply overwrites it),
//! the core [`Bus::bulk_comm`] hands each reply to a callback. With the `"alloc"` feature, the
//! [`Bus::bulk_read_alloc`] convenience copies each reply into an owned [`Vec`].

use super::super::Bus;
use crate::error::{BufferTooSmallError, TransferError};
use crate::protocol::Response;
use crate::{Instruction, StatusRegister};

/// Broadcast ID used to address all motors with a bulk packet.
const BROADCAST_ID: u8 = 0xFE;

/// Bytes per register value on the wire (4 little-endian bytes).
const REGISTER_BYTES: usize = 4;

/// Non-payload bytes in a status reply, added to the read length to estimate the
/// reply's on-wire size for the read timeout.
const REPLY_FRAMING_BYTES: usize = 3;

#[super::super::bisync]
impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: super::super::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    /// Bulk read and/or write status registers across multiple motors in a single packet.
    ///
    /// This is the core bulk primitive; [`Bus::bulk_read`], [`Bus::bulk_write`] and
    /// [`Bus::bulk_read_write`] are thin wrappers around it.
    ///
    /// - `motor_ids`: the motors to address, in the order they appear in the packet.
    /// - `read_registers`: the [`StatusRegister`]s to read from every motor (may be empty).
    /// - `write_registers`: the [`StatusRegister`]s to write to every motor (may be empty).
    /// - `write_data`: one entry per motor (same order as `motor_ids`). Each entry must contain
    ///   `write_registers.len() * 4` encoded bytes (4 little-endian bytes per register). When
    ///   `write_registers` is empty this argument is ignored and may be `&[]`.
    /// - `on_response`: called once per motor reply, in the order the motors responded. The
    ///   [`Response::data`] is the concatenated read bytes (`read_registers.len() * 4` bytes); split
    ///   it into 4-byte chunks and decode each with [`f32::from_le_bytes`] / [`u32::from_le_bytes`]
    ///   according to the register type. When `read_registers` is empty no reply is sent and
    ///   `on_response` is never called.
    pub async fn bulk_comm<F>(
        &mut self,
        motor_ids: &[u8],
        read_registers: &[StatusRegister],
        write_registers: &[StatusRegister],
        write_data: &[&[u8]],
        mut on_response: F,
    ) -> Result<(), TransferError<SerialPort::Error>>
    where
        F: FnMut(Response<&[u8]>),
    {
        let motor_count = motor_ids.len();
        let read_count = read_registers.len();
        let write_count = write_registers.len();
        let write_len = write_count * REGISTER_BYTES;

        let parameter_count = 2 + read_count + write_count + motor_count * (1 + write_len);

        self.write_packet(BROADCAST_ID, Instruction::BulkComm as u8, parameter_count, |buffer| {
            buffer[0] = motor_count as u8;
            buffer[1] = ((read_count as u8) << 4) | (write_count as u8);
            let mut idx = 2;
            for register in read_registers {
                buffer[idx] = *register as u8;
                idx += 1;
            }
            for register in write_registers {
                buffer[idx] = *register as u8;
                idx += 1;
            }
            for (i, &motor_id) in motor_ids.iter().enumerate() {
                buffer[idx] = motor_id;
                idx += 1;
                if write_len > 0 {
                    let row = write_data[i];
                    BufferTooSmallError::check(write_len, row.len())?;
                    buffer[idx..idx + write_len].copy_from_slice(&row[..write_len]);
                    idx += write_len;
                }
            }
            Ok(())
        })
        .await?;

        // Write-only bulk: the firmware sends no reply.
        if read_count == 0 {
            return Ok(());
        }

        // One status reply per motor, in the order they respond.
        let expected_parameters = (read_count * REGISTER_BYTES + REPLY_FRAMING_BYTES) as u8;
        for _ in 0..motor_count {
            let response = self.read_response(expected_parameters).await?;
            on_response(response);
        }
        Ok(())
    }

    /// Bulk read status registers from multiple motors in a single packet.
    ///
    /// See [`Bus::bulk_comm`] for the meaning of the arguments and `on_response` callback.
    pub async fn bulk_read<F>(
        &mut self,
        motor_ids: &[u8],
        read_registers: &[StatusRegister],
        on_response: F,
    ) -> Result<(), TransferError<SerialPort::Error>>
    where
        F: FnMut(Response<&[u8]>),
    {
        self.bulk_comm(motor_ids, read_registers, &[], &[], on_response).await
    }

    /// Bulk write status registers to multiple motors in a single packet.
    ///
    /// `write_data` has one entry per motor (same order as `motor_ids`), each containing
    /// `write_registers.len() * 4` encoded bytes. No reply is expected from the motors.
    pub async fn bulk_write(
        &mut self,
        motor_ids: &[u8],
        write_registers: &[StatusRegister],
        write_data: &[&[u8]],
    ) -> Result<(), TransferError<SerialPort::Error>> {
        self.bulk_comm(motor_ids, &[], write_registers, write_data, |_| {}).await
    }

    /// Bulk read and write status registers across multiple motors in a single packet.
    ///
    /// Equivalent to [`Bus::bulk_comm`]; provided to mirror the read/write naming.
    pub async fn bulk_read_write<F>(
        &mut self,
        motor_ids: &[u8],
        read_registers: &[StatusRegister],
        write_registers: &[StatusRegister],
        write_data: &[&[u8]],
        on_response: F,
    ) -> Result<(), TransferError<SerialPort::Error>>
    where
        F: FnMut(Response<&[u8]>),
    {
        self.bulk_comm(motor_ids, read_registers, write_registers, write_data, on_response).await
    }

    /// Bulk read status registers, returning the replies as owned [`Vec`]s.
    ///
    /// Convenience wrapper around [`Bus::bulk_read`] available with the `"alloc"` feature. Each
    /// [`Response::data`] holds `read_registers.len() * 4` bytes; decode in 4-byte chunks.
    #[cfg(feature = "alloc")]
    pub async fn bulk_read_alloc(
        &mut self,
        motor_ids: &[u8],
        read_registers: &[StatusRegister],
    ) -> Result<alloc::vec::Vec<Response<alloc::vec::Vec<u8>>>, TransferError<SerialPort::Error>> {
        let mut results = alloc::vec::Vec::with_capacity(motor_ids.len());
        self.bulk_read(motor_ids, read_registers, |response| {
            results.push(Response {
                motor_id: response.motor_id,
                warning: response.warning,
                data: response.data.to_vec(),
            });
        })
        .await?;
        Ok(results)
    }

    /// Bulk read and write status registers, returning the read replies as owned [`Vec`]s.
    ///
    /// Convenience wrapper around [`Bus::bulk_read_write`] available with the `"alloc"` feature.
    #[cfg(feature = "alloc")]
    pub async fn bulk_read_write_alloc(
        &mut self,
        motor_ids: &[u8],
        read_registers: &[StatusRegister],
        write_registers: &[StatusRegister],
        write_data: &[&[u8]],
    ) -> Result<alloc::vec::Vec<Response<alloc::vec::Vec<u8>>>, TransferError<SerialPort::Error>> {
        let mut results = alloc::vec::Vec::with_capacity(motor_ids.len());
        self.bulk_read_write(motor_ids, read_registers, write_registers, write_data, |response| {
            results.push(Response {
                motor_id: response.motor_id,
                warning: response.warning,
                data: response.data.to_vec(),
            });
        })
        .await?;
        Ok(results)
    }
}
