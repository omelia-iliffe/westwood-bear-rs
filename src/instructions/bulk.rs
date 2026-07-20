//! Bulk read/write of *status* registers across multiple motors in a single packet.
//!
//! The bulk instruction ([`Instruction::BulkComm`], `0x12`) is broadcast to all motors on the bus
//! (using the broadcast ID `0xFE`). It addresses a list of motors at once, reading and/or writing the
//! same set of [`StatusRegister`]s for every motor, with per-motor write values.
//!
//! Bulk is only available for status registers (the firmware reads/writes its status table for `0x12`).
//!
//! Because each motor reply borrows the bus' shared read buffer (and the next reply overwrites it),
//! the core [`Bus::bulk_read_write`] hands each reply to a callback. With the `"alloc"` feature, the
//! [`Bus::bulk_read_alloc`] convenience copies each reply into an owned [`Vec`].

use super::super::Bus;
use crate::error::{BufferTooSmallError, ReadError, TooManyRegistersError, TransferError, WriteError};
use crate::protocol::Response;
use crate::{BulkWriteData, Instruction, StatusRegister};

/// Broadcast ID used to address all motors with a bulk packet.
const BROADCAST_ID: u8 = 0xFE;

/// Bytes per register value on the wire (4 little-endian bytes).
const REGISTER_BYTES: usize = 4;

/// Maximum registers per direction in a bulk packet. The read and write counts
/// share one byte (a 4-bit nibble each), so each direction supports at most 15.
const MAX_BULK_REGISTERS: usize = 0x0F;

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
    /// This is the core bulk primitive; [`Bus::bulk_read`] and [`Bus::bulk_write`] are thin
    /// wrappers around it.
    ///
    /// - `devices`: one [`BulkWriteData`] per motor, in the order they appear in the packet. Each
    ///   pairs a `motor_id` with its encoded write bytes, so the id and its data travel together
    ///   instead of as two parallel slices. The iterator must report its length
    ///   ([`ExactSizeIterator`]). When `write_registers` is empty the `data` is ignored and may be
    ///   `&[]`; otherwise each `data` must contain `write_registers.len() * 4` encoded bytes
    ///   (4 little-endian bytes per register).
    /// - `read_registers`: the [`StatusRegister`]s to read from every motor (may be empty).
    /// - `write_registers`: the [`StatusRegister`]s to write to every motor (may be empty).
    /// - `on_response`: called once per expected motor reply, in the order the motors responded.
    ///   Each call receives a [`Result`]: on success the [`Response::data`] is the concatenated read
    ///   bytes (`read_registers.len() * 4` bytes). Decode each register by its position with
    ///   [`Response::f32`] / [`Response::u32`] (e.g. `response.f32(0)`), or split the bytes into
    ///   4-byte chunks and decode manually with [`f32::from_le_bytes`] / [`u32::from_le_bytes`]
    ///   according to the register type. A reply that
    ///   fails to read (e.g. a motor times out) is delivered as an [`Err`] and the remaining replies
    ///   are still drained, so one bad reply does not abort the rest. When `read_registers` is empty
    ///   no reply is sent and `on_response` is never called.
    pub async fn bulk_read_write<Iter, Data, T, F>(
        &mut self,
        devices: Iter,
        read_registers: &[StatusRegister],
        write_registers: &[StatusRegister],
        mut on_response: F,
    ) -> Result<(), TransferError<SerialPort::Error>>
    where
        Iter: IntoIterator<Item = Data>,
        Iter::IntoIter: ExactSizeIterator,
        Data: AsRef<BulkWriteData<T>>,
        T: AsRef<[u8]>,
        F: FnMut(Result<Response<&[u8]>, ReadError<SerialPort::Error>>),
    {
        let devices = devices.into_iter();
        let motor_count = devices.len();
        let read_count = read_registers.len();
        let write_count = write_registers.len();
        let write_len = write_count * REGISTER_BYTES;

        // The read and write counts are packed into a single byte (a nibble each),
        // so reject anything that would overflow the nibble instead of silently truncating.
        TooManyRegistersError::check(read_count, MAX_BULK_REGISTERS).map_err(WriteError::from)?;
        TooManyRegistersError::check(write_count, MAX_BULK_REGISTERS).map_err(WriteError::from)?;

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
            for device in devices {
                let device = device.as_ref();
                buffer[idx] = device.motor_id;
                idx += 1;
                if write_len > 0 {
                    let row = device.data.as_ref();
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

        // One status reply per motor, in the order they respond. A failed reply (e.g. a motor
        // times out) is handed to the callback as an `Err`; we keep draining so one bad reply
        // doesn't strand the remaining replies in the buffer or abort the rest.
        let expected_parameters = (read_count * REGISTER_BYTES + REPLY_FRAMING_BYTES) as u8;
        for _ in 0..motor_count {
            let response = self.read_response(expected_parameters).await;
            on_response(response);
        }
        Ok(())
    }

    /// Bulk read status registers from multiple motors in a single packet.
    ///
    /// See [`Bus::bulk_read_write`] for the meaning of the arguments and `on_response` callback.
    pub async fn bulk_read<F>(
        &mut self,
        motor_ids: &[u8],
        read_registers: &[StatusRegister],
        on_response: F,
    ) -> Result<(), TransferError<SerialPort::Error>>
    where
        F: FnMut(Result<Response<&[u8]>, ReadError<SerialPort::Error>>),
    {
        // Read-only bulk carries no per-motor write data, so each entry is just the id.
        let devices = motor_ids.iter().map(|&motor_id| BulkWriteData {
            motor_id,
            data: &[][..],
        });
        self.bulk_read_write(devices, read_registers, &[], on_response).await
    }

    /// Bulk write status registers to multiple motors in a single packet.
    ///
    /// `devices` yields one [`BulkWriteData`] per motor, each pairing the motor id with its
    /// `write_registers.len() * 4` encoded bytes. No reply is expected from the motors.
    pub async fn bulk_write<Iter, Data, T>(
        &mut self,
        devices: Iter,
        write_registers: &[StatusRegister],
    ) -> Result<(), TransferError<SerialPort::Error>>
    where
        Iter: IntoIterator<Item = Data>,
        Iter::IntoIter: ExactSizeIterator,
        Data: AsRef<BulkWriteData<T>>,
        T: AsRef<[u8]>,
    {
        self.bulk_read_write(devices, &[], write_registers, |_| {}).await
    }

    /// Bulk read status registers, returning one reply per motor as owned [`Vec`]s.
    ///
    /// Convenience wrapper around [`Bus::bulk_read`] available with the `"alloc"` feature. The
    /// returned `Vec` holds one entry per motor, in the order the motors responded: an [`Ok`] reply
    /// whose [`Response::data`] holds `read_registers.len() * 4` bytes (decode in 4-byte chunks), or
    /// an [`Err`] for a reply that failed to read (e.g. a motor timed out).
    #[cfg(feature = "alloc")]
    pub async fn bulk_read_alloc(
        &mut self,
        motor_ids: &[u8],
        read_registers: &[StatusRegister],
    ) -> Result<
        alloc::vec::Vec<Result<Response<alloc::vec::Vec<u8>>, ReadError<SerialPort::Error>>>,
        TransferError<SerialPort::Error>,
    > {
        let mut results = alloc::vec::Vec::with_capacity(motor_ids.len());
        self.bulk_read(motor_ids, read_registers, |response| {
            results.push(response.map(|response| Response {
                motor_id: response.motor_id,
                warning: response.warning,
                data: response.data.to_vec(),
            }));
        })
        .await?;
        Ok(results)
    }

    /// Bulk read and write status registers, returning one read reply per motor as owned [`Vec`]s.
    ///
    /// Convenience wrapper around [`Bus::bulk_read_write`] available with the `"alloc"` feature. The
    /// returned `Vec` holds one entry per motor, in the order the motors responded: an [`Ok`] reply,
    /// or an [`Err`] for a reply that failed to read (see [`Bus::bulk_read_alloc`]).
    #[cfg(feature = "alloc")]
    pub async fn bulk_read_write_alloc<Iter, Data, T>(
        &mut self,
        devices: Iter,
        read_registers: &[StatusRegister],
        write_registers: &[StatusRegister],
    ) -> Result<
        alloc::vec::Vec<Result<Response<alloc::vec::Vec<u8>>, ReadError<SerialPort::Error>>>,
        TransferError<SerialPort::Error>,
    >
    where
        Iter: IntoIterator<Item = Data>,
        Iter::IntoIter: ExactSizeIterator,
        Data: AsRef<BulkWriteData<T>>,
        T: AsRef<[u8]>,
    {
        let mut results = alloc::vec::Vec::new();
        self.bulk_read_write(devices, read_registers, write_registers, |response| {
            results.push(response.map(|response| Response {
                motor_id: response.motor_id,
                warning: response.warning,
                data: response.data.to_vec(),
            }));
        })
        .await?;
        Ok(results)
    }
}
