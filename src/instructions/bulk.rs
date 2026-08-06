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
use super::super::bus::message_transfer_time;
use crate::error::{
    BufferTooSmallError, InvalidPacketId, InvalidParameterCount, ReadError, TooManyRegistersError, TransferError,
    WriteError,
};
use crate::protocol::Response;
use crate::{BulkWriteData, Instruction, StatusRegister};
use core::time::Duration;

/// Broadcast ID used to address all motors with a bulk packet.
const BROADCAST_ID: u8 = 0xFE;

/// Byte offset of the parameter section within a written packet: `FF FF`, id, len, instruction.
const PACKET_PARAMS_START: usize = 5;

/// Bytes per register value on the wire (4 little-endian bytes).
const REGISTER_BYTES: usize = 4;

/// Maximum registers per direction in a bulk packet. The read and write counts
/// share one byte (a 4-bit nibble each), so each direction supports at most 15.
const MAX_BULK_REGISTERS: usize = 0x0F;

/// Non-payload bytes framing a packet on the wire: `FF FF`, id, length, instruction (in a request)
/// or error (in a reply), and the trailing checksum.
///
/// The same six bytes frame the request written and every status reply read back, so adding this to
/// a payload length gives the number of bytes that actually cross the wire.
const PACKET_FRAMING_BYTES: usize = 6;

/// Time budget covering the entire burst of replies to a bulk read.
///
/// The motors answer back-to-back, and the host's serial hardware does not hand bytes to userspace
/// one reply at a time: it delivers in chunks, when a FIFO trigger or an idle timeout is reached. A
/// single read can therefore return anything from part of one reply to the whole burst. Budgeting
/// each read for one reply means the *first* read waits for most of the burst on a single reply's
/// allowance, times out with those bytes still buffered, and hands them to the next read, shifting
/// every later reply one slot early so the last motor is never read at all. One deadline sized for
/// the whole burst, shared across every read, is the budget that matches what is on the wire.
///
/// The request is counted because [`super::super::SerialPort::write_all`] returns once the kernel
/// has accepted the packet, not once it has been transmitted, so it may still be draining when the
/// first read begins. `padding` is [`Bus::response_timeout_padding`], which becomes headroom on top
/// of a correctly sized budget: the motors' own turnaround plus any scheduling jitter.
fn bulk_burst_timeout(
    request_parameters: usize,
    motor_count: usize,
    read_count: usize,
    baud_rate: u32,
    padding: Duration,
) -> Duration {
    let request_bytes = PACKET_FRAMING_BYTES + request_parameters;
    let reply_bytes = motor_count * (PACKET_FRAMING_BYTES + read_count * REGISTER_BYTES);
    message_transfer_time((request_bytes + reply_bytes) as u32, baud_rate) + padding
}

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
    ///   fails to read (e.g. a motor times out), whose id doesn't match the expected motor, or whose
    ///   data length is wrong is delivered as an [`Err`] in that slot; the remaining replies are still
    ///   drained, so one bad reply does not abort the rest. When `read_registers` is empty no reply is
    ///   sent and `on_response` is never called.
    ///
    /// All the replies share a single deadline, sized for the whole burst from the byte count of the
    /// request and of every expected reply (plus [`Bus::response_timeout_padding`] as headroom), so
    /// the batch is not cut short once it grows past what one reply's budget would allow. A motor
    /// that never answers therefore costs the whole burst deadline rather than one reply's worth of
    /// it, which is slower to notice than a per-reply timeout but does not corrupt the replies that
    /// follow it.
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

        // Recover each motor's expected id from the packet still held in the write buffer.
        let write_stride = 1 + write_len;
        let first_id_index = PACKET_PARAMS_START + 2 + read_count + write_count;
        let expected_data_len = read_count * REGISTER_BYTES;

        // One deadline for the whole burst, shared by every read below rather than recomputed per
        // reply. See `bulk_burst_timeout` for why a per-reply budget silently loses the last motor
        // once the batch outgrows it.
        let timeout = bulk_burst_timeout(
            parameter_count,
            motor_count,
            read_count,
            self.baud_rate,
            self.response_timeout_padding,
        );
        let deadline = self.serial_port.make_deadline(timeout);

        for i in 0..motor_count {
            let expected_id = self.write_buffer.as_ref()[first_id_index + i * write_stride];
            let response = self.read_response_deadline(deadline).await.and_then(|response| {
                InvalidPacketId::check(response.motor_id, expected_id)?;
                InvalidParameterCount::check(response.data.len(), expected_data_len)?;
                Ok(response)
            });
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

#[cfg(test)]
mod tests {
    use super::*;

    const BAUD: u32 = 2_000_000;

    /// Parameter bytes of a read-only bulk request, mirroring `parameter_count` in
    /// [`Bus::bulk_read_write`]: the motor count, the register-count nibbles, one register address
    /// per read register, and one id per motor.
    fn read_only_parameters(motor_count: usize, read_count: usize) -> usize {
        2 + read_count + motor_count
    }

    /// Bytes one motor's reply puts on the wire.
    fn reply_frame_bytes(read_count: usize) -> usize {
        PACKET_FRAMING_BYTES + read_count * REGISTER_BYTES
    }

    /// Each extra motor adds a whole reply frame to the wire, so the budget must grow by that
    /// frame's transfer time. This is the property that was broken: a per-reply budget does not
    /// move with the motor count at all, so past a certain batch size the first read times out
    /// mid-burst and every later reply lands one slot early.
    #[test]
    fn burst_timeout_grows_by_one_reply_frame_per_motor() {
        for read_count in [1, 6, MAX_BULK_REGISTERS] {
            let at = |motor_count| {
                bulk_burst_timeout(
                    read_only_parameters(motor_count, read_count),
                    motor_count,
                    read_count,
                    BAUD,
                    Duration::ZERO,
                )
            };
            // One id byte joins the request alongside the motor's reply frame.
            let step = message_transfer_time(reply_frame_bytes(read_count) as u32 + 1, BAUD);

            assert_eq!(at(2) - at(1), step, "read_count {read_count}");
            assert_eq!(at(12) - at(11), step, "read_count {read_count}");
            assert_eq!(at(12) - at(1), step * 11, "read_count {read_count}");
        }
    }

    /// Each extra register widens every motor's reply by four payload bytes and adds one register
    /// address to the request, so the budget must scale with the read set too.
    #[test]
    fn burst_timeout_grows_with_registers_read() {
        for motor_count in [1, 6, 12] {
            let at = |read_count| {
                bulk_burst_timeout(
                    read_only_parameters(motor_count, read_count),
                    motor_count,
                    read_count,
                    BAUD,
                    Duration::ZERO,
                )
            };
            let step = message_transfer_time((motor_count * REGISTER_BYTES) as u32 + 1, BAUD);

            assert_eq!(at(2) - at(1), step, "motor_count {motor_count}");
            assert_eq!(at(6) - at(5), step, "motor_count {motor_count}");
        }
    }

    /// The configuration this was diagnosed against: 12 motors, a 6-register read, 2 Mbaud. The
    /// request is 26 bytes and each of the 12 replies is 30, so 386 bytes cross the wire — 1.93 ms,
    /// matching the 1930 µs of measured wire time within a 2161 µs round trip.
    #[test]
    fn burst_timeout_covers_measured_twelve_motor_read() {
        let padding = Duration::from_millis(3);
        let timeout = bulk_burst_timeout(read_only_parameters(12, 6), 12, 6, BAUD, padding);

        assert_eq!(reply_frame_bytes(6), 30);
        assert_eq!(timeout, Duration::from_micros(1930) + padding);

        // Every reply is covered, not just the one the old per-reply budget accounted for.
        assert!(timeout >= message_transfer_time((12 * reply_frame_bytes(6)) as u32, BAUD));
        assert!(timeout > message_transfer_time(reply_frame_bytes(6) as u32, BAUD) + padding);
    }

    /// The padding is headroom on top of the wire time, not a substitute for it.
    #[test]
    fn burst_timeout_adds_padding_on_top_of_wire_time() {
        let at = |padding| bulk_burst_timeout(read_only_parameters(12, 6), 12, 6, BAUD, padding);

        assert_eq!(
            at(Duration::from_millis(3)) - at(Duration::ZERO),
            Duration::from_millis(3)
        );
    }
}
