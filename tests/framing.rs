//! Framing and resynchronisation tests against a mock serial port.
//!
//! These cover the read loop's recovery behaviour and the one-byte `LEN`
//! field's limits, both of which are only reachable through the client's
//! multi-reply path: [`Bus::bulk_read`] issues one instruction and then reads a
//! reply per motor, so a frame that fails validation is followed by a good one
//! that must still arrive.

use std::time::Duration;

use ww_bear::error::{TransferError, WriteError};
use ww_bear::{BulkWriteData, Bus, MAX_PACKET_SIZE, MAX_PARAMETER_COUNT, SerialPort, StatusRegister};

/// A fake serial port that records written bytes and serves scripted bytes to reads.
struct MockPort {
    written: Vec<u8>,
    to_read: Vec<u8>,
    read_pos: usize,
    baud: u32,
}

impl MockPort {
    fn new(to_read: Vec<u8>) -> Self {
        Self {
            written: Vec::new(),
            to_read,
            read_pos: 0,
            baud: 8_000_000,
        }
    }
}

impl SerialPort for MockPort {
    type Error = std::io::Error;
    type Instant = ();

    fn baud_rate(&self) -> Result<u32, Self::Error> {
        Ok(self.baud)
    }

    fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), Self::Error> {
        self.baud = baud_rate;
        Ok(())
    }

    fn discard_input_buffer(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8], _deadline: &Self::Instant) -> Result<usize, Self::Error> {
        if self.read_pos >= self.to_read.len() {
            return Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "no more data"));
        }
        let n = (self.to_read.len() - self.read_pos).min(buffer.len());
        buffer[..n].copy_from_slice(&self.to_read[self.read_pos..self.read_pos + n]);
        self.read_pos += n;
        Ok(n)
    }

    fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        self.written.extend_from_slice(buffer);
        Ok(())
    }

    fn make_deadline(&self, _timeout: Duration) -> Self::Instant {}

    fn is_timeout_error(error: &Self::Error) -> bool {
        error.kind() == std::io::ErrorKind::TimedOut
    }
}

/// Checksum matching the crate: `255 - sum(bytes)` (wrapping).
fn checksum(bytes: &[u8]) -> u8 {
    let sum = bytes.iter().fold(0u8, |acc, b| acc.wrapping_add(*b));
    255u8.wrapping_sub(sum)
}

/// Build a single-motor status reply: `FF FF id len err [data] crc`, len = 2 + data.len().
fn status_packet(id: u8, data: &[u8]) -> Vec<u8> {
    let len = (2 + data.len()) as u8;
    let mut packet = vec![0xFF, 0xFF, id, len, 0x80];
    packet.extend_from_slice(data);
    let crc = checksum(&packet[2..]);
    packet.push(crc);
    packet
}

fn open_with(to_read: Vec<u8>, buffer_size: usize) -> Bus<MockPort, Vec<u8>> {
    Bus::<MockPort, Vec<u8>>::with_buffers(MockPort::new(to_read), vec![0u8; buffer_size], vec![0u8; buffer_size])
        .unwrap()
}

/// Collect one outcome per motor: `Ok((id, data))` or `Err(rendered error)`.
fn bulk_read_outcomes(bus: &mut Bus<MockPort, Vec<u8>>, ids: &[u8]) -> Vec<Result<(u8, Vec<u8>), String>> {
    let mut got = Vec::new();
    bus.bulk_read(ids, &[StatusRegister::PresentPos], |response| {
        got.push(match response {
            Ok(response) => Ok((response.motor_id, response.data.to_vec())),
            Err(error) => Err(format!("{error:?}")),
        });
    })
    .unwrap();
    got
}

/// A frame whose length byte was corrupted must cost only that frame.
///
/// The length used to resynchronise came from the frame that just failed its
/// checksum, so trusting it let one line glitch swallow the next reply as well.
#[test]
fn a_corrupted_length_does_not_swallow_the_following_reply() {
    let good = [0xAAu8, 0xAA, 0xAA, 0xAA];
    let mut wire = status_packet(1, &good);
    wire[3] = 0x0C; // LEN corrupted after the checksum was computed over 0x06.
    wire.extend_from_slice(&status_packet(2, &good));

    let mut bus = open_with(wire, 128);
    let got = bulk_read_outcomes(&mut bus, &[1, 2]);

    assert_eq!(got.len(), 2);
    assert!(got[0].is_err(), "the corrupted frame should fail validation");
    assert!(
        got[0].as_ref().unwrap_err().contains("InvalidChecksum"),
        "expected a checksum failure, got {:?}",
        got[0]
    );
    assert_eq!(
        got[1],
        Ok((2, good.to_vec())),
        "motor 2's reply was consumed along with the corrupted frame"
    );
}

/// A frame too large for the read buffer is dropped, not retried forever.
#[test]
fn an_oversize_reply_does_not_wedge_the_reader() {
    let good = [0xAAu8, 0xAA, 0xAA, 0xAA];
    // 140 payload bytes is a legal frame, but 146 on the wire does not fit 128.
    let mut wire = status_packet(1, &[0xAA; 140]);
    wire.extend_from_slice(&status_packet(2, &good));

    let mut bus = open_with(wire, 128);
    let got = bulk_read_outcomes(&mut bus, &[1, 2]);

    assert_eq!(got.len(), 2);
    assert!(
        got[0].as_ref().unwrap_err().contains("BufferFull"),
        "expected the oversize frame to be refused, got {:?}",
        got[0]
    );
    assert_eq!(
        got[1],
        Ok((2, good.to_vec())),
        "the reader wedged on the frame it could not hold"
    );
}

/// `LEN` is one byte, so an oversize parameter block cannot be framed at all.
///
/// A buffer large enough to hold the bytes does not make the frame legal, which
/// is what made this reachable: `bulk_read_write` does not bound its motor
/// count, so a caller passing large buffers could silently emit a frame whose
/// length byte disagreed with its contents.
#[test]
fn a_parameter_block_too_large_to_describe_is_refused() {
    const WRITE_REGISTERS: [StatusRegister; 15] = [StatusRegister::GoalPos; 15];

    // 5 motors: 2 + 15 + 5 * (1 + 60) = 322 parameter bytes. The buffers hold it.
    let devices: Vec<_> = [1u8, 2, 3, 4, 5]
        .into_iter()
        .map(|motor_id| BulkWriteData {
            motor_id,
            data: [0u8; 60],
        })
        .collect();

    let mut bus = open_with(Vec::new(), 512);
    let result = bus.bulk_read_write(devices, &[], &WRITE_REGISTERS, |_| {});
    assert!(
        matches!(result, Err(TransferError::WriteError(WriteError::BufferTooSmall(_)))),
        "expected a refusal, got {result:?}"
    );
    assert!(
        bus.serial_port().written.is_empty(),
        "nothing may reach the wire for a frame that cannot be described"
    );

    // 3 motors is 200 parameter bytes, which the length byte can describe.
    let devices: Vec<_> = [1u8, 2, 3]
        .into_iter()
        .map(|motor_id| BulkWriteData {
            motor_id,
            data: [0u8; 60],
        })
        .collect();
    bus.bulk_read_write(devices, &[], &WRITE_REGISTERS, |_| {})
        .expect("the largest describable block was refused");
    let written = bus.serial_port().written.clone();
    assert_eq!(written[3] as usize, 200 + 2, "length byte");
    assert_eq!(written.len(), 200 + 6);
}

/// The default buffers have to carry the largest reply the protocol allows.
///
/// These are facts about the wire format rather than about any run, so they are
/// asserted at compile time: a change that breaks one should fail the build.
#[test]
fn max_packet_size_covers_every_legal_frame() {
    // The firmware's config table is 31 addressable registers, so a reply
    // reading all of them carries 31 four-byte words plus six bytes of framing.
    const CONFIG_TABLE_REGISTERS: usize = 31;
    const FULL_CONFIG_REPLY: usize = CONFIG_TABLE_REGISTERS * 4 + 6;

    // LEN counts the instruction/error byte and the checksum.
    const { assert!(MAX_PARAMETER_COUNT == 253) };
    const { assert!(MAX_PACKET_SIZE == 259) };

    // The case the old 128 byte default could not serve.
    const { assert!(FULL_CONFIG_REPLY == 130) };
    const { assert!(FULL_CONFIG_REPLY <= MAX_PACKET_SIZE) };

    // Writing that same table in one packet is larger again: index plus value.
    const { assert!(CONFIG_TABLE_REGISTERS * 5 + 6 <= MAX_PACKET_SIZE) };
}
