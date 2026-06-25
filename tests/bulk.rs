//! Tests for the bulk read/write instruction against a mock serial port.
//!
//! These assert the exact request bytes produced (cross-checked against the firmware packet layout)
//! and that per-motor responses are parsed and dispatched correctly.

use std::time::Duration;

use ww_bear::{BulkWriteData, Bus, SerialPort, StatusRegister};

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
fn status_packet(id: u8, error: u8, data: &[u8]) -> Vec<u8> {
    let len = (2 + data.len()) as u8;
    let mut p = vec![0xFF, 0xFF, id, len, error];
    p.extend_from_slice(data);
    let crc = checksum(&p[2..]);
    p.push(crc);
    p
}

fn open(to_read: Vec<u8>) -> Bus<MockPort, Vec<u8>> {
    Bus::<MockPort, Vec<u8>>::with_buffers(MockPort::new(to_read), vec![0u8; 128], vec![0u8; 128]).unwrap()
}

#[test]
fn bulk_read_request_and_responses() {
    // Two motors, reading PresentPos (0x09) and PresentVel (0x08).
    let m1_data = [0u8, 0, 0x80, 0x3F, 0, 0, 0, 0x40]; // 1.0f32, 2.0f32
    let m2_data = [0u8, 0, 0x40, 0x40, 0, 0, 0x80, 0x40]; // 3.0f32, 4.0f32
    let mut responses = status_packet(1, 0x80, &m1_data);
    responses.extend_from_slice(&status_packet(2, 0x80, &m2_data));

    let mut bus = open(responses);

    let mut got: Vec<(u8, Vec<u8>)> = Vec::new();
    bus.bulk_read(
        &[1, 2],
        &[StatusRegister::PresentPos, StatusRegister::PresentVel],
        |r| {
            let r = r.unwrap();
            got.push((r.motor_id, r.data.to_vec()));
        },
    )
    .unwrap();

    // Exact request bytes: FF FF FE LEN 12 M flags read_addrs... crc
    let expected = [0xFF, 0xFF, 0xFE, 0x08, 0x12, 0x02, 0x20, 0x09, 0x08, 0x01, 0x02, 0xB1];
    assert_eq!(bus.serial_port().written, expected);

    assert_eq!(got.len(), 2);
    assert_eq!(got[0], (1, m1_data.to_vec()));
    assert_eq!(got[1], (2, m2_data.to_vec()));
}

#[test]
fn bulk_read_write_request() {
    // Two motors: read PresentPos (0x09), write GoalPos (0x05) with per-motor data.
    let m1_data = [0xAAu8, 0xBB, 0xCC, 0xDD];
    let m2_data = [0x11u8, 0x22, 0x33, 0x44];

    // Read replies (4 bytes each) so the read loop completes.
    let mut responses = status_packet(1, 0x80, &[1, 2, 3, 4]);
    responses.extend_from_slice(&status_packet(2, 0x80, &[5, 6, 7, 8]));

    let devices = [
        BulkWriteData {
            motor_id: 1,
            data: &m1_data[..],
        },
        BulkWriteData {
            motor_id: 2,
            data: &m2_data[..],
        },
    ];
    let mut bus = open(responses);
    bus.bulk_read_write(
        &devices,
        &[StatusRegister::PresentPos],
        &[StatusRegister::GoalPos],
        |_| {},
    )
    .unwrap();

    let expected = [
        0xFF, 0xFF, 0xFE, 0x10, 0x12, 0x02, 0x11, 0x09, 0x05, 0x01, 0xAA, 0xBB, 0xCC, 0xDD, 0x02, 0x11, 0x22, 0x33,
        0x44, 0x03,
    ];
    assert_eq!(bus.serial_port().written, expected);
}

#[test]
fn bulk_write_only_sends_no_read() {
    let devices = [
        BulkWriteData {
            motor_id: 1,
            data: [1u8, 2, 3, 4],
        },
        BulkWriteData {
            motor_id: 2,
            data: [5u8, 6, 7, 8],
        },
    ];
    let mut bus = open(Vec::new());
    bus.bulk_write(&devices, &[StatusRegister::GoalPos]).unwrap();

    let expected = [
        0xFF, 0xFF, 0xFE, 0x0F, 0x12, 0x02, 0x01, 0x05, 0x01, 0x01, 0x02, 0x03, 0x04, 0x02, 0x05, 0x06, 0x07, 0x08,
        0xB1,
    ];
    assert_eq!(bus.serial_port().written, expected);
    // Write-only: no reply should have been read.
    assert_eq!(bus.serial_port().read_pos, 0);
}

#[test]
fn bulk_read_alloc_returns_owned() {
    let m1_data = [0u8, 0, 0x80, 0x3F];
    let m2_data = [0u8, 0, 0x40, 0x40];
    let mut responses = status_packet(7, 0x80, &m1_data);
    responses.extend_from_slice(&status_packet(9, 0x80, &m2_data));

    let mut bus = open(responses);
    let replies = bus.bulk_read_alloc(&[7, 9], &[StatusRegister::PresentPos]).unwrap();

    assert_eq!(replies.len(), 2);
    let r0 = replies[0].as_ref().unwrap();
    assert_eq!(r0.motor_id, 7);
    assert_eq!(r0.data, m1_data.to_vec());
    let r1 = replies[1].as_ref().unwrap();
    assert_eq!(r1.motor_id, 9);
    assert_eq!(r1.data, m2_data.to_vec());
}
