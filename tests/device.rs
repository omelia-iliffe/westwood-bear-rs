//! Device-side tests: what a motor sees when the client half of this crate talks.
//!
//! Almost every test here is a round trip. The client builds a real instruction,
//! the bytes it put on the wire are handed straight to a [`Device`], and the
//! decode is asserted. That closes the loop without hardware and without a
//! hand-written byte script, so the two halves cannot drift apart: a change to
//! the encoder that the decoder does not expect fails here.
//!
//! The exceptions are the fixed-vector tests, which pin the wire format itself
//! against frames taken from the motor firmware rather than from this crate.

use std::time::Duration;

use ww_bear::error::{ReadError, WriteError};
use ww_bear::{
    BulkComm, BulkWriteData, Bus, ConfigRegister, Device, ErrorFlags, Instruction, MAX_PARAMETER_COUNT, Packet,
    PacketKind, SerialPort, StatusRegister, register_writes,
};

/// A fake serial port: records what is written, serves a fixed script to reads.
///
/// `Instant = ()` because the deadline in [`SerialPort`] is fully abstract; the
/// script either has bytes or it does not, so there is no clock to consult.
struct MockPort {
    written: Vec<u8>,
    to_read: Vec<u8>,
    read_pos: usize,
}

impl MockPort {
    fn new(to_read: Vec<u8>) -> Self {
        Self {
            written: Vec::new(),
            to_read,
            read_pos: 0,
        }
    }
}

impl SerialPort for MockPort {
    type Error = std::io::Error;
    type Instant = ();

    fn baud_rate(&self) -> Result<u32, Self::Error> {
        Ok(4_000_000)
    }

    fn set_baud_rate(&mut self, _baud_rate: u32) -> Result<(), Self::Error> {
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

/// Build a well-formed frame by hand.
///
/// The round-trip tests below drive the client instead, but the client only
/// ever addresses one register at a time and never emits a malformed frame, so
/// the multi-register and corruption cases have to be written out.
fn frame(id: u8, instruction: u8, parameters: &[u8]) -> Vec<u8> {
    let mut wire = vec![0xFF, 0xFF, id, (parameters.len() + 2) as u8, instruction];
    wire.extend_from_slice(parameters);
    let sum = wire[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    wire.push(255u8.wrapping_sub(sum));
    wire
}

fn client() -> Bus<MockPort, Vec<u8>> {
    Bus::<MockPort, Vec<u8>>::with_buffers(MockPort::new(Vec::new()), vec![0u8; 256], vec![0u8; 256]).unwrap()
}

fn device(wire: Vec<u8>) -> Device<MockPort, Vec<u8>> {
    Device::<MockPort, Vec<u8>>::with_buffers(MockPort::new(wire), vec![0u8; 256], vec![0u8; 256]).unwrap()
}

/// Run a client instruction, then decode the bytes it produced as a device would.
///
/// The client's reply read is expected to fail (the mock has no script), which is
/// fine: the request bytes are already on the wire by then.
fn roundtrip(build: impl FnOnce(&mut Bus<MockPort, Vec<u8>>), check: impl FnOnce(&[u8], Packet<&[u8]>)) {
    let mut bus = client();
    build(&mut bus);
    let wire = bus.serial_port().written.clone();
    assert!(!wire.is_empty(), "client wrote nothing");

    let mut device = device(wire.clone());
    let packet = device.read(Duration::from_millis(1)).expect("device failed to decode");
    check(&wire, packet);
}

#[test]
fn ping_roundtrip() {
    roundtrip(
        |bus| {
            let _ = bus.ping(7);
        },
        |wire, packet| {
            // Fixed vector: the firmware's own hand-built ping is
            // `FF FF | id | LEN=2 | INST=1 | ~sum(id..inst)`. LEN counts the
            // instruction and the checksum, which is the classic off-by-one.
            assert_eq!(wire, &[0xFF, 0xFF, 0x07, 0x02, 0x01, !(0x07u8 + 0x02 + 0x01)]);
            assert_eq!(packet.id, 7);
            assert!(matches!(packet.kind, PacketKind::Ping));
        },
    );
}

#[test]
fn read_status_roundtrip() {
    roundtrip(
        |bus| {
            let _ = bus.read_status(3, StatusRegister::PresentPos);
        },
        |_, packet| {
            assert_eq!(packet.id, 3);
            let PacketKind::ReadStat { registers } = packet.kind else {
                panic!("expected ReadStat, got {:?}", packet.kind)
            };
            assert_eq!(registers, &[StatusRegister::PresentPos as u8]);
        },
    );
}

#[test]
fn read_config_roundtrip() {
    roundtrip(
        |bus| {
            let _ = bus.read_config(3, ConfigRegister::LimitPosMax);
        },
        |_, packet| {
            let PacketKind::ReadCfg { registers } = packet.kind else {
                panic!("expected ReadCfg, got {:?}", packet.kind)
            };
            assert_eq!(registers, &[ConfigRegister::LimitPosMax as u8]);
        },
    );
}

#[test]
fn read_config_raw_index_roundtrip() {
    // `0x1D` is reserved: in range for the table, but with no `ConfigRegister`
    // variant. A bare `u8` is how a client reaches it.
    roundtrip(
        |bus| {
            let _ = bus.read_config(3, 0x1D);
        },
        |_, packet| {
            let PacketKind::ReadCfg { registers } = packet.kind else {
                panic!("expected ReadCfg, got {:?}", packet.kind)
            };
            assert_eq!(registers, &[0x1D]);
        },
    );
}

#[test]
fn write_status_roundtrip() {
    roundtrip(
        |bus| {
            let _ = bus.write_status(9, StatusRegister::GoalPos, &1.5f32.to_le_bytes());
        },
        |_, packet| {
            assert_eq!(packet.id, 9);
            let PacketKind::WriteStat { parameters } = packet.kind else {
                panic!("expected WriteStat, got {:?}", packet.kind)
            };
            let writes: Vec<_> = register_writes(parameters).collect();
            assert_eq!(writes.len(), 1);
            assert_eq!(writes[0].register, StatusRegister::GoalPos as u8);
            assert_eq!(writes[0].f32(), 1.5);
        },
    );
}

#[test]
fn write_config_roundtrip() {
    roundtrip(
        |bus| {
            let _ = bus.write_config(9, ConfigRegister::Id, &12u32.to_le_bytes());
        },
        |_, packet| {
            let PacketKind::WriteCfg { parameters } = packet.kind else {
                panic!("expected WriteCfg, got {:?}", packet.kind)
            };
            let writes: Vec<_> = register_writes(parameters).collect();
            assert_eq!(writes[0].register, ConfigRegister::Id as u8);
            assert_eq!(writes[0].u32(), 12);
        },
    );
}

#[test]
fn write_status_raw_index_roundtrip() {
    // `0x0E` has no `StatusRegister` variant, but the firmware answers it.
    roundtrip(
        |bus| {
            let _ = bus.write_status(9, 0x0E, &2.5f32.to_le_bytes());
        },
        |_, packet| {
            let PacketKind::WriteStat { parameters } = packet.kind else {
                panic!("expected WriteStat, got {:?}", packet.kind)
            };
            let writes: Vec<_> = register_writes(parameters).collect();
            assert_eq!(writes[0].register, 0x0E);
            assert_eq!(writes[0].f32(), 2.5);
        },
    );
}

#[test]
fn save_config_roundtrip() {
    roundtrip(
        |bus| {
            let _ = bus.save_config(4);
        },
        |_, packet| {
            assert_eq!(packet.id, 4);
            assert!(matches!(packet.kind, PacketKind::SaveCfg));
        },
    );
}

#[test]
fn set_absolute_position_roundtrip() {
    roundtrip(
        |bus| {
            let _ = bus.set_absolute_position(2, 1.25, 0.01);
        },
        |_, packet| {
            let PacketKind::SetAbsPos { theta, tolerance } = packet.kind else {
                panic!("expected SetAbsPos, got {:?}", packet.kind)
            };
            assert_eq!(theta, 1.25);
            assert_eq!(tolerance, 0.01);
        },
    );
}

/// The eight-actuator case the hand controller has to serve.
#[test]
fn bulk_read_eight_motors_roundtrip() {
    const IDS: [u8; 8] = [10, 11, 12, 13, 20, 21, 22, 23];

    roundtrip(
        |bus| {
            let _ = bus.bulk_read(&IDS, &[StatusRegister::PresentPos, StatusRegister::PresentIq], |_| {});
        },
        |_, packet| {
            // Broadcast: every motor parses it, and each decides for itself
            // whether it is listed.
            assert_eq!(packet.id, 0xFE);
            let PacketKind::BulkComm { bulk } = packet.kind else {
                panic!("expected BulkComm, got {:?}", packet.kind)
            };

            assert_eq!(bulk.motor_count(), 8);
            assert_eq!(
                bulk.read_registers(),
                &[StatusRegister::PresentPos as u8, StatusRegister::PresentIq as u8]
            );
            assert!(bulk.write_registers().is_empty());
            assert_eq!(bulk.reply_len(), 8, "two registers, four bytes each");

            // Positions must match request order: that order is the reply order.
            let entries: Vec<_> = bulk.entries().collect();
            assert_eq!(entries.len(), 8);
            for (i, entry) in entries.iter().enumerate() {
                assert_eq!(entry.position, i);
                assert_eq!(entry.motor_id, IDS[i]);
                assert!(entry.write_data.is_empty(), "read-only bulk carries no write data");
            }

            // The sequencing rule: position 0 leads, everyone else follows the
            // previous ID in the list.
            assert_eq!(bulk.predecessor(0), None);
            for i in 1..8 {
                assert_eq!(bulk.predecessor(i), Some(IDS[i - 1]));
            }

            assert_eq!(bulk.find(22).map(|e| e.position), Some(6));
            assert_eq!(bulk.find(99), None, "a motor not listed must not match");
        },
    );
}

#[test]
fn bulk_read_write_carries_per_motor_data() {
    let devices = [
        BulkWriteData {
            motor_id: 10,
            data: 0.25f32.to_le_bytes(),
        },
        BulkWriteData {
            motor_id: 11,
            data: 0.75f32.to_le_bytes(),
        },
    ];

    roundtrip(
        |bus| {
            let _ = bus.bulk_read_write(
                devices,
                &[StatusRegister::PresentPos],
                &[StatusRegister::GoalPos],
                |_| {},
            );
        },
        |_, packet| {
            let PacketKind::BulkComm { bulk } = packet.kind else {
                panic!("expected BulkComm, got {:?}", packet.kind)
            };
            assert_eq!(bulk.write_registers(), &[StatusRegister::GoalPos as u8]);

            // The entry carries the register list its data belongs to, so the
            // pairing cannot be got wrong by passing the wrong slice back in.
            let goals: Vec<f32> = bulk
                .entries()
                .map(|entry| {
                    assert_eq!(entry.write_registers(), &[StatusRegister::GoalPos as u8]);
                    entry.writes().next().expect("one write per motor").f32()
                })
                .collect();
            assert_eq!(goals, vec![0.25, 0.75]);
        },
    );
}

/// A reply must never be mistaken for an instruction.
///
/// This is the property bulk ordering rests on: byte 4 is the instruction in a
/// request and the error byte in a reply, and only bit 7 separates them.
#[test]
fn status_packet_is_not_an_instruction() {
    // `FF FF | id | len | err | data | csum` with the status flag set. The error
    // byte here is 0x82, whose low bits collide with `ReadStat` (0x02).
    let mut wire = vec![0xFF, 0xFF, 0x0B, 0x06, 0x82, 0x00, 0x00, 0x80, 0x3F];
    let sum = wire[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    wire.push(255u8.wrapping_sub(sum));

    let mut device = device(wire);
    let packet = device.read(Duration::from_millis(1)).unwrap();

    assert_eq!(packet.id, 0x0B);
    let PacketKind::Status { error, parameters } = packet.kind else {
        panic!("expected Status, got {:?}", packet.kind)
    };
    assert_eq!(error, 0x82);
    assert_eq!(f32::from_le_bytes(parameters.try_into().unwrap()), 1.0);

    // And it is addressed to nobody: the ID names the sender.
    assert!(
        !packet.addresses(0x0B),
        "a reply must not look like a request to its own sender"
    );
}

#[test]
fn addressing_accepts_own_id_and_broadcast() {
    roundtrip(
        |bus| {
            let _ = bus.ping(10);
        },
        |_, packet| {
            assert!(packet.addresses(10));
            assert!(!packet.addresses(11));
        },
    );

    roundtrip(
        |bus| {
            let _ = bus.bulk_read(&[10], &[StatusRegister::PresentPos], |_| {});
        },
        |_, packet| {
            // Broadcast reaches every device, whatever ID it holds.
            assert!(packet.addresses(10));
            assert!(packet.addresses(200));
        },
    );
}

/// Bulk is the only instruction the broadcast ID carries.
///
/// A device that answered a broadcast of any other kind would transmit at the
/// same moment as every other motor on the bus, and the replies would collide.
#[test]
fn broadcast_addresses_nobody_except_for_bulk() {
    for instruction in [
        Instruction::Ping as u8,
        Instruction::SaveCfg as u8,
        Instruction::ReadStat as u8,
    ] {
        let parameters: &[u8] = match instruction {
            x if x == Instruction::ReadStat as u8 => &[StatusRegister::PresentPos as u8],
            _ => &[],
        };
        let mut device = device(frame(0xFE, instruction, parameters));
        let packet = device.read(Duration::from_millis(1)).expect("failed to decode");

        assert!(
            !packet.addresses(3) && !packet.addresses(200),
            "instruction {instruction:#04X} must not be served on the broadcast ID"
        );
    }

    // Addressed to a specific motor, the same instructions are served normally.
    let mut device = device(frame(3, Instruction::Ping as u8, &[]));
    let packet = device.read(Duration::from_millis(1)).unwrap();
    assert!(packet.addresses(3));
    assert!(!packet.addresses(200));
}

/// The whole point of the device half: answer a read with data the request chose.
///
/// The reply borrows nothing from the request only because [`Packet::copy_into`]
/// detaches it first. Without that step this does not compile, which is exactly
/// the failure this test exists to pin.
#[test]
fn a_reply_can_be_built_from_the_request_it_answers() {
    const REGISTERS: [u8; 3] = [
        StatusRegister::PresentPos as u8,
        StatusRegister::PresentIq as u8,
        StatusRegister::WindingTemp as u8,
    ];

    let mut dev = device(frame(7, Instruction::ReadStat as u8, &REGISTERS));

    let mut scratch = [0u8; ConfigRegister::COUNT];
    let packet = dev
        .read(Duration::from_millis(1))
        .expect("device failed to decode")
        .copy_into(&mut scratch)
        .expect("scratch too small");

    assert!(packet.addresses(7));
    let PacketKind::ReadStat { registers } = packet.kind else {
        panic!("expected ReadStat, got {:?}", packet.kind)
    };
    assert_eq!(registers, &REGISTERS);

    // `dev` is free again here: the request no longer borrows it.
    dev.write_status(7, ErrorFlags::empty(), registers.len() * 4, |out| {
        for (slot, register) in out.chunks_exact_mut(4).zip(registers) {
            slot.copy_from_slice(&(f32::from(*register) * 2.0).to_le_bytes());
        }
        Ok(())
    })
    .expect("device rejected its own reply");

    // And the client half decodes what came back, register for register.
    let reply = dev.serial_port().written.clone();
    let mut bus = Bus::<MockPort, Vec<u8>>::with_buffers(MockPort::new(reply), vec![0u8; 256], vec![0u8; 256]).unwrap();
    let response = bus.ping(7).expect("client rejected the device's reply");
    for (index, register) in REGISTERS.iter().enumerate() {
        assert_eq!(response.f32(index), Some(f32::from(*register) * 2.0));
    }
}

/// The allocating counterpart to [`Packet::copy_into`].
#[test]
fn into_owned_detaches_a_packet_from_the_read_buffer() {
    let mut dev = device(frame(9, Instruction::WriteStat as u8, &[0x05, 0x00, 0x00, 0xC0, 0x3F]));
    let packet = dev.read(Duration::from_millis(1)).unwrap().into_owned();

    let PacketKind::WriteStat { parameters } = &packet.kind else {
        panic!("expected WriteStat, got {:?}", packet.kind)
    };
    let writes: Vec<_> = register_writes(parameters).collect();
    assert_eq!(writes[0].register, StatusRegister::GoalPos as u8);
    assert_eq!(writes[0].f32(), 1.5);

    // Reading again overwrites the device's buffer; the copy is unaffected.
    let _ = dev.read(Duration::from_millis(1));
    assert_eq!(register_writes(parameters).next().unwrap().f32(), 1.5);
}

/// A frame whose length byte was corrupted must cost only that frame.
///
/// The length used to resynchronise came from the frame that just failed its
/// checksum, so trusting it let one line glitch swallow the next instruction as
/// well. On a device, which reads the bus continuously, that is the difference
/// between one dropped packet and two.
#[test]
fn a_corrupted_length_does_not_swallow_the_following_frame() {
    let mut wire = frame(1, Instruction::Ping as u8, &[]);
    wire[3] = 0x08; // LEN corrupted after the checksum was computed over 0x02.
    wire.extend_from_slice(&frame(2, Instruction::Ping as u8, &[]));

    let mut dev = device(wire);
    assert!(
        matches!(dev.read(Duration::from_millis(1)), Err(ReadError::InvalidMessage(_))),
        "the corrupted frame should fail its checksum"
    );

    let packet = dev
        .read(Duration::from_millis(1))
        .expect("the following frame was consumed along with the corrupted one");
    assert_eq!(packet.id, 2);
    assert!(matches!(packet.kind, PacketKind::Ping));
}

/// A frame too large for the read buffer is dropped, not retried forever.
#[test]
fn an_oversize_frame_does_not_wedge_the_reader() {
    let mut wire = frame(1, Instruction::WriteStat as u8, &[0u8; 40]);
    wire.extend_from_slice(&frame(2, Instruction::Ping as u8, &[]));

    let mut dev = Device::<MockPort, Vec<u8>>::with_buffers(MockPort::new(wire), vec![0u8; 32], vec![0u8; 32]).unwrap();

    assert!(
        matches!(dev.read(Duration::from_millis(1)), Err(ReadError::BufferFull(_))),
        "a 46 byte frame should not fit a 32 byte buffer"
    );

    let packet = dev
        .read(Duration::from_millis(1))
        .expect("reader wedged on the frame it could not hold");
    assert_eq!(packet.id, 2);
}

/// `LEN` is one byte, so an oversize parameter block cannot be framed at all.
///
/// Truncating it would put a frame on the wire whose length byte disagrees with
/// its contents: every receiver mis-frames it, fails the checksum, and then
/// desynchronises on the remainder.
#[test]
fn a_parameter_block_too_large_to_describe_is_refused() {
    let mut dev =
        Device::<MockPort, Vec<u8>>::with_buffers(MockPort::new(Vec::new()), vec![0u8; 512], vec![0u8; 512]).unwrap();

    // A whole number of registers, and it fits the buffer. It still cannot be
    // described: 256 + 2 does not fit in a byte.
    let result = dev.write_status_bytes(9, ErrorFlags::empty(), &[0xAA; 256]);
    assert!(
        matches!(result, Err(WriteError::BufferTooSmall(_))),
        "expected a refusal, got {result:?}"
    );
    assert!(
        dev.serial_port().written.is_empty(),
        "nothing may reach the wire for a frame that cannot be described"
    );

    // The largest describable block still goes out, with an honest length byte.
    let largest = MAX_PARAMETER_COUNT - MAX_PARAMETER_COUNT % 4;
    dev.write_status_bytes(9, ErrorFlags::empty(), &vec![0xAA; largest])
        .expect("the largest legal block was refused");
    let wire = dev.serial_port().written.clone();
    assert_eq!(wire[3] as usize, largest + 2, "length byte");
    assert_eq!(wire.len(), largest + 6);
}

/// Bytes captured before a baud rate change were sampled at the old rate.
#[test]
fn changing_the_baud_rate_drops_what_was_buffered() {
    let mut wire = frame(1, Instruction::Ping as u8, &[]);
    wire.extend_from_slice(&frame(2, Instruction::Ping as u8, &[]));

    // One read pulls both frames off the port, so the second is buffered here.
    let mut kept = device(wire.clone());
    assert_eq!(kept.read(Duration::from_millis(1)).unwrap().id, 1);
    assert_eq!(
        kept.read(Duration::from_millis(1)).unwrap().id,
        2,
        "the second frame should be buffered"
    );

    let mut switched = device(wire);
    assert_eq!(switched.read(Duration::from_millis(1)).unwrap().id, 1);
    switched.set_baud_rate(115_200).unwrap();
    assert!(
        matches!(switched.read(Duration::from_millis(1)), Err(ReadError::Io(_))),
        "buffered bytes must not survive the switch"
    );
}

/// `ExactSizeIterator` requires `size_hint` to be exact wherever it is implemented.
#[test]
fn exact_size_iterators_agree_with_their_size_hint() {
    let parameters = [0x05, 0x00, 0x00, 0x80, 0x3F, 0x06, 0x00, 0x00, 0x00, 0x00];
    let writes = register_writes(&parameters);
    assert_eq!(writes.len(), 2);
    assert_eq!(writes.size_hint(), (2, Some(2)));

    // A bulk block for three motors, one read register and one write register.
    let mut block = vec![
        0x03,
        0x11,
        StatusRegister::PresentPos as u8,
        StatusRegister::GoalPos as u8,
    ];
    for id in [10u8, 11, 12] {
        block.push(id);
        block.extend_from_slice(&1.0f32.to_le_bytes());
    }
    let bulk = BulkComm::parse(block.as_slice()).expect("bulk layout rejected");

    let mut entries = bulk.entries();
    assert_eq!(entries.size_hint(), (3, Some(3)));
    entries.next();
    assert_eq!(entries.size_hint(), (2, Some(2)));
}

/// The device must set bit 7 on every reply, whatever the caller passes.
#[test]
fn written_status_always_sets_the_status_flag() {
    for error in [ErrorFlags::empty(), ErrorFlags::OVERHEAT, ErrorFlags::JOINT_LIMIT] {
        let mut dev = device(Vec::new());
        dev.write_status_bytes(10, error, &1.0f32.to_le_bytes()).unwrap();
        let wire = dev.serial_port().written.clone();

        assert_eq!(&wire[..2], &[0xFF, 0xFF]);
        assert_eq!(wire[2], 10);
        assert_eq!(wire[3] as usize, 2 + 4, "len counts the error byte and the checksum");
        assert_ne!(wire[4] & 0x80, 0, "status flag missing for {error:?}");
        assert_eq!(wire[4] & 0x7F, error.bits(), "error bits mangled");

        // And the client half must accept what we produced.
        let sum = wire[2..wire.len() - 1].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        assert_eq!(*wire.last().unwrap(), 255u8.wrapping_sub(sum), "checksum");
    }
}

/// A device reply has to decode as a reply on the client side, or the whole
/// emulation is invisible. This is the other half of the round trip.
#[test]
fn client_parses_a_device_reply() {
    let mut dev = device(Vec::new());
    dev.write_status_bytes(12, ErrorFlags::OVERHEAT, &2.5f32.to_le_bytes())
        .unwrap();
    let wire = dev.serial_port().written.clone();

    let mut bus = Bus::<MockPort, Vec<u8>>::with_buffers(MockPort::new(wire), vec![0u8; 128], vec![0u8; 128]).unwrap();
    let response = bus.ping(12).expect("client rejected the device's reply");

    assert_eq!(response.motor_id, 12);
    // Bit 7 is not a flag, so it must not survive into the decoded warnings.
    assert_eq!(response.warning, ErrorFlags::OVERHEAT);
    assert_eq!(response.f32(0), Some(2.5));
}

#[test]
fn rejects_a_bulk_packet_whose_length_disagrees_with_its_counts() {
    // motor_count = 2, one read register, no writes: the block should be
    // 2 + 1 + 0 + 2 * 1 = 5 parameter bytes. Give it 4.
    let mut wire = vec![0xFF, 0xFF, 0xFE, 0x06, 0x12, 0x02, 0x10, 0x09, 0x0A];
    let sum = wire[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    wire.push(255u8.wrapping_sub(sum));

    let mut device = device(wire);
    let result = device.read(Duration::from_millis(1));
    assert!(
        matches!(result, Err(ReadError::InvalidMessage(_))),
        "expected a rejected bulk layout, got {result:?}"
    );
}

#[test]
fn rejects_a_ping_carrying_parameters() {
    let mut wire = vec![0xFF, 0xFF, 0x01, 0x03, 0x01, 0x00];
    let sum = wire[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    wire.push(255u8.wrapping_sub(sum));

    let mut device = device(wire);
    assert!(matches!(
        device.read(Duration::from_millis(1)),
        Err(ReadError::InvalidMessage(_))
    ));
}

#[test]
fn unknown_instruction_is_reported_not_guessed() {
    let mut wire = vec![0xFF, 0xFF, 0x01, 0x03, 0x7F, 0xAB];
    let sum = wire[2..].iter().fold(0u8, |a, b| a.wrapping_add(*b));
    wire.push(255u8.wrapping_sub(sum));

    let mut device = device(wire);
    let packet = device.read(Duration::from_millis(1)).unwrap();
    let PacketKind::Unknown {
        instruction,
        parameters,
    } = packet.kind
    else {
        panic!("expected Unknown, got {:?}", packet.kind)
    };
    assert_eq!(instruction, 0x7F);
    assert_eq!(parameters, &[0xAB]);
}
