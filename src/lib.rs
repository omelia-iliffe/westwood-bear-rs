#![warn(missing_docs)]
#![warn(missing_debug_implementations)]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "serial2")]
/// Public re-export of the serial2 crate.
pub use serial2;
mod bus;
pub use bus::Bus;
mod checksum;
pub mod error;
mod instructions;
mod protocol;
pub use protocol::*;
mod serial_port;

pub use serial_port::SerialPort;

#[cfg(test)]
mod tests {
    use crate::bus::Bus;
    use crate::protocol::{Instruction, StatusRegister};
    use serial2::SerialPort;
    use test_log::test;

    fn setup_bus() -> Bus {
        let port = SerialPort::open("/dev/ttyUSB0", 8000000).unwrap();
        Bus::with_buffers(port, vec![0; 100], vec![0; 100]).unwrap()
    }

    #[test]
    fn make_packet() {
        let mut buffer = vec![0; 1000];
        let pos = 10.0_f32;
        let r = Bus::<SerialPort>::make_packet(&mut buffer, 1, Instruction::WriteStat as u8, 5, |buffer| {
            buffer[0] = 5;

            buffer[1..].copy_from_slice(&pos.to_le_bytes());
            Ok(())
        })
        .unwrap();

        let r = &buffer[..r];

        assert_eq!(r, &[255, 255, 1, 7, 3, 5, 0, 0, 32, 65, 142])
    }

    #[test]
    fn read_raw() {
        let mut bus = setup_bus();
        let r = bus
            .read_raw(1, Instruction::ReadStat as u8, StatusRegister::TorqueEnable as u8, 1)
            .unwrap();
        dbg!(r);
    }

    #[test]
    fn read() {
        let mut bus = setup_bus();
        let r = bus.read::<crate::registers::Id>(1).unwrap();
        dbg!(r);
    }

    #[test]
    fn write() {
        let mut bus = setup_bus();
        bus.write::<crate::registers::TorqueEnable>(1, 1).unwrap()
    }
}
