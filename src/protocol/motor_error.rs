use bitflags::bitflags;
use core::fmt::{Display, Formatter};

bitflags! {
    #[derive(Debug, Clone, Copy, Eq, PartialEq)]
    pub struct ErrorFlags: u8 {
        /// A corrupted data packet was received. This warning resets automatically
        /// and is only associated with corresponding round of communication
        const COMMUNICATION = 0b00000001;
        /// The temperature of at least one component among IC, powerstage and winding
        /// in this BEAR has exceeded the value written to temperature limit low. This warning resets
        /// automatically when the temperature limit low value is higher than the highest temperature
        /// measured in this BEAR module
        const OVERHEAT = 0b00000010;
        /// Error whilst reading the absolute position
        const ABSOLUTE_POSITION = 0b00000100;
        /// When in mode 0(torque mode), motor enabled and watchdog
        /// timer configured, watchdog timeout triggers this error.
        /// External ESTOP signals also triggers this error. Including physical signal and writing 0x03 to
        /// torque enable.
        const WATCHDOG_ESTOP = 0b00001000;
        /// Joint limit exceeded
        const JOINT_LIMIT = 0b00010000;
        /// Input Voltage out of range or MOSFET driver fault
        const HARDWARE = 0b00100000;
        /// Corrupted save file in flash, calibration needed
        const INITIALIZATION = 0b01000000;
    }
}

pub(crate) const WARNING_FLAGS: ErrorFlags = ErrorFlags::OVERHEAT.union(ErrorFlags::COMMUNICATION);
pub(crate) const ERROR_FLAGS: ErrorFlags = WARNING_FLAGS.complement();

impl Display for ErrorFlags {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        bitflags::parser::to_writer(self, f)
    }
}

#[cfg(test)]
mod tests {
    use crate::protocol::motor_error::ErrorFlags;

    #[test]
    fn error_flags() {
        let value = 128_u8;
        let flag = ErrorFlags::from_bits(value);
        assert!(flag.is_none())
    }
}
