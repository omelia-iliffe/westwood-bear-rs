/// One motor's entry in a bulk write.
///
/// Pairs a `motor_id` with its encoded write bytes so the two travel together, rather than as
/// two parallel slices that must be kept in the same order. The `data` holds the concatenated
/// write registers for this motor: `write_registers.len() * 4` little-endian bytes (4 bytes per
/// register). See [`crate::Bus::bulk_write`].
///
/// For the common case of a single write register, use [`BulkWriteData::from_f32`] or
/// [`BulkWriteData::from_u32`] to build an entry without hand-encoding the little-endian bytes.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BulkWriteData<T> {
    /// The motor to write to.
    pub motor_id: u8,

    /// The encoded write bytes for this motor.
    pub data: T,
}

impl BulkWriteData<[u8; 4]> {
    /// Build an entry for a single `f32` write register, encoding `value` as little-endian bytes.
    pub fn from_f32(motor_id: u8, value: f32) -> Self {
        Self {
            motor_id,
            data: value.to_le_bytes(),
        }
    }

    /// Build an entry for a single `u32` write register, encoding `value` as little-endian bytes.
    pub fn from_u32(motor_id: u8, value: u32) -> Self {
        Self {
            motor_id,
            data: value.to_le_bytes(),
        }
    }
}

impl<T> AsRef<BulkWriteData<T>> for BulkWriteData<T> {
    fn as_ref(&self) -> &BulkWriteData<T> {
        self
    }
}
