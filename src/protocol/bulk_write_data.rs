/// One motor's entry in a bulk write.
///
/// Pairs a `motor_id` with its encoded write bytes so the two travel together, rather than as
/// two parallel slices that must be kept in the same order. The `data` holds the concatenated
/// write registers for this motor: `write_registers.len() * 4` little-endian bytes (4 bytes per
/// register). See [`crate::Bus::bulk_write`].
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BulkWriteData<T> {
    /// The motor to write to.
    pub motor_id: u8,

    /// The encoded write bytes for this motor.
    pub data: T,
}

impl<T> AsRef<BulkWriteData<T>> for BulkWriteData<T> {
    fn as_ref(&self) -> &BulkWriteData<T> {
        self
    }
}
