use crate::protocol::motor_error::ErrorFlags;

/// A response from a motor.
///
/// Note that the `Eq` and `PartialEq` compare all fields of the struct,
/// including the `motor_id` and `alert`.
#[derive(Debug, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Response<T> {
    /// The motor that sent the response.
    pub motor_id: u8,

    /// The motor returns an [`ErrorFlags`] byte with every response. If the flag contains any errors an Err is returned, or any warnings this field is Some(_)
    pub warning: ErrorFlags,

    /// The data from the motor.
    pub data: T,
}

/// Bytes per register value on the wire (4 little-endian bytes).
const REGISTER_BYTES: usize = 4;

impl<T: AsRef<[u8]>> Response<T> {
    /// Decode the `index`th 4-byte register of the reply data as an `f32`.
    ///
    /// Available on any response whose `data` is raw bytes: a single [`Bus::read_status`] /
    /// [`Bus::read_config`] reply (use `index` 0), or a bulk reply whose `data` concatenates
    /// several registers (`index` is the register's position in the `read_registers` slice).
    /// The counterpart to [`crate::BulkWriteData::from_f32`] for the read path. Returns `None` if
    /// the data is too short to hold that register.
    ///
    /// [`Bus::read_status`]: crate::Bus::read_status
    /// [`Bus::read_config`]: crate::Bus::read_config
    pub fn f32(&self, index: usize) -> Option<f32> {
        self.register_bytes(index).map(f32::from_le_bytes)
    }

    /// Decode the `index`th 4-byte register of the reply data as a `u32`.
    ///
    /// Available on any response whose `data` is raw bytes: a single [`Bus::read_status`] /
    /// [`Bus::read_config`] reply (use `index` 0), or a bulk reply whose `data` concatenates
    /// several registers (`index` is the register's position in the `read_registers` slice).
    /// The counterpart to [`crate::BulkWriteData::from_u32`] for the read path. Returns `None` if
    /// the data is too short to hold that register.
    ///
    /// [`Bus::read_status`]: crate::Bus::read_status
    /// [`Bus::read_config`]: crate::Bus::read_config
    pub fn u32(&self, index: usize) -> Option<u32> {
        self.register_bytes(index).map(u32::from_le_bytes)
    }

    fn register_bytes(&self, index: usize) -> Option<[u8; REGISTER_BYTES]> {
        let start = index * REGISTER_BYTES;
        self.data.as_ref().get(start..start + REGISTER_BYTES)?.try_into().ok()
    }
}
