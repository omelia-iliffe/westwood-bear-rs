use crate::protocol::motor_error::ErrorFlags;

/// A response from a motor.
///
/// Note that the `Eq` and `PartialEq` compare all fields of the struct,
/// including the `motor_id` and `alert`.
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct Response<T> {
    /// The motor that sent the response.
    pub motor_id: u8,

    /// The motor returns an [`ErrorFlags`] byte with every response. If the flag contains any errors an Err is returned, or any warnings this field is Some(_)
    pub warning: Option<ErrorFlags>,

    /// The data from the motor.
    pub data: T,
}
