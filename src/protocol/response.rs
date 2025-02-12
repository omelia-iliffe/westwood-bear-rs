/// A response from a motor.
///
/// Note that the `Eq` and `PartialEq` compare all fields of the struct,
/// including the `motor_id` and `alert`.
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct Response<T> {
    /// The motor that sent the response.
    pub motor_id: u8,

    /// The error byte from the motor
    pub error: u8,

    /// The data from the motor.
    pub data: T,
}
