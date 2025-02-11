

pub fn calculate_checksum(data: &[u8]) -> u8 {
    let sum = data.iter().fold(0_u8, |acc, d| {
        acc.wrapping_add(*d)
    });
    255 - sum
}