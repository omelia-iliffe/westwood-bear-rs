#[repr(u8)]
pub enum Instruction {
    Ping = 0x01,
    ReadStat = 0x02,
    WriteStat = 0x03,
    ReadCfg = 0x04,
    WriteCfg = 0x05,
    SaveCfg = 0x06,
    BulkComm = 0x12,
}

/// Special packet IDs.
pub mod packet_id {
    /// The broadcast address.
    pub const BROADCAST: u8 = 0xFE;
}