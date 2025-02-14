use std::thread;
use std::time::Duration;
use ww_bear::Bus;

const ID: u8 = 1;
use ww_bear::registers;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut bear = Bus::open("/dev/ttyUSB0", 8_000_000)?;

    match bear.ping(ID) {
        Ok(_) => {
            log::info!("ping id {ID} success")
        },
        Err(e) => {
            log::info!("failed to ping {ID}");
            Err(e)?;
        },
    };

    bear.write::<registers::PGainIq>(ID, 0.02)?;
    bear.write::<registers::IGainIq>(ID, 0.02)?;
    bear.write::<registers::DGainIq>(ID, 0.0)?;

    bear.write::<registers::PGainId>(ID, 0.02)?;
    bear.write::<registers::IGainId>(ID, 0.02)?;
    bear.write::<registers::DGainId>(ID, 0.0)?;

    bear.write::<registers::PGainPos>(ID, 5.0)?;
    bear.write::<registers::IGainPos>(ID, 0.0)?;
    bear.write::<registers::DGainPos>(ID, 0.02)?;

    bear.write::<registers::Mode>(ID, 2)?;

    bear.write::<registers::LimitIMax>(ID, 1.5)?;

    let start_pos = bear.read::<registers::PresentPos>(ID)?.data;

    bear.write::<registers::GoalPos>(ID, start_pos)?;
    bear.write::<registers::TorqueEnable>(ID, 1)?;

    for _ in 1..10 {
        bear.write::<registers::GoalPos>(ID, start_pos - 0.5)?;
        thread::sleep(Duration::from_millis(1500));

        bear.write::<registers::GoalPos>(ID, start_pos + 0.5)?;
        thread::sleep(Duration::from_millis(1500));
    }
    Ok(())
}
