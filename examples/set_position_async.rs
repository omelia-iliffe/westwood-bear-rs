use std::thread;
use std::time::Duration;
use ww_bear::asynchronous::Bus;

const ID: u8 = 1;
use ww_bear::registers::{config, status};

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut bear = Bus::open("/dev/ttyUSB0", 8_000_000)?;

    match bear.ping(ID).await {
        Ok(_) => {
            log::info!("ping id {ID} success")
        },
        Err(e) => {
            log::info!("failed to ping {ID}");
            Err(e)?;
        },
    };

    bear.write::<config::PGainIq>(ID, 0.02).await?;
    bear.write::<config::IGainIq>(ID, 0.02).await?;
    bear.write::<config::DGainIq>(ID, 0.0).await?;

    bear.write::<config::PGainId>(ID, 0.02).await?;
    bear.write::<config::IGainId>(ID, 0.02).await?;
    bear.write::<config::DGainId>(ID, 0.0).await?;

    bear.write::<config::PGainPos>(ID, 5.0).await?;
    bear.write::<config::IGainPos>(ID, 0.0).await?;
    bear.write::<config::DGainPos>(ID, 0.02).await?;

    bear.write::<config::Mode>(ID, 2).await?;

    bear.write::<config::LimitIMax>(ID, 1.5).await?;

    let min_pos = bear.read::<config::LimitPosMin>(ID).await?.data;

    let max_pos = bear.read::<config::LimitPosMax>(ID).await?.data;

    bear.write::<status::GoalPos>(ID, min_pos).await?;

    bear.write::<status::TorqueEnable>(ID, 1).await?;

    for _ in 1..10 {
        bear.write::<status::GoalPos>(ID, min_pos + 0.1).await?;
        thread::sleep(Duration::from_millis(1500));

        bear.write::<status::GoalPos>(ID, max_pos - 0.1).await?;
        thread::sleep(Duration::from_millis(1500));
    }
    Ok(())
}
