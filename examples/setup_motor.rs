use clap::Parser;
use std::io::{self, Write};
use ww_bear::Bus;

#[derive(Parser)]
struct Args {
    /// Serial port (e.g. /dev/ttyUSB0)
    port: String,
    /// Motor ID to configure
    #[arg(short, long)]
    id: u8,
    /// Baud rate
    #[arg(short, long, default_value_t = 8_000_000)]
    baud: u32,
}

struct MotorProfile {
    name: &'static str,
    p_gain: f32,
    i_gain: f32,
    d_gain: f32,
}

const MOTOR_PROFILES: &[MotorProfile] = &[
    MotorProfile { name: "Koala V2",              p_gain: 0.277, i_gain: 0.061, d_gain: 0.0 },
    MotorProfile { name: "Koala Muscle Build V1", p_gain: 0.358, i_gain: 0.045, d_gain: 0.0 },
    MotorProfile { name: "Panda V2",              p_gain: 0.099, i_gain: 0.039, d_gain: 0.0 },
    MotorProfile { name: "Panda Plus V2",         p_gain: 0.184, i_gain: 0.065, d_gain: 0.0 },
    MotorProfile { name: "Kodiak V1",             p_gain: 0.250, i_gain: 0.017, d_gain: 0.0 },
];

fn prompt(msg: &str) -> String {
    print!("{msg}");
    io::stdout().flush().unwrap();
    let mut line = String::new();
    io::stdin().read_line(&mut line).unwrap();
    line.trim().to_string()
}

fn prompt_f32(label: &str, default: f32) -> f32 {
    loop {
        let s = prompt(&format!("  {label} [{default:.3}]: "));
        if s.is_empty() {
            return default;
        }
        match s.parse() {
            Ok(v) => return v,
            Err(_) => println!("  Please enter a valid number."),
        }
    }
}

fn prompt_u8(label: &str, default: u8) -> u8 {
    loop {
        let s = prompt(&format!("  {label} [{default}]: "));
        if s.is_empty() {
            return default;
        }
        match s.parse::<u8>() {
            Ok(v) if v >= 1 => return v,
            _ => println!("  Please enter a value between 1 and 255."),
        }
    }
}

fn prompt_u32(label: &str, default: u32) -> u32 {
    loop {
        let s = prompt(&format!("  {label} [{default}]: "));
        if s.is_empty() {
            return default;
        }
        match s.parse() {
            Ok(v) => return v,
            Err(_) => println!("  Please enter a valid integer."),
        }
    }
}

fn section(title: &str) {
    println!("\n── {title} ──");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();
    let mut bus = Bus::open(&args.port, args.baud)?;

    println!("=== BEAR Motor Setup ===");
    println!("Connecting to motor ID {} on {}...", args.id, args.port);
    bus.ping(args.id)?;
    println!("Connected.");

    let id = args.id;

    // Config registers can only be written while torque is disabled.
    bus.write_torque_enable(id, 0)?;

    // ── 1. Motor ID ──────────────────────────────────────────────────────────
    section("1. Motor ID");
    let new_id = prompt_u8(&format!("New ID (keep {id} to skip)"), id);
    if new_id != id {
        bus.write_id(id, new_id as u32)?;
        println!("  ID register set to {new_id}. Takes effect after save + reboot.");
    } else {
        println!("  ID unchanged.");
    }

    // ── 2. Id/Iq current loop gains ──────────────────────────────────────────
    section("2. Motor Type  (Id / Iq Gains)");
    for (i, p) in MOTOR_PROFILES.iter().enumerate() {
        println!("  {}. {:<25}  P={:.3}  I={:.3}  D={:.3}", i + 1, p.name, p.p_gain, p.i_gain, p.d_gain);
    }
    let profile = loop {
        let s = prompt(&format!("  Select [1-{}]: ", MOTOR_PROFILES.len()));
        match s.parse::<usize>() {
            Ok(n) if n >= 1 && n <= MOTOR_PROFILES.len() => break &MOTOR_PROFILES[n - 1],
            _ => println!("  Please enter a number between 1 and {}.", MOTOR_PROFILES.len()),
        }
    };
    bus.write_p_gain_id(id, profile.p_gain)?;
    bus.write_i_gain_id(id, profile.i_gain)?;
    bus.write_d_gain_id(id, profile.d_gain)?;
    bus.write_p_gain_iq(id, profile.p_gain)?;
    bus.write_i_gain_iq(id, profile.i_gain)?;
    bus.write_d_gain_iq(id, profile.d_gain)?;
    println!("  Gains set for {}.", profile.name);

    // ── 3. Mode ──────────────────────────────────────────────────────────────
    section("3. Mode");
    let mode = prompt_u32("Mode", 2);
    bus.write_mode(id, mode)?;

    // ── 4. Position loop gains ───────────────────────────────────────────────
    section("4. Position Loop Gains (PGainPos / IGainPos / DGainPos)");
    let p_gain_pos = prompt_f32("P Gain", 17.0);
    let i_gain_pos = prompt_f32("I Gain", 0.0);
    let d_gain_pos = prompt_f32("D Gain", 1.5);
    bus.write_p_gain_pos(id, p_gain_pos)?;
    bus.write_i_gain_pos(id, i_gain_pos)?;
    bus.write_d_gain_pos(id, d_gain_pos)?;

    // ── 5. Max velocity ──────────────────────────────────────────────────────
    section("5. Max Velocity (rad/s)");
    let vel_max = prompt_f32("LimitVelMax", 40.0);
    bus.write_limit_vel_max(id, vel_max)?;

    // ── 6. Max acceleration ──────────────────────────────────────────────────
    section("6. Max Acceleration (rad/s²)");
    let acc_max = prompt_f32("LimitAccMax", 80.0);
    bus.write_limit_acc_max(id, acc_max)?;

    // ── 7. IMAX ──────────────────────────────────────────────────────────────
    section("7. Max Current (A)");
    let i_max = prompt_f32("LimitIMax", 5.0);
    bus.write_limit_i_max(id, i_max)?;

    // ── 8. Min position limit ────────────────────────────────────────────────
    section("8. Minimum Position Limit");
    let pos = bus.read_present_pos(id)?.data;
    println!("  Current position: {pos:.4} rad");
    prompt("  Move motor to the MINIMUM position, then press Enter...");
    let pos_min = bus.read_present_pos(id)?.data;
    bus.write_limit_pos_min(id, pos_min)?;
    println!("  LimitPosMin set to {pos_min:.4} rad");

    // ── 9. Max position limit ────────────────────────────────────────────────
    section("9. Maximum Position Limit");
    let pos = bus.read_present_pos(id)?.data;
    println!("  Current position: {pos:.4} rad");
    prompt("  Move motor to the MAXIMUM position, then press Enter...");
    let pos_max = bus.read_present_pos(id)?.data;
    bus.write_limit_pos_max(id, pos_max)?;
    println!("  LimitPosMax set to {pos_max:.4} rad");

    // ── 10. Save ─────────────────────────────────────────────────────────────
    section("10. Saving Configuration");
    bus.save_config(id)?;
    println!("  Saved.");

    // ── Summary (read back from motor) ───────────────────────────────────────
    let rb_mode      = bus.read_mode(id)?.data;
    let rb_p_id      = bus.read_p_gain_id(id)?.data;
    let rb_i_id      = bus.read_i_gain_id(id)?.data;
    let rb_d_id      = bus.read_d_gain_id(id)?.data;
    let rb_p_pos     = bus.read_p_gain_pos(id)?.data;
    let rb_i_pos     = bus.read_i_gain_pos(id)?.data;
    let rb_d_pos     = bus.read_d_gain_pos(id)?.data;
    let rb_vel_max   = bus.read_limit_vel_max(id)?.data;
    let rb_acc_max   = bus.read_limit_acc_max(id)?.data;
    let rb_i_max     = bus.read_limit_i_max(id)?.data;
    let rb_pos_min   = bus.read_limit_pos_min(id)?.data;
    let rb_pos_max   = bus.read_limit_pos_max(id)?.data;

    println!("\n╔══════════════════════════════════╗");
    println!("║        Setup Complete            ║");
    println!("╠══════════════════════════════════╣");
    if new_id != id {
        println!("║  Motor ID:    {id:<3} → {new_id:<3} (on reboot)   ║");
    } else {
        println!("║  Motor ID:    {id:<3}                    ║");
    }
    println!("║  Motor type:  {:<19} ║", profile.name);
    println!("║  Mode:        {rb_mode:<3}                    ║");
    println!("║─────────────────────────────────║");
    println!("║  Id/Iq  P:    {rb_p_id:<19.3} ║");
    println!("║  Id/Iq  I:    {rb_i_id:<19.3} ║");
    println!("║  Id/Iq  D:    {rb_d_id:<19.3} ║");
    println!("║─────────────────────────────────║");
    println!("║  Pos    P:    {rb_p_pos:<19.3} ║");
    println!("║  Pos    I:    {rb_i_pos:<19.3} ║");
    println!("║  Pos    D:    {rb_d_pos:<19.3} ║");
    println!("║─────────────────────────────────║");
    println!("║  Vel max:     {rb_vel_max:<19.3} ║");
    println!("║  Acc max:     {rb_acc_max:<19.3} ║");
    println!("║  I max:       {rb_i_max:<19.3} ║");
    println!("║─────────────────────────────────║");
    println!("║  Pos min:     {rb_pos_min:<+19.4} ║");
    println!("║  Pos max:     {rb_pos_max:<+19.4} ║");
    println!("╚══════════════════════════════════╝");

    Ok(())
}
