//! Run the fit over a capture from the board, on your laptop.
//!
//! ```text
//! cargo run -p packet --example decode -- board.bin --csv > samples.csv
//! cargo run -p magcal --example survey -- samples.csv
//! ```
//!
//! The input is three raw magnetometer readings a line, in nanotesla, before
//! any offset is taken off them — what `packet`'s `decode --csv` pulls out of a
//! captured stream. This is the same arithmetic the firmware runs, over the
//! same samples the firmware ran it over, so it should reach the same answer
//! the board reported. If it does not, the difference is the board's `f32`
//! accumulation against your laptop's, and that is worth knowing about.
//!
//! The CSV is why this is possible at all: the samples are a file, so anything
//! that reads a CSV can re-fit the identical bytes and be compared against the
//! board's answer. A tool that captured its own samples could only ever be
//! compared against a different run.
//!
//! Expect a captured *flat spin* to be refused. That is the point — see the
//! crate docs on why a circle cannot locate the centre of a sphere.

use magcal::{Fit, glam::Vec3};

/// One `mx,my,mz` row. Blank lines and `#` comments are skipped so a capture
/// can be annotated by hand without breaking the reader.
fn sample(line: &str) -> Option<Vec3> {
    let line = line.trim();
    if line.is_empty() || line.starts_with('#') {
        return None;
    }

    let mut axes = line.split(',').map(|axis| axis.trim().parse::<f32>());
    let x = axes.next()?.ok()?;
    let y = axes.next()?.ok()?;
    let z = axes.next()?.ok()?;
    if axes.next().is_some() {
        return None;
    }

    Some(Vec3::new(x, y, z))
}

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: survey <samples.csv>");
        std::process::exit(2);
    });
    let text = std::fs::read_to_string(&path).unwrap_or_else(|error| {
        eprintln!("{path}: {error}");
        std::process::exit(2);
    });

    let mut fit = Fit::new();
    let mut rows = 0usize;
    let mut unreadable = 0usize;

    for line in text.lines() {
        if line.trim().is_empty() || line.trim_start().starts_with('#') {
            continue;
        }
        rows += 1;
        match sample(line) {
            Some(reading) => fit.push(reading),
            None => unreadable += 1,
        }
    }

    println!("{path}");
    println!("  rows       {rows}");
    println!("  accepted   {}", fit.samples());
    if unreadable > 0 {
        println!("  unreadable {unreadable} — expected three numbers a line");
    }
    println!("  sectors    {:#010b}", fit.sectors());
    println!("  spread     {:.6}", fit.spread());
    println!("  ready      {}", fit.is_ready());

    match fit.solve() {
        Ok(s) => {
            let offset = s.calibration.offset();
            println!(
                "  centre     ({:.0}, {:.0}, {:.0}) nT",
                offset.x, offset.y, offset.z
            );
            println!("  offset     {:.0} nT", offset.length());
            println!("  radius     {:.0} nT", s.radius);
            println!("  residual   {:.0} nT", s.residual);
            println!("  ratio      {:.2}", offset.length() / s.radius);
        }
        Err(e) => println!("  REFUSED    {e:?}"),
    }
}
