//! Decode a captured stream, using the same code the browser runs.
//!
//! Point it at raw bytes read off the serial port and it reports what it found.
//! Useful when the browser is not the thing under suspicion — if this says the
//! stream is good, the firmware end is fine and the problem is on the page.
//!
//! ```text
//! cargo run -p packet --example decode -- board.bin
//! ```
//!
//! Capturing the bytes in the first place, on macOS — note the two traps, that
//! `/dev/cu.*` and not `/dev/tty.*` is the one that opens without waiting for
//! carrier detect, and that `stty` settings revert when it closes the port, so
//! the descriptor has to stay open across both:
//!
//! ```text
//! (stty -f /dev/cu.usbmodem2103 raw 115200 && dd if=/dev/cu.usbmodem2103 of=board.bin bs=1 count=20000) \
//!     < /dev/cu.usbmodem2103
//! ```
//!
//! # `--csv`, and why the fits share an input
//!
//! With `--csv` it stops reporting and starts extracting: one `mx,my,mz` line
//! per calibration packet, on stdout, so the report can still go to the
//! terminal while the samples go to a file.
//!
//! ```text
//! cargo run -p packet --example decode -- board.bin --csv > samples.csv
//! ```
//!
//! Those are the raw magnetometer readings, in nanotesla, before any offset is
//! taken off them — the same numbers the board itself fitted. Two things read
//! that file: `magcal`'s `survey` example, which runs the fit the firmware
//! runs, and `notes/fit_sphere.py`, which runs an unrelated one in another
//! language. Handing both of them the identical bytes is the whole point. Two
//! implementations agreeing on the same samples is evidence; two
//! implementations agreeing on samples they each reconstructed differently is
//! not.
//!
//! Idle packets are left out. The board sends those between runs, and their
//! sample is wherever the board happens to be sitting — a cluster of nearly
//! identical readings that would drag a fit toward one point on the sphere.

use packet::{Packet, RunState, Scan};

fn main() {
    let mut path = None;
    let mut csv = false;
    for arg in std::env::args().skip(1) {
        match arg.as_str() {
            "--csv" => csv = true,
            _ => path = Some(arg),
        }
    }

    let Some(path) = path else {
        eprintln!("usage: decode <file of raw bytes> [--csv]");
        std::process::exit(2);
    };

    let bytes = std::fs::read(&path).unwrap_or_else(|error| {
        eprintln!("{path}: {error}");
        std::process::exit(2);
    });

    // With `--csv` stdout belongs to the samples, so the report moves aside to
    // stderr rather than interleaving itself into the data.
    macro_rules! report {
        ($($arg:tt)*) => {
            if csv { eprintln!($($arg)*) } else { println!($($arg)*) }
        };
    }

    let mut orientations = 0u32;
    let mut calibrations = 0u32;
    let mut written = 0u32;
    let mut idle = 0u32;
    // How each run in the capture ended, in order. Both endings are single
    // packets among thousands, so a capture that holds one says so in one line
    // here and nowhere else.
    let mut endings: Vec<packet::Calibration> = Vec::new();
    let mut first_at = None;
    let mut last = None;
    let mut at = 0;

    // The same scanner the browser runs, over bytes that really came off a
    // board. Walking it one packet at a time is only so the byte offset of the
    // first one can be reported.
    let mut scan = Scan::new(&bytes);
    while let Some(decoded) = scan.next() {
        if first_at.is_none() {
            first_at = Some(scan.consumed() - frame_len_of(&decoded));
        }
        match &decoded {
            Packet::Orientation(_) => orientations += 1,
            Packet::Calibration(c) => {
                calibrations += 1;
                if matches!(c.state, RunState::Solved | RunState::Refused) {
                    endings.push(*c);
                }
                if c.state == RunState::Idle {
                    idle += 1;
                } else if csv {
                    let s = c.sample;
                    println!("{},{},{}", s.x, s.y, s.z);
                    written += 1;
                }
            }
        }
        last = Some(decoded);
        at = scan.consumed();
    }

    let rejected = scan.rejected();
    let leftover = bytes.len() - at;

    report!("{} bytes", bytes.len());
    report!("  {orientations} orientation, {calibrations} calibration");
    report!("  {rejected} rejected, {leftover} bytes left over at the end");

    match first_at {
        Some(0) => report!("  the capture began exactly on a packet boundary"),
        Some(n) => report!("  found the first packet {n} bytes in — joined mid-stream"),
        None => {
            report!("  nothing decoded — wrong baud rate, or not this format");
            std::process::exit(1);
        }
    }

    // Every byte should be inside a packet apart from the ragged one at each
    // end. More than that means the stream is being corrupted, not merely
    // joined late.
    let inside = orientations as usize * packet::ORIENTATION_LEN
        + calibrations as usize * packet::CALIBRATION_LEN;
    let loose = bytes.len() - inside;
    report!("  {inside} bytes inside packets, {loose} loose");
    if loose > packet::MAX_FRAME_LEN * 2 {
        report!("  more loose bytes than a ragged start and end explain");
    }

    if calibrations > 0 {
        if endings.is_empty() {
            report!("  no run ended inside this capture");
        }
        for end in &endings {
            match end.state {
                RunState::Solved => report!(
                    "  SOLVED from {} samples — offset ({:.0}, {:.0}, {:.0}) nT, \
                     field {:.0} nT, scatter {:.2}%",
                    end.samples,
                    end.offset.x,
                    end.offset.y,
                    end.offset.z,
                    end.radius,
                    100.0 * end.residual / end.radius,
                ),
                // The offset here is the *old* one, kept. Saying so matters —
                // a refusal is not a broken compass, it is an unchanged one.
                RunState::Refused => report!(
                    "  REFUSED after {} samples — {:?}, spread {:.4}. \
                     Old offset ({:.0}, {:.0}, {:.0}) nT kept",
                    end.samples,
                    end.fit,
                    end.spread,
                    end.offset.x,
                    end.offset.y,
                    end.offset.z,
                ),
                _ => unreachable!("only terminal states are collected"),
            }
        }
    }

    match last {
        Some(Packet::Orientation(o)) => {
            let (x, y, z) = (o.rotation.x, o.rotation.y, o.rotation.z);
            report!("  last: w {:.5} x {x:.5} y {y:.5} z {z:.5}", o.rotation.w);
            report!(
                "  length {:.6} — a rotation, so this should be 1",
                o.rotation.length()
            );
        }
        Some(Packet::Calibration(c)) => {
            report!(
                "  last: {:?}, {} samples, spread {:.4}, radius {:.0} nT, residual {:.0} nT",
                c.state,
                c.samples,
                c.spread,
                c.radius,
                c.residual
            );
            if c.state == RunState::Idle {
                report!("  not calibrating — the offset is the one compiled in");
            }
        }
        None => unreachable!("first_at was set"),
    }

    if csv {
        report!("  wrote {written} samples, skipped {idle} idle");
        if written == 0 {
            report!("  nothing to fit — was the blue USER button pressed during the capture?");
            std::process::exit(1);
        }
    }
}

/// How long the frame that produced this packet was.
fn frame_len_of(decoded: &Packet) -> usize {
    match decoded {
        Packet::Orientation(_) => packet::ORIENTATION_LEN,
        Packet::Calibration(_) => packet::CALIBRATION_LEN,
    }
}
