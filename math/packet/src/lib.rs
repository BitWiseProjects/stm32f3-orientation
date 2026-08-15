//! The wire format, written down once.
//!
//! The board sends bytes down a serial line and the browser reads them. That
//! only works if both ends agree on where every field starts, and the way to
//! guarantee they agree is for both of them to compile this file — the same
//! trick [`vector`](../vector/index.html) plays with the transform that draws
//! the board.
//!
//! Before this crate existed the format was written out by hand in four
//! places, and the only thing keeping them in step was care.
//!
//! # Two packets
//!
//! Every frame begins with a two-byte marker and ends with a CRC-8 over
//! everything between them. The marker's second byte is what says which packet
//! this is:
//!
//! | Marker | Packet | Length |
//! |---|---|---|
//! | `AA 55` | [`Orientation`] — where the board is pointing | 19 bytes |
//! | `AA 56` | [`Calibration`] — one magnetometer sample and the fit so far | 46 bytes |
//!
//! Splitting on the marker rather than adding a type byte after it is
//! deliberate: the orientation packet predates the calibration one, three
//! firmware stages already emit it, and the episode has already explained it
//! on camera. Reaching back to change it would have cost more than it saved.
//!
//! # Reading a stream you joined late
//!
//! A receiver hunts for a marker, looks the length up with [`frame_len`],
//! checks the CRC, and either takes the whole frame or steps forward a single
//! byte and looks again. Stepping one byte matters: two payload bytes can
//! happen to read `AA 55`, and skipping a whole frame's worth on the strength
//! of that would throw away a real packet that starts inside it.
//!
//! Looking the length up is what keeps the two packets from interfering. A
//! receiver that knew only about the 19-byte packet would scan straight
//! through a calibration frame's payload, and sooner or later find a marker
//! and a passing checksum in there by coincidence.
//!
//! # Little-endian, and why the CRC skips the marker
//!
//! Every `f32` and `u32` is little-endian, which is the byte order the chip
//! and the browser both use natively, so neither end swaps anything.
//!
//! The CRC covers the payload only. The marker is a constant — checking it
//! would prove nothing the receiver did not already know by having matched it.

#![cfg_attr(not(test), no_std)]

use glam::{Quat, Vec3};

// Re-exported so callers do not have to depend on glam separately and risk
// ending up on a different version of it.
pub use glam;

/// The marker every frame begins with. Only the second byte varies.
pub const MARKER: u8 = 0xAA;

/// `AA 55` — the orientation packet.
pub const ORIENTATION_MARKER: [u8; 2] = [MARKER, 0x55];

/// `AA 56` — the calibration packet.
pub const CALIBRATION_MARKER: [u8; 2] = [MARKER, 0x56];

/// Marker, four floats, checksum.
pub const ORIENTATION_LEN: usize = 19;

/// Marker, two vectors, three floats, a count, three status bytes, checksum.
pub const CALIBRATION_LEN: usize = 46;

/// The longest frame there is. A receiver needs this many bytes in hand before
/// it can be sure a marker at the front is not simply incomplete.
pub const MAX_FRAME_LEN: usize = CALIBRATION_LEN;

/// How far a calibration run has got.
///
/// Sent every sample, so the receiver never has to infer the run's state from
/// the numbers moving.
///
/// # A run has two ends, and both of them are on the wire
///
/// [`Solved`](Self::Solved) and [`Refused`](Self::Refused) are the only ways a
/// run finishes, and each is sent exactly once, at the moment it happens. The
/// board returns to fusing on the same tick either way, so the next packet out
/// is already an idle one — without these two the far end would see collecting,
/// collecting, then silence, and would have to guess what that meant.
///
/// Guessing is not good enough, because the two ends need different words.
/// A receiver that read "back to idle" as failure would report a flat spin
/// every time a solved packet lost its checksum, which is a lie told
/// confidently. See [`FitStatus`] for how a refusal says *why*.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum RunState {
    /// Not calibrating. The offset in the packet is the one the read path is
    /// using — the constant compiled into the firmware.
    Idle = 0,
    /// Collecting samples, and not yet past the thresholds.
    Collecting = 1,
    /// Past the thresholds and still collecting. More samples from here only
    /// make the answer better.
    Ready = 2,
    /// The run finished and the fit solved. This packet's offset is the answer,
    /// and the board is already using it.
    Solved = 3,
    /// The run finished and the fit would not solve. The offset in the packet
    /// is the *old* one, which the board has kept, and [`Calibration::fit`]
    /// says what went wrong — most often [`FitStatus::Coplanar`], the flat spin.
    Refused = 4,
}

/// What the last attempt to solve the fit came back with.
///
/// A mirror of `magcal::FitError`, plus the case where there was no error. It
/// is a separate enum rather than that one so that this crate stays a
/// description of the wire and nothing else — a receiver can read the format
/// without linking the maths.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum FitStatus {
    /// The fit solved. The offset, radius and residual are real numbers.
    Ok = 0,
    /// Not enough samples yet.
    TooFewSamples = 1,
    /// The samples lie in a plane — the flat spin. A circle cannot say where
    /// the centre of a sphere is.
    Coplanar = 2,
    /// The system would not eliminate. Stranger than coplanar, and rarer.
    Singular = 3,
    /// A sphere was found and the samples are nowhere near it.
    ///
    /// Not the flat spin — this is the board that was barely moved. Its noise
    /// is isotropic, so it passes every check that looks at where the samples
    /// sit, and fits a sphere the size of the noise floor. Says *"you did not
    /// move it enough"*, where [`Coplanar`](Self::Coplanar) says *"you kept it
    /// flat"*.
    Scattered = 4,
}

impl RunState {
    fn from_byte(byte: u8) -> Option<Self> {
        Some(match byte {
            0 => Self::Idle,
            1 => Self::Collecting,
            2 => Self::Ready,
            3 => Self::Solved,
            4 => Self::Refused,
            _ => return None,
        })
    }
}

impl FitStatus {
    fn from_byte(byte: u8) -> Option<Self> {
        Some(match byte {
            0 => Self::Ok,
            1 => Self::TooFewSamples,
            2 => Self::Coplanar,
            3 => Self::Singular,
            4 => Self::Scattered,
            _ => return None,
        })
    }
}

/// Where the board is pointing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Orientation {
    /// Body to world. Written `w x y z`, which is not the order glam's
    /// constructor takes — see [`Orientation::decode`].
    pub rotation: Quat,
}

/// One magnetometer sample, and everything the fit knows at the moment it was
/// taken.
///
/// Sample and fit state travel together on purpose. The receiver draws a point
/// and the sphere that was fitted when that point arrived, so the cloud
/// filling in and the sphere settling onto it are one event rather than two
/// streams to reconcile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    /// Straight off the sensor, in nT. **Uncorrected** — applying the offset
    /// is the receiver's job, and watching it happen is the point.
    pub sample: Vec3,
    /// In nT. While a run is going this is the fit's current estimate. When
    /// [`state`](Self::state) is [`RunState::Idle`] or [`RunState::Refused`] it
    /// is the offset the read path is actually using — on a refusal the board
    /// keeps the old correction rather than adopting a fit it does not believe.
    pub offset: Vec3,
    /// Field strength the fit found, in nT. Near 50,000 at the earth's
    /// surface; far from it means the fit found something that is not the
    /// earth.
    pub radius: f32,
    /// RMS distance from a sample to the fitted sphere, in nT. Read it against
    /// the radius — a few percent is a clean hard-iron-only fit.
    pub residual: f32,
    /// `magcal`'s conditioning number: 0 is flat, 1 is a uniform ball.
    pub spread: f32,
    /// How many samples have gone in.
    pub samples: u32,
    /// Which of the eight compass sectors have been visited, one bit each.
    pub sectors: u8,
    /// How far the run has got.
    pub state: RunState,
    /// What the last solve came back with. This is the field that carries the
    /// reason on a [`RunState::Refused`] packet, and it is never
    /// [`FitStatus::Ok`] there.
    pub fit: FitStatus,
}

/// One decoded frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Packet {
    Orientation(Orientation),
    Calibration(Calibration),
}

/// How long the frame starting with these bytes is, if it starts a frame at all.
///
/// Give it at least two bytes. Fewer, or a marker this crate does not know,
/// and the answer is `None` — which a receiver should treat as "this is not a
/// frame boundary", not as an error.
pub fn frame_len(bytes: &[u8]) -> Option<usize> {
    match bytes {
        [a, b, ..] if [*a, *b] == ORIENTATION_MARKER => Some(ORIENTATION_LEN),
        [a, b, ..] if [*a, *b] == CALIBRATION_MARKER => Some(CALIBRATION_LEN),
        _ => None,
    }
}

/// Decode a whole frame — marker, payload and checksum.
///
/// `None` covers every way a candidate can fail to be a packet: a marker
/// nobody knows, the wrong number of bytes, a checksum that does not match, or
/// a status byte outside the range this version understands. All of them mean
/// the same thing to a receiver, which is that these bytes are not a packet
/// and it should carry on looking.
pub fn decode(frame: &[u8]) -> Option<Packet> {
    let len = frame_len(frame)?;
    if frame.len() != len {
        return None;
    }

    let payload = &frame[2..len - 1];
    if crc8(payload) != frame[len - 1] {
        return None;
    }

    Some(if frame[1] == ORIENTATION_MARKER[1] {
        Packet::Orientation(Orientation::decode(payload))
    } else {
        Packet::Calibration(Calibration::decode(payload)?)
    })
}

/// Walk a run of bytes, handing back every whole packet in it.
///
/// This is the hunting described at the top of this file, and it is here
/// rather than in the receiver because getting it wrong is subtle and there is
/// more than one receiver. Iterate it, then ask [`consumed`](Self::consumed)
/// how much of the input is finished with — the remainder is a partial frame
/// and belongs at the front of the next run.
///
/// Nothing is allocated and nothing is copied, so this works the same on the
/// board as it does in a browser.
pub struct Scan<'a> {
    bytes: &'a [u8],
    at: usize,
    rejected: u32,
}

impl<'a> Scan<'a> {
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            at: 0,
            rejected: 0,
        }
    }

    /// How many bytes from the front are dealt with — decoded, or discarded as
    /// not being the start of anything.
    ///
    /// Only meaningful once iteration has stopped.
    pub fn consumed(&self) -> usize {
        self.at
    }

    /// Candidates whose marker matched and whose checksum did not.
    ///
    /// Some of these are real corruption. The rest are payload bytes that
    /// happened to read like a marker, which is not a fault at all — which is
    /// why this is a count to watch rather than an error to report.
    pub fn rejected(&self) -> u32 {
        self.rejected
    }
}

impl Iterator for Scan<'_> {
    type Item = Packet;

    fn next(&mut self) -> Option<Packet> {
        while self.at < self.bytes.len() {
            let rest = &self.bytes[self.at..];

            let Some(len) = frame_len(rest) else {
                // Too few bytes to even tell — leave them for next time.
                if rest.len() < 2 {
                    return None;
                }
                self.at += 1;
                continue;
            };

            if rest.len() < len {
                // A marker, but the rest of the frame has not arrived.
                return None;
            }

            match decode(&rest[..len]) {
                Some(decoded) => {
                    self.at += len;
                    return Some(decoded);
                }
                None => {
                    // Step one byte, not one frame. A real packet can start
                    // inside what would otherwise be skipped.
                    self.rejected += 1;
                    self.at += 1;
                }
            }
        }

        None
    }
}

impl Orientation {
    /// Payload layout, from the start of the payload:
    ///
    /// | At | Size | Field |
    /// |---|---|---|
    /// | 0 | 4 | `w` |
    /// | 4 | 4 | `x` |
    /// | 8 | 4 | `y` |
    /// | 12 | 4 | `z` |
    pub fn encode(&self) -> [u8; ORIENTATION_LEN] {
        let mut frame = [0u8; ORIENTATION_LEN];
        frame[0..2].copy_from_slice(&ORIENTATION_MARKER);

        let q = self.rotation;
        put_f32(&mut frame, 2, q.w);
        put_f32(&mut frame, 6, q.x);
        put_f32(&mut frame, 10, q.y);
        put_f32(&mut frame, 14, q.z);

        frame[18] = crc8(&frame[2..18]);
        frame
    }

    fn decode(payload: &[u8]) -> Self {
        // glam takes x, y, z, w; the packet leads with w.
        Self {
            rotation: Quat::from_xyzw(
                get_f32(payload, 4),
                get_f32(payload, 8),
                get_f32(payload, 12),
                get_f32(payload, 0),
            ),
        }
    }
}

impl Calibration {
    /// Payload layout, from the start of the payload:
    ///
    /// | At | Size | Field |
    /// |---|---|---|
    /// | 0 | 12 | `sample` x, y, z |
    /// | 12 | 12 | `offset` x, y, z |
    /// | 24 | 4 | `radius` |
    /// | 28 | 4 | `residual` |
    /// | 32 | 4 | `spread` |
    /// | 36 | 4 | `samples` |
    /// | 40 | 1 | `sectors` |
    /// | 41 | 1 | `state` |
    /// | 42 | 1 | `fit` |
    pub fn encode(&self) -> [u8; CALIBRATION_LEN] {
        let mut frame = [0u8; CALIBRATION_LEN];
        frame[0..2].copy_from_slice(&CALIBRATION_MARKER);

        put_vec3(&mut frame, 2, self.sample);
        put_vec3(&mut frame, 14, self.offset);
        put_f32(&mut frame, 26, self.radius);
        put_f32(&mut frame, 30, self.residual);
        put_f32(&mut frame, 34, self.spread);
        frame[38..42].copy_from_slice(&self.samples.to_le_bytes());
        frame[42] = self.sectors;
        frame[43] = self.state as u8;
        frame[44] = self.fit as u8;

        frame[45] = crc8(&frame[2..45]);
        frame
    }

    fn decode(payload: &[u8]) -> Option<Self> {
        Some(Self {
            sample: get_vec3(payload, 0),
            offset: get_vec3(payload, 12),
            radius: get_f32(payload, 24),
            residual: get_f32(payload, 28),
            spread: get_f32(payload, 32),
            samples: u32::from_le_bytes([payload[36], payload[37], payload[38], payload[39]]),
            sectors: payload[40],
            state: RunState::from_byte(payload[41])?,
            fit: FitStatus::from_byte(payload[42])?,
        })
    }
}

/// CRC-8, polynomial `0x07`.
///
/// A plain sum would miss two errors that cancel, and byte order entirely.
/// This is eight lines and catches essentially everything a serial line does
/// to a frame this short.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;

    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }

    crc
}

fn put_f32(frame: &mut [u8], at: usize, value: f32) {
    frame[at..at + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_vec3(frame: &mut [u8], at: usize, value: Vec3) {
    put_f32(frame, at, value.x);
    put_f32(frame, at + 4, value.y);
    put_f32(frame, at + 8, value.z);
}

fn get_f32(payload: &[u8], at: usize) -> f32 {
    f32::from_le_bytes([
        payload[at],
        payload[at + 1],
        payload[at + 2],
        payload[at + 3],
    ])
}

fn get_vec3(payload: &[u8], at: usize) -> Vec3 {
    Vec3::new(
        get_f32(payload, at),
        get_f32(payload, at + 4),
        get_f32(payload, at + 8),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn a_calibration() -> Calibration {
        Calibration {
            sample: Vec3::new(21_000.0, -14_915.5, -33_100.25),
            offset: Vec3::new(9265.5, -28_189.5, -11_732.9),
            radius: 46_917.9,
            residual: 1158.5,
            spread: 0.1234,
            samples: 1512,
            sectors: 0b1110_1111,
            state: RunState::Ready,
            fit: FitStatus::Ok,
        }
    }

    /// Every state the wire can carry, found by asking the decoder rather than
    /// by writing a list next to the enum.
    ///
    /// A hand-written list is exactly the thing that goes stale when a variant
    /// is added — the tests keep passing while quietly covering one case fewer.
    /// Sweeping the byte space cannot go stale, and the count assertions in
    /// [`the_status_enums_are_the_size_this_module_thinks_they_are`] fail loudly
    /// if a variant arrives without anyone revisiting this module.
    fn every_run_state() -> Vec<RunState> {
        (0..=u8::MAX).filter_map(RunState::from_byte).collect()
    }

    fn every_fit_status() -> Vec<FitStatus> {
        (0..=u8::MAX).filter_map(FitStatus::from_byte).collect()
    }

    #[test]
    fn an_orientation_survives_the_round_trip() {
        let sent = Orientation {
            rotation: Quat::from_xyzw(0.1, -0.2, 0.3, 0.9).normalize(),
        };

        let frame = sent.encode();
        assert_eq!(frame.len(), ORIENTATION_LEN);
        assert_eq!(decode(&frame), Some(Packet::Orientation(sent)));
    }

    #[test]
    fn a_calibration_survives_the_round_trip() {
        let sent = a_calibration();

        let frame = sent.encode();
        assert_eq!(frame.len(), CALIBRATION_LEN);
        assert_eq!(decode(&frame), Some(Packet::Calibration(sent)));
    }

    #[test]
    fn the_two_packets_are_told_apart_by_their_marker() {
        assert_eq!(
            frame_len(
                &Orientation {
                    rotation: Quat::IDENTITY
                }
                .encode()
            ),
            Some(ORIENTATION_LEN)
        );
        assert_eq!(frame_len(&a_calibration().encode()), Some(CALIBRATION_LEN));
        assert_eq!(frame_len(&[0xAA, 0x57]), None);
        assert_eq!(frame_len(&[0xAA]), None);
        assert_eq!(frame_len(&[]), None);
    }

    #[test]
    fn every_single_bit_flip_is_caught() {
        // The whole reason there is a checksum rather than a length and hope.
        //
        // Run over every state, because the status byte is the one place where
        // a flip can land on another legal value — `Solved` is 3 and `Refused`
        // is 4, so a single bit turns 5 into 4 and a corrupt frame into a
        // plausible refusal. The checksum is what stands between those, and it
        // has to be checked against each pattern rather than one of them.
        for state in every_run_state() {
            let frame = Calibration {
                state,
                ..a_calibration()
            }
            .encode();

            for byte in 0..frame.len() {
                for bit in 0..8 {
                    let mut corrupted = frame;
                    corrupted[byte] ^= 1 << bit;

                    // Flipping a marker bit stops it being this packet at all,
                    // which the receiver handles by not matching rather than by
                    // failing the checksum.
                    if byte < 2 && frame_len(&corrupted) != Some(CALIBRATION_LEN) {
                        continue;
                    }

                    assert_eq!(
                        decode(&corrupted),
                        None,
                        "{state:?}: byte {byte} bit {bit} got through"
                    );
                }
            }
        }
    }

    #[test]
    fn a_frame_of_the_wrong_length_is_refused() {
        let frame = a_calibration().encode();

        assert_eq!(decode(&frame[..CALIBRATION_LEN - 1]), None);
        assert_eq!(
            decode(
                &Orientation {
                    rotation: Quat::IDENTITY
                }
                .encode()[..18]
            ),
            None
        );
    }

    #[test]
    fn the_status_enums_are_the_size_this_module_thinks_they_are() {
        // Not busywork. Every exhaustiveness test below sweeps whatever the
        // decoder accepts, so a variant added to the enum and to `from_byte`
        // would be swept silently and this file would never be revisited. This
        // is the tripwire that says: a state was added, go and decide what it
        // means at both ends.
        assert_eq!(every_run_state().len(), 5, "RunState gained or lost a variant");
        assert_eq!(every_fit_status().len(), 5, "FitStatus gained or lost a variant");
    }

    #[test]
    fn every_state_and_fit_combination_survives_the_round_trip() {
        for state in every_run_state() {
            for fit in every_fit_status() {
                let sent = Calibration {
                    state,
                    fit,
                    ..a_calibration()
                };

                assert_eq!(
                    decode(&sent.encode()),
                    Some(Packet::Calibration(sent)),
                    "{state:?} with {fit:?} did not survive"
                );
            }
        }
    }

    #[test]
    fn a_status_byte_from_a_newer_firmware_is_refused_rather_than_guessed() {
        // Both status bytes, every value, not a sample of them. A byte this
        // build does not understand means the frame came from firmware it does
        // not understand, and the only safe reading of that is "not a packet".
        for byte in 0..=u8::MAX {
            for at in [43usize, 44] {
                let mut frame = a_calibration().encode();
                frame[at] = byte;
                frame[45] = crc8(&frame[2..45]);

                let known = if at == 43 {
                    RunState::from_byte(byte).is_some()
                } else {
                    FitStatus::from_byte(byte).is_some()
                };

                assert_eq!(
                    decode(&frame).is_some(),
                    known,
                    "byte {byte} at offset {at} was read wrongly"
                );
            }
        }
    }

    #[test]
    fn the_status_bytes_land_where_the_layout_says() {
        // Pinned one at a time, because these are wire values: renumbering a
        // variant is a silent protocol break that nothing else here would
        // catch, and `Refused = 4` in particular has to stay off the end so
        // that the four older values keep meaning what they always meant.
        for (state, byte) in [
            (RunState::Idle, 0),
            (RunState::Collecting, 1),
            (RunState::Ready, 2),
            (RunState::Solved, 3),
            (RunState::Refused, 4),
        ] {
            let frame = Calibration {
                state,
                ..a_calibration()
            }
            .encode();
            assert_eq!(frame[43], byte, "{state:?} moved on the wire");
            assert_eq!(RunState::from_byte(byte), Some(state));
        }

        for (fit, byte) in [
            (FitStatus::Ok, 0),
            (FitStatus::TooFewSamples, 1),
            (FitStatus::Coplanar, 2),
            (FitStatus::Singular, 3),
            (FitStatus::Scattered, 4),
        ] {
            let frame = Calibration {
                fit,
                ..a_calibration()
            }
            .encode();
            assert_eq!(frame[44], byte, "{fit:?} moved on the wire");
            assert_eq!(FitStatus::from_byte(byte), Some(fit));
        }
    }

    #[test]
    fn a_refusal_reaches_the_far_end_with_its_reason_and_the_old_offset() {
        // The packet 1.5 originally left out. A flat spin has to arrive as a
        // finished run that failed, saying why — not as a run that stopped
        // talking, which is what a receiver would otherwise have to interpret.
        let kept = Vec3::new(9265.5, -28_189.5, -11_732.9);
        let sent = Calibration {
            offset: kept,
            radius: 0.0,
            residual: 0.0,
            state: RunState::Refused,
            fit: FitStatus::Coplanar,
            ..a_calibration()
        };

        let Some(Packet::Calibration(got)) = decode(&sent.encode()) else {
            panic!("a refusal did not decode as a calibration packet");
        };

        assert_eq!(got.state, RunState::Refused);
        assert_eq!(got.fit, FitStatus::Coplanar);
        // The board keeps the correction it already had rather than adopting a
        // fit it just rejected, and the packet has to say so.
        assert_eq!(got.offset, kept);
        // Nothing was solved, so there is no field strength to report and the
        // packet must not imply one.
        assert_eq!(got.radius, 0.0);
        assert_eq!(got.residual, 0.0);
    }

    #[test]
    fn a_refusal_is_not_mistakeable_for_an_idle_packet() {
        // The distinction the viewer's whole state machine turns on: idle is
        // the 2 Hz heartbeat between runs, refused is a terminus that happens
        // once. They must not be told apart by the numbers being zero, because
        // an idle packet before any run has those zeros too.
        let refused = Calibration {
            state: RunState::Refused,
            fit: FitStatus::Coplanar,
            radius: 0.0,
            residual: 0.0,
            samples: 0,
            sectors: 0,
            ..a_calibration()
        };
        let idle = Calibration {
            state: RunState::Idle,
            ..refused
        };

        assert_ne!(refused.encode(), idle.encode());
        assert_ne!(refused, idle);
    }

    #[test]
    fn a_scan_finds_packets_in_a_stream_it_joined_late() {
        // A ragged start, both packet types, and a truncated one at the end —
        // which is what every real connect looks like.
        let mut stream = vec![0x8F, 0xFF];
        stream.extend_from_slice(&a_calibration().encode());
        stream.extend_from_slice(
            &Orientation {
                rotation: Quat::IDENTITY,
            }
            .encode(),
        );
        let tail = a_calibration().encode();
        stream.extend_from_slice(&tail[..10]);

        let mut scan = Scan::new(&stream);
        let found: Vec<_> = scan.by_ref().collect();

        assert_eq!(found.len(), 2);
        assert_eq!(found[0], Packet::Calibration(a_calibration()));
        assert_eq!(scan.rejected(), 0);

        // The ten bytes of the truncated frame are left for the next chunk.
        assert_eq!(scan.consumed(), stream.len() - 10);
    }

    #[test]
    fn a_whole_run_comes_back_out_of_a_scan_in_order() {
        // The shape a receiver actually sees, both ways a run can end. The
        // terminal packet is sent exactly once, so a scanner that dropped it —
        // or reordered it behind the idle heartbeat that follows a millisecond
        // later — would lose the answer with nothing to indicate it had.
        for ending in [RunState::Solved, RunState::Refused] {
            let run = [
                RunState::Idle,
                RunState::Collecting,
                RunState::Collecting,
                RunState::Ready,
                ending,
                RunState::Idle,
            ];

            let mut stream = Vec::new();
            for state in run {
                stream.extend_from_slice(
                    &Calibration {
                        state,
                        ..a_calibration()
                    }
                    .encode(),
                );
                // Orientation packets are interleaved with these on the real
                // wire, three to every one of them.
                stream.extend_from_slice(
                    &Orientation {
                        rotation: Quat::IDENTITY,
                    }
                    .encode(),
                );
            }

            let mut scan = Scan::new(&stream);
            let states: Vec<RunState> = scan
                .by_ref()
                .filter_map(|p| match p {
                    Packet::Calibration(c) => Some(c.state),
                    Packet::Orientation(_) => None,
                })
                .collect();

            assert_eq!(states, run, "a run ending in {ending:?} did not survive");
            assert_eq!(scan.rejected(), 0);
            assert_eq!(scan.consumed(), stream.len());
        }
    }

    #[test]
    fn a_marker_inside_a_payload_does_not_swallow_the_packet_behind_it() {
        // Two bytes that read like an orientation marker, followed by a real
        // packet that starts inside the frame a naive parser would skip.
        let real = Orientation {
            rotation: Quat::from_xyzw(0.0, 0.0, 0.6, 0.8),
        }
        .encode();

        let mut stream = vec![0xAA, 0x55, 0x01, 0x02];
        stream.extend_from_slice(&real);

        let mut scan = Scan::new(&stream);
        let found: Vec<_> = scan.by_ref().collect();

        assert_eq!(found.len(), 1, "the real packet was skipped over");
        assert_eq!(scan.rejected(), 1);
    }

    #[test]
    fn the_crc_matches_the_routine_the_firmware_has_always_run() {
        // Worked out by hand against the polynomial, so this pins the
        // convention rather than merely agreeing with itself.
        assert_eq!(crc8(&[]), 0x00);
        assert_eq!(crc8(&[0x00]), 0x00);
        assert_eq!(crc8(b"123456789"), 0xF4);
    }
}
