//! Finding packets in a stream of bytes. The hunting itself is `packet::Scan`.

use packet::{MAX_FRAME_LEN, Packet, Scan};

/// Give up and resynchronize rather than buffer without bound. At 200 packets
/// a second this is a couple of seconds of garbage, which is far more than a
/// resync ever needs.
const MAX_BUFFERED: usize = 8192;

#[derive(Default)]
pub struct Parser {
    buffer: Vec<u8>,
    /// Packets that passed the checksum.
    pub good: u32,
    /// Candidates whose marker matched and whose checksum did not. Some of
    /// these are real corruption; the rest are payload that looked like a
    /// marker, which is not a fault at all.
    pub bad: u32,
}

impl Parser {
    /// Feed a chunk in; get back every whole packet it completed, in order.
    ///
    /// Every packet rather than only the newest, because the calibration
    /// stream is a point cloud — dropping samples there would thin out the
    /// very thing being drawn. Deciding that a stale orientation is not worth
    /// drawing is the caller's business, not the parser's.
    pub fn feed(&mut self, chunk: &[u8]) -> Vec<Packet> {
        self.buffer.extend_from_slice(chunk);

        let mut scan = Scan::new(&self.buffer);
        let found: Vec<Packet> = scan.by_ref().collect();
        let consumed = scan.consumed();
        let rejected = scan.rejected();

        self.good = self.good.wrapping_add(found.len() as u32);
        self.bad = self.bad.wrapping_add(rejected);
        self.buffer.drain(..consumed);

        // If this much has piled up without yielding a packet, something is
        // wrong with the baud rate or the wire, and keeping it will not help.
        if self.buffer.len() > MAX_BUFFERED {
            self.buffer.clear();
        }

        debug_assert!(self.buffer.len() < MAX_BUFFERED + MAX_FRAME_LEN);

        found
    }
}
