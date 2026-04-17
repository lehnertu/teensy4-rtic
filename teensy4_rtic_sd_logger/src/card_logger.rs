//! Tag logging writing to an SD card on a Teensy 4.1
//! 
//! The logger uses raw block writes to an `sdio_host`.
//! All card initialization and block management is handled by the `sdio_host`
//! 
//! The datasets written to the card contain:
//! - a u16 type marker
//! - a u32 timestamp (in milliseconds)
//! - a u16 data length (in bytes)
//! - a variable length data payload (up to 240 bytes)
//! 
//! For RTIC and tight timing constraints, we use a producer/consumer split:
//! 1) High-priority tasks only enqueue dataset bytes into an in-RAM buffer (very short critical section).
//! 2) When a buffer becomes full, that triggers a low-priority writer task.
//! 3) Writer task owns SD writes and drains ready buffers to card.
//! 4) Use a small ring of block buffers so producers can continue while one buffer is being written.
//! 
//! For calling the logger API there are some macros provided:
//! card_logger::info!()
//! card_logger::warning!()
//! card_logger::error!()
//! card_logger::raw!(type_id: U16, payload_bytes: &[u8])

#![allow(dead_code)]

use imxrt_usdhc::BlockingSdioHost;

/// SD card block size in bytes.
pub const BLOCKSIZE: usize = 512;

/// Number of RAM blocks used in the logger ring buffer.
pub const RING_BLOCK_COUNT: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockRingError {
	RingFull,
}

/// In-memory ring buffer of fixed-size card blocks.
///
/// This structure tracks:
/// - full state of each block
/// - current write block index
/// - current write byte position inside that block
pub struct BlockRingBuffer {
	blocks: [[u8; BLOCKSIZE]; RING_BLOCK_COUNT],
	full: [bool; RING_BLOCK_COUNT],
	write_block: usize,
	write_pos: usize,
    next_block_to_flush: usize,
}

impl BlockRingBuffer {

    /// Creates an empty ring buffer.
	pub const fn new() -> Self {
		Self {
			blocks: [[0u8; BLOCKSIZE]; RING_BLOCK_COUNT],
			full: [false; RING_BLOCK_COUNT],
			write_block: 0,
			write_pos: 0,
            next_block_to_flush: 0,
		}
	}

    /// Announces that the next block is ready to be flushed to the card.
    pub fn have_pending_flush(&self) -> bool {
        self.full[self.next_block_to_flush]
    }

    /// Returns a reference to the next block to be flushed to the card.
	pub fn current_flush_block(&self) -> &[u8; BLOCKSIZE] {
		&self.blocks[self.next_block_to_flush]
	}

	/// Marks one block empty after the writer has flushed it.
	pub fn current_flush_done(&mut self) {
		self.full[self.next_block_to_flush] = false;
		self.next_block_to_flush = (self.next_block_to_flush + 1) % RING_BLOCK_COUNT;
	}

    /// Returns the total writable byte capacity.
    /// This is the free space in the current write block
    /// plus 200 bytes in the next block if that is empty.
    /// Capping the next-block contribution to 200 bytes ensures
    /// that write_pos never reaches BLOCKSIZE after a split write.
    fn writable_capacity(&self) -> usize {
		let current_free = BLOCKSIZE.saturating_sub(self.write_pos);
		let next = (self.write_block + 1) % RING_BLOCK_COUNT;
		if !self.full[next] {
			current_free + 200
		} else {
			current_free
		}
	}

	/// Appends bytes to the ring buffer, splitting into the next block if needed.
	///
	/// This method performs the block placement logic for the producer path:
	/// - writes at current `write_block` / `write_pos`
	/// - marks blocks full when they are filled
	/// - advances to the next empty block
	///
	/// Returns `BlockRingError::RingFull` if there is not enough free space.
	pub fn emplace_bytes(&mut self, data: &[u8]) -> Result<(), BlockRingError> {
        // handle empty data case
		if data.is_empty() {
			return Ok(());
		}
        // check if there is enough capacity for the new data
		if self.writable_capacity() < data.len() {
			return Err(BlockRingError::RingFull);
		}
        // check if we stay within the current block
        if self.write_pos + data.len() <= BLOCKSIZE {
			let dst_start = self.write_pos;
			let dst_end = dst_start + data.len();
			self.blocks[self.write_block][dst_start..dst_end].copy_from_slice(data);
			self.write_pos = dst_end;
            // if we exactly hit the block boundary, mark it full and advance
			if self.write_pos == BLOCKSIZE {
				self.full[self.write_block] = true;
				self.write_block = (self.write_block + 1) % RING_BLOCK_COUNT;
				self.write_pos = 0;
			}
        } else {
            // we need to split the data across the current block and the next block
			let first_part_len = BLOCKSIZE - self.write_pos;
			let second_part_len = data.len() - first_part_len;
            // fill the current block to the end
			let dst_start = self.write_pos;
			self.blocks[self.write_block][dst_start..BLOCKSIZE]
				.copy_from_slice(&data[..first_part_len]);
            // advance to the next block
            // Note: we know the next block is empty because writable_capacity() check passed
			self.full[self.write_block] = true;
			self.write_block = (self.write_block + 1) % RING_BLOCK_COUNT;
			self.write_pos = 0;
            // fill the remaining data into the next block
            // Note: we know this can be at most 200 bytes because of the writable_capacity() check
			self.blocks[self.write_block][..second_part_len]
				.copy_from_slice(&data[first_part_len..]);
			self.write_pos = second_part_len;
        }
        return Ok(());
    }
}

/// Buffer size in bytes for a single message w/o headers.
pub const MAX_PAYLOAD_BYTES: usize = 240;
pub const BUFFERSIZE: usize = MAX_PAYLOAD_BYTES + 8;

/// Type marker ranges and known built-in markers.
pub mod type_ids {
	/// Built-in/logger-owned markers.
	pub const ERROR_TEXT: u16 = 0xCC00;
    pub const WARNING_TEXT: u16 = 0xCC01;
	pub const INFO_TEXT: u16 = 0xCC02;

	/// User/application-owned markers.
	pub const USER_MIN: u16 = 0xCC80;
	pub const USER_MAX: u16 = 0xCCFF;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CardLoggerError {
    IllegalType,
	RingFull,
    PayloadTooLarge,
	CardIo,
}
/// card logger.
pub struct CardLogger {
    /// the card interface driver
	host: BlockingSdioHost,
    /// the next block on the card to be written
	next_card_block: u32,
    /// in-memory ring buffer of blocks to be filled
    /// written to the card when full
	block_ring: BlockRingBuffer,
}

impl CardLogger {
	/// Creates a logger and consumes the SD host.
	/// The logger becomes the exclusive SD card owner.
	pub fn new(host: BlockingSdioHost) -> Self {
		Self {
			host,
			next_card_block: 0,
			block_ring: BlockRingBuffer::new(),
		}
	}

	/// Logs some text
	pub fn log_text(
		&mut self,
		text: &[u8],
	) -> Result<(), CardLoggerError> {
		if text.len() > MAX_PAYLOAD_BYTES {
			return Err(CardLoggerError::PayloadTooLarge);
		}
        // fixed buffer
		let mut frame = [0u8; BUFFERSIZE];
        // encode message type
        frame[0..2].copy_from_slice(&type_ids::INFO_TEXT.to_le_bytes());
        // encode time stamp
        let timestamp_ms = crate::app::system_time_ms();
        frame[2..6].copy_from_slice(&timestamp_ms.to_le_bytes());
        // encode payload size
		let payload_len_u16 = text.len() as u16;
        frame[6..8].copy_from_slice(&payload_len_u16.to_le_bytes());
        frame[8..8 + text.len()].copy_from_slice(text);
        // send the framed message to the ring buffer
		self
			.block_ring
			.emplace_bytes(&frame[..8 + text.len()])
			.map_err(|_| CardLoggerError::RingFull)
	}

	/// Writes one pending full block to the SD card, if any.
    /// Use the board LED as an activity indicator.
	pub fn flush(&mut self, led: &mut teensy4_bsp::board::Led) -> Result<(), CardLoggerError> {
		if !self.block_ring.have_pending_flush() {
			return Ok(());
		}

		led.set();
		let write_result = self
			.host
			.write_block(self.next_card_block, self.block_ring.current_flush_block());
		led.clear();

		write_result.map_err(|_| CardLoggerError::CardIo)?;
		self.next_card_block += 1;
		self.block_ring.current_flush_done();
		Ok(())
	}

}

/// Trait for compile-time known payload types.
///
/// Implementors define a fixed type marker and serialization into a caller-provided byte buffer.
pub trait LogPayload {
	const TYPE_ID: u16;

	/// Returns number of bytes written to `out`.
	fn encode_payload(&self, out: &mut [u8]) -> Result<usize, CardLoggerError>;
}

/// Encodes one dataset into `out` and returns byte count.
///
/// Layout (little-endian):
/// - bytes 0..2:  `type_id` (u16)
/// - bytes 2..6:  `timestamp_ms` (u32)
/// - bytes 6..8:  `payload_len` (u16)
/// - bytes 8..N:  payload
pub fn encode_dataset(
	type_id: u16,
	timestamp_ms: u32,
	payload: &[u8],
	out: &mut [u8],
) -> Result<usize, CardLoggerError> {
	if payload.len() > MAX_PAYLOAD_BYTES {
		return Err(CardLoggerError::PayloadTooLarge);
	}

	let payload_len_u16 = u16::try_from(payload.len()).map_err(|_| CardLoggerError::PayloadTooLarge)?;
	let total = payload.len() + 8;

	if out.len() < total {
		return Err(CardLoggerError::PayloadTooLarge);
	}

	out[0..2].copy_from_slice(&type_id.to_le_bytes());
	out[2..6].copy_from_slice(&timestamp_ms.to_le_bytes());
	out[6..8].copy_from_slice(&payload_len_u16.to_le_bytes());
	out[8..total].copy_from_slice(payload);

	Ok(total)
}

/// Example typed payload.
pub struct TempCentiDeg {
	pub sensor_id: u8,
	pub value_centi_deg: i16,
}

impl LogPayload for TempCentiDeg {
	const TYPE_ID: u16 = type_ids::USER_MIN;

	fn encode_payload(&self, out: &mut [u8]) -> Result<usize, CardLoggerError> {
		const LEN: usize = 3;
		if out.len() < LEN {
			return Err(CardLoggerError::PayloadTooLarge);
		}

		out[0] = self.sensor_id;
		out[1..3].copy_from_slice(&self.value_centi_deg.to_le_bytes());
		Ok(LEN)
	}
}