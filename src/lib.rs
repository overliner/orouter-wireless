#![cfg_attr(not(feature = "std"), no_std)]
//! Defines and implements protocol used on oRouter's physical radio layer (now using LoRa)
//!
//! Application level message can be theoretically unlimited, but LoRa can only transmit 255B in
//! one message. This protocol takes care of splitting message to appropriate number of parts with
//! necessary added information allowing in to be joined back on receiving end when all parts
//! arrive.
//!
//! [`crate::MessageSlicer`] takes care of the splitting part and is used before the data is
//! transmitted using oRouter. [`crate::MessagePool`] is used on receiving end to put parts of the
//! application level / logical message together to form the original message back. Note that the
//! parts don't have to arrive in order, only that all parts of the message have to arrive
//! eventually.
//!
//! [`crate::WirelessMessagePart`] represents raw chunk of data transmitted/received using oRouters
//! radio chip. This crate implements and uses following scheme for message part:
//!
//! | name          | length in bytes | description                                                                               |
//! |---------------|-----------------|-------------------------------------------------------------------------------------------|
//! | network bytes | 2               | network bytes. always 0xAA 0xCC (will be configurable in next release)                      |
//! | hash          | 6               | hash - first 3B are random, second 3B form a prefix grouping parts of the message to one  |
//! | part_num      | 1               | part number 1, 2 or 3 (only 3-part messages supported)                                    |
//! | total_count   | 1               | total count of messages with this prefix                                                  |
//! | length        | 1               | length of data                                                                            |
//! | msg type      | 1               | overline message type                                                                     |
//! | data type     | 1               | byte identifying data type, if previous field is data                                     |
//! | data          | 1 - 240         | actual data                                                                               |
//! | CRC16         | 2               | CRC16 of the whole message (header + data)                                                |
//!
//! Example of using a [`crate::MessageSlicer`] to split some data for wireless transmission:
//!
//! ```rust,no_run
//! use orouter_wireless::{MessageSlicer, MessageType, network};
//!
//! fn main() {
//!     // VVV in practice provide a good random seed here VVV
//!     let mut slicer = orouter_wireless::MessageSlicer::new(1234u64, network::DEFAULT);
//!     let messages = slicer
//!         .slice(&[0xc0, 0xff, 0xee], MessageType::Data, 0x01).unwrap();
//!     println!("slices = {:?}", messages);
//! }
//! ```
//!
//! Example of using a [`crate::MessagePool`] to assemble data back from received message parts:
//!
//! ```rust
//! use orouter_wireless::MessagePool;
//!
//! fn main() {
//!     let mut message_pool = MessagePool::default();
//!     // this represents a message part received from oRouter
//!     //
//!     // in this example, there is 1 part of total 1 forming the whole message, because the data
//!     // contained in the message are short
//!     for part in vec![
//!         vec![
//!             0xaa, 0xcc, 0x1b, 0xf2, 0x73, 0x86, 0x80, 0xe1, 0x01, 0x01,
//!             0x05, 0x01, 0x01, 0x41, 0x48, 0x4f, 0x59, 0x21, 0x53, 0xef
//!         ]
//!     ] {
//!         match message_pool.try_insert(part.clone()) {
//!             Ok(Some(message)) => assert_eq!(message.data(), b"AHOY!"),
//!             Ok(None) => {}
//!             Err(_) => {
//!                 eprintln!(
//!                     "error while trying to insert message = {:02x?}",
//!                     part
//!                 )
//!             }
//!         }
//!     }
//! }
//! ```
use crc16::{State, X_25};

const NETWORK_BYTES_IDX: usize = 0;
const NETWORK_BYTES_LENGTH: usize = network::DEFAULT.len();

const HASH_RND_PART_IDX: usize = NETWORK_BYTES_IDX + NETWORK_BYTES_LENGTH; // 0 + 2 = 2
const HASH_RND_PART_LENGTH: usize = 3;

const PREFIX_IDX: usize = HASH_RND_PART_IDX + HASH_RND_PART_LENGTH; // 2 + 3 = 5
const PREFIX_LENGTH: usize = 3;

/// HASH_* describe the full hash as used in [overline store](
const HASH_IDX: usize = NETWORK_BYTES_IDX + NETWORK_BYTES_LENGTH; // 0 + 2 = 2
const HASH_LENGTH: usize = HASH_RND_PART_LENGTH + PREFIX_LENGTH;

const PART_NUMBER_IDX: usize = HASH_IDX + HASH_LENGTH; // 2 + 6 = 8
const PART_NUMBER_LENGTH: usize = 1;

const TOTAL_COUNT_IDX: usize = PART_NUMBER_IDX + PART_NUMBER_LENGTH; // 8 + 1 = 9
const TOTAL_COUNT_LENGTH: usize = 1;

const LENGTH_IDX: usize = TOTAL_COUNT_IDX + TOTAL_COUNT_LENGTH; // 9 + 1 = 10
const LENGTH_LENGTH: usize = 1;

pub const MSG_TYPE_IDX: usize = LENGTH_IDX + LENGTH_LENGTH; // 10 + 1 = 11;
const MSG_TYPE_LENGTH: usize = 1;

const DATA_TYPE_IDX: usize = MSG_TYPE_IDX + MSG_TYPE_LENGTH; // 11 + 1 = 12;
const DATA_TYPE_LENGTH: usize = 1;

const HEADER_LENGTH: usize = NETWORK_BYTES_LENGTH
    + HASH_LENGTH
    + PART_NUMBER_LENGTH
    + TOTAL_COUNT_LENGTH
    + LENGTH_LENGTH
    + MSG_TYPE_LENGTH
    + DATA_TYPE_LENGTH; // 13

const CRC_LENGTH: usize = 2;

#[cfg(feature = "std")]
mod lib_impl;

pub mod network {
    /// Specifies a wireless network
    pub type Network = [u8; 2];
    pub const DEFAULT: Network = [0xAA, 0xCC];
    pub const TEST: Network = [0xCC, 0xAA];
}

#[cfg(feature = "std")]
pub use lib_impl::*;

/// Part of an application level message of arbitrary length, 1-N of these form a whole message
// this is here because orouter-wireless needs std and cannot be imported as project dependency
#[cfg(feature = "no_std")]
pub type WirelessMessagePart = heapless::Vec<u8, MAX_LORA_MESSAGE_SIZE>;

/// Part of an application level message of arbitrary length, 1-N of these for a whole message
#[cfg(feature = "std")]
pub type WirelessMessagePart = Vec<u8>;

pub const MAX_LORA_MESSAGE_SIZE: usize = 255;

// FIXME define these according to the design document
/// Represents different message types which are treated differently by the oRouter hardware
#[derive(PartialEq, Clone)]
#[cfg_attr(feature = "std", derive(Debug))]
#[cfg_attr(feature = "no_std", derive(defmt::Format))]
pub enum MessageType {
    /// Common data message, no meaning for orouter
    Data,
    /// Request for a distance challenge
    Challenge,
    /// Proof of distance
    Proof,
    /// TBD
    Flush,
    Receipt,
    /// Any other message type
    Other,
}

impl From<u8> for MessageType {
    fn from(n: u8) -> Self {
        match n {
            0x01 => Self::Data,
            0x02 => Self::Challenge,
            0x03 => Self::Proof,
            0x04 => Self::Flush,
            0x05 => Self::Receipt,
            _ => Self::Other,
        }
    }
}

impl Into<u8> for MessageType {
    fn into(self) -> u8 {
        match self {
            Self::Data => 0x01,
            Self::Challenge => 0x02,
            Self::Proof => 0x03,
            Self::Flush => 0x04,
            Self::Receipt => 0x05,
            Self::Other => 0xff,
        }
    }
}

#[cfg_attr(feature = "std", derive(Debug, PartialEq))]
#[cfg_attr(feature = "no_std", derive(defmt::Format))]
pub enum ValidationError {
    LessThanMinimalLength,
    NetworkBytesMismatch,
    PartNumHigherThanTotalCount,
    IndicatedLenHigherThanMaxLen,
    IndicatedLenDifferentFromActualLen,
    // expected, actual
    IncorrectCrc(u16, u16),
}

/// validates if slice of bytes follow rules set by the protocol implementing in the crate
pub fn is_valid_message(network: network::Network, msg: &[u8]) -> Result<(), ValidationError> {
    // HEADER + 1B data + 2B CRC16
    if msg.len() < HEADER_LENGTH + 1 + CRC_LENGTH {
        return Err(ValidationError::LessThanMinimalLength);
    }

    let network_actual = &msg[NETWORK_BYTES_IDX..NETWORK_BYTES_LENGTH];
    if network_actual != network {
        return Err(ValidationError::NetworkBytesMismatch);
    }

    let len = &msg[LENGTH_IDX];
    let part_num = &msg[PART_NUMBER_IDX];
    let total_count = &msg[TOTAL_COUNT_IDX];

    if *part_num > *total_count {
        return Err(ValidationError::PartNumHigherThanTotalCount);
    }

    // max data length can be (255-header length-2B CRC)
    let max_length = crate::MAX_LORA_MESSAGE_SIZE - HEADER_LENGTH - CRC_LENGTH;
    if *len as usize > max_length {
        return Err(ValidationError::IndicatedLenHigherThanMaxLen);
    }

    // claimed len is different from actual remaining data bytes
    if *len as usize != msg.len() - HEADER_LENGTH - CRC_LENGTH {
        return Err(ValidationError::IndicatedLenDifferentFromActualLen);
    }

    // check crc
    // [0, 1, 2, 3, 4]
    let i = msg.len() - CRC_LENGTH;
    let expected_crc = &msg[i..];
    let data = &msg[..i];
    let actual_crc = State::<X_25>::calculate(data);
    let actual = actual_crc.to_be_bytes();
    eprintln!("expected = {expected_crc:02x?}, actual = {actual:02x?}");
    if actual_crc.to_be_bytes() != expected_crc {
        return Err(ValidationError::IncorrectCrc(
            u16::from_be_bytes([expected_crc[0], expected_crc[1]]),
            u16::from_be(actual_crc),
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::is_valid_message;
    use super::ValidationError;

    #[test]
    fn test_is_valid_message_missing_network_bytes() {
        //                             hash                             | num | tot | len | data
        assert_eq!(
            Err(ValidationError::NetworkBytesMismatch),
            is_valid_message(
                crate::network::DEFAULT,
                &[
                    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                    0xff, 0x04, 0x01, 0x02
                ]
            )
        );
    }

    #[test]
    fn test_is_valid_message_shorter_than_possible() {
        //                             prefix               | num | tot | len | data
        assert_eq!(
            Err(ValidationError::LessThanMinimalLength),
            is_valid_message(
                crate::network::DEFAULT,
                &[0xff, 0xff, 0xff, 0xff, 0x04, 0x01, 0x02]
            )
        );
    }

    #[test]
    fn test_is_valid_message_wrong_num() {
        assert_eq!(
            Err(ValidationError::PartNumHigherThanTotalCount),
            is_valid_message(
                crate::network::DEFAULT,
                &[
                    // network | hash                             | num | tot | len | typ  |dtyp| data
                    0xAA, 0xCC, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x02, 0x01, 0x02, 0x01, 0x01,
                    0xff, 0xff, 0x4b, 0x8c // crc16
                ]
            )
        );
    }

    #[test]
    fn test_is_valid_message_wrong_len() {
        assert_eq!(
            Err(ValidationError::IndicatedLenDifferentFromActualLen),
            is_valid_message(
                crate::network::DEFAULT,
                &[
                    // network | hash                             | num | tot | len | typ  |dtyp| data
                    0xAA, 0xCC, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01,
                    0xff, 0xff, 0xfe, 0x2e // crc16
                ],
            )
        );
    }
}
