#![cfg_attr(not(feature = "std"), no_std)]

use crc16::{State, X_25};

/// crate::WirelessMessagePart represents raw chunk of data received using radio chip.
/// It uses following structure:
///
/// | name        | length in bytes | description                                            |
/// |--:-:--------|--:-:------------|--:-:---------------------------------------------------|
/// | magic bytes | 2               | magic bytes. always 0xAA 0xCC                          |
/// | hash        | 6               | hash - first 3B are random, second 3B form a prefix grouping parts of the message to one            |
/// | part_num    | 1               | part number 1, 2 or 3 (only 3-part messages supported) |
/// | total_count | 1               | total count of messages with this prefix               |
/// | length      | 1               | length of data                                         |
/// | msg type    | 1               | overline message type                                  |
/// | data type   | 1               | byte identifying data type, if previous field is data  |
/// | data        | 1 - 240         | actual data                                            |
/// | CRC16       | 2               | CRC16 of the whole message (header + data)             |

const MAGIC_BYTES: [u8; 2] = [0xAA, 0xCC];
const MAGIC_BYTES_LENGTH: usize = MAGIC_BYTES.len();
const MAGIC_BYTES_IDX: usize = 0;

const HASH_RND_PART_LENGTH: usize = 3;
const HASH_RND_PART_IDX: usize = MAGIC_BYTES_IDX + MAGIC_BYTES_LENGTH; // 0 + 2 = 2

const PREFIX_LENGTH: usize = 3;
const PREFIX_IDX: usize = HASH_RND_PART_IDX + HASH_RND_PART_LENGTH; // 2 + 3 = 5

/// HASH_* describe the full hash as used in [overline store](
const HASH_LENGTH: usize = HASH_RND_PART_LENGTH + PREFIX_LENGTH;
const HASH_IDX: usize = MAGIC_BYTES_IDX + MAGIC_BYTES_LENGTH; // 0 + 2 = 2

const PART_NUMBER_LENGTH: usize = 1;
const PART_NUMBER_IDX: usize = HASH_IDX + HASH_LENGTH; // 2 + 6 = 8

const TOTAL_COUNT_LENGTH: usize = 1;
const TOTAL_COUNT_IDX: usize = PART_NUMBER_IDX + PART_NUMBER_LENGTH; // 8 + 1 = 9

const LENGTH_LENGTH: usize = 1;
const LENGTH_IDX: usize = TOTAL_COUNT_IDX + TOTAL_COUNT_LENGTH; // 9 + 1 = 10

const MSG_TYPE_LENGTH: usize = 1;
pub const MSG_TYPE_IDX: usize = LENGTH_IDX + LENGTH_LENGTH; // 10 + 1 = 11;

const DATA_TYPE_LENGTH: usize = 1;
const DATA_TYPE_IDX: usize = MSG_TYPE_IDX + MSG_TYPE_LENGTH; // 11 + 1 = 12;

const HEADER_LENGTH: usize = MAGIC_BYTES_LENGTH
    + HASH_LENGTH
    + PART_NUMBER_LENGTH
    + TOTAL_COUNT_LENGTH
    + LENGTH_LENGTH
    + MSG_TYPE_LENGTH
    + DATA_TYPE_LENGTH; // 13

const CRC_LENGTH: usize = 2;

#[cfg(feature = "std")]
mod lib_impl;

#[cfg(feature = "std")]
pub use lib_impl::*;

// this is here because wireless-protocol needs std and cannot be imported as project dependency
#[cfg(feature = "no_std")]
pub type WirelessMessagePart = heapless::Vec<u8, MAX_LORA_MESSAGE_SIZE>;

#[cfg(feature = "std")]
pub type WirelessMessagePart = Vec<u8>;

pub const MAX_LORA_MESSAGE_SIZE: usize = 255;

// FIXME define these according to the design document
#[derive(PartialEq, Clone)]
#[cfg_attr(feature = "std", derive(Debug))]
#[cfg_attr(feature = "no_std", derive(defmt::Format))]
pub enum MessageType {
    Data,
    Challenge,
    Proof,
    Flush,
    Receipt,
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
    MagicBytesMismatch,
    PartNumHigherThanTotalCount,
    IndicatedLenHigherThanMaxLen,
    IndicatedLenDifferentFromActualLen,
    // expected, actual
    IncorrectCrc(u16, u16),
}

pub fn is_valid_message(msg: &[u8]) -> Result<(), ValidationError> {
    // HEADER + 1B data + 2B CRC16
    if msg.len() < HEADER_LENGTH + 1 + CRC_LENGTH {
        return Err(ValidationError::LessThanMinimalLength);
    }

    let magic_bytes = &msg[MAGIC_BYTES_IDX..MAGIC_BYTES_LENGTH];
    if magic_bytes != MAGIC_BYTES {
        return Err(ValidationError::MagicBytesMismatch);
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
    let i = msg.len() - 2;
    let expected_crc = &msg[i..];
    let data = &msg[..i];
    let actual_crc = State::<X_25>::calculate(data);
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
    fn test_is_valid_message_missing_magic_bytes() {
        //                             hash                             | num | tot | len | data
        assert_eq!(
            Err(ValidationError::MagicBytesMismatch),
            is_valid_message(&[
                0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                0x04, 0x01, 0x02
            ])
        );
    }

    #[test]
    fn test_is_valid_message_shorter_than_possible() {
        //                             prefix               | num | tot | len | data
        assert_eq!(
            Err(ValidationError::LessThanMinimalLength),
            is_valid_message(&[0xff, 0xff, 0xff, 0xff, 0x04, 0x01, 0x02])
        );
    }

    #[test]
    fn test_is_valid_message_wrong_num() {
        assert_eq!(
            Err(ValidationError::PartNumHigherThanTotalCount),
            is_valid_message(&[
                // magic   | hash                             | num | tot | len | typ  |dtyp| data
                0xAA, 0xCC, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x02, 0x01, 0x02, 0x01, 0x01, 0xff,
                0xff, 0x4b, 0x8c // crc16
            ])
        );
    }

    #[test]
    fn test_is_valid_message_wrong_len() {
        assert_eq!(
            Err(ValidationError::IndicatedLenDifferentFromActualLen),
            is_valid_message(&[
                // magic   | hash                             | num | tot | len | typ  |dtyp| data
                0xAA, 0xCC, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0xff,
                0xff, 0xfe, 0x2e // crc16
            ])
        );
    }
}
