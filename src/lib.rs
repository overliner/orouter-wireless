#![cfg_attr(not(feature = "std"), no_std)]

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
