#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
mod lib_impl;

#[cfg(feature = "std")]
pub use lib_impl::*;

pub const MAX_LORA_MESSAGE_SIZE: usize = 255;

// this is here because wireless-protocol needs std and cannot be imported as project dependency
#[cfg(feature = "no_std")]
pub type WirelessMessagePart = heapless::Vec<u8, MAX_LORA_MESSAGE_SIZE>;

#[cfg(feature = "std")]
pub type WirelessMessagePart = Vec<u8>;
