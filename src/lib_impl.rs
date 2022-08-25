//! Defines and implements protocol used on physical radio layer (now using LoRa)
//!
//! Logical [crate::overline::OverlineMessage] length can be [crate::overline::MaxLoraPayloadLength](255 B), but LoRa can only transmit 255B in
//! one message. MessageSlicer takes care of splitting message to appropriate number of parts with
//! necessary header information. On the receiving end these has to be assembled back to a logical
//! message - this is job of MessagePool
//
// TODO Remaining tasks from .plan
// * flush out unfinished messages from MessagePool after some time?
// * - else what happens when a lost of unreceived parts blocks out the Pool for newer ones
// * 7th message with 4th unmatching prefix comes in
// * parts_vec could take Option<P2pMessagePart> for nicer api and lower alloc size with resize_default

use std::collections::HashMap;

use crc::{Crc, CRC_16_IBM_SDLC};
use rand::prelude::*;

use overline_protocol::overline;

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
type RndPart = Vec<u8>;

const PREFIX_LENGTH: usize = 3;
const PREFIX_IDX: usize = HASH_RND_PART_IDX + HASH_RND_PART_LENGTH; // 2 + 3 = 5
type Prefix = Vec<u8>;

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
const MSG_TYPE_IDX: usize = LENGTH_IDX + LENGTH_LENGTH; // 10 + 1 = 11;

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

pub type P2pMessagePart = crate::WirelessMessagePart;

const MAX_P2P_MESSAGE_PART_COUNT: usize = u8::MAX as usize;
const MAX_OVERLINE_MESSAGE_LENGTH: usize = 240 * MAX_P2P_MESSAGE_PART_COUNT;
const MAX_UNFINISHED_P2P_MESSAGE_COUNT: usize = u8::MAX as usize;

/// Represents a raw p2p message constructed back from chunks
/// This has yet to be parsed into a typed [overline
/// message](../overline/enum.OverlineMessageType.html)
#[derive(Debug, PartialEq)]
pub struct P2pMessage {
    typ: overline::MessageType,
    data_type: u8,
    data: Vec<u8>,
}

impl P2pMessage {
    pub fn typ(&self) -> overline::MessageType {
        self.typ.clone()
    }

    pub fn data_type(&self) -> u8 {
        self.data_type
    }

    pub fn data(&self) -> &[u8] {
        self.data.as_slice()
    }
}

#[derive(Debug, PartialEq)]
pub enum Error {
    PoolFull,
    MalformedMessage,
    TooLong,
}

/// Holds parts of multipart messages before all parts have arrived
/// Maximum of 3 different sets of incomplete messages can be stored
///
/// Messages in vector under each prefix key are inserted at the correct index - are kept sorted
/// example of the map
/// {
///     '3B prefix': \[\[part_1\], \[part_2\], \[part3\]\], // these are ordered
///     ...
///     ...
/// }
pub struct MessagePool {
    /// Contains parts of the raw P2pMessage. Parts are stored without the prefix
    incomplete_message_map: HashMap<Prefix, Vec<P2pMessagePart>>,
    crc: Crc<u16>,
}

impl Default for MessagePool {
    fn default() -> Self {
        MessagePool {
            crc: Crc::<u16>::new(&CRC_16_IBM_SDLC),
            incomplete_message_map: Default::default(),
        }
    }
}

impl MessagePool {
    /// Try insert another part of sliced message. Will return None if this is not last (or the
    /// only) message, else it will return just the data of this message
    pub fn try_insert(&mut self, msg: P2pMessagePart) -> Result<Option<P2pMessage>, Error> {
        if !self.is_valid_message(&msg) {
            return Err(Error::MalformedMessage);
        }

        let part_num = &msg[PART_NUMBER_IDX];
        let total_count = &msg[TOTAL_COUNT_IDX];
        let len = msg[LENGTH_IDX] as usize;
        let typ = msg[MSG_TYPE_IDX];
        let data_type = msg[DATA_TYPE_IDX];

        // if this is part #1 of total count = 1, return immediately
        if *part_num == 1 as u8 && *total_count == 1 as u8 {
            return Ok(Some(P2pMessage {
                typ: typ.into(),
                data_type,
                data: msg[HEADER_LENGTH..HEADER_LENGTH + len].to_vec(),
            }));
        }

        log::trace!(
            "after part_num = {}, total_count = {}",
            part_num,
            total_count
        );

        let prefix = msg[PREFIX_IDX..PREFIX_IDX + PREFIX_LENGTH].to_vec();

        log::trace!("prefix = {:02x?}", prefix);
        // get the parts vec
        let parts_vec = match self.incomplete_message_map.get_mut(&prefix) {
            Some(parts) => parts,
            None => {
                // FIXME this should not be possible as MAX_UNFINISHED_P2P_MESSAGE_COUNT is u8::MAX
                // and total_count field is 1B
                if self.incomplete_message_map.len() == MAX_UNFINISHED_P2P_MESSAGE_COUNT as usize {
                    return Err(Error::PoolFull);
                }
                let v = vec![];
                log::trace!("inserting prefix = {:02x?}", prefix);
                //
                self.incomplete_message_map.insert(prefix.clone(), v);
                self.incomplete_message_map.get_mut(&prefix).unwrap()
            }
        };
        let parts_index = part_num - 1;
        log::trace!("parts_index = {}", parts_index);

        match parts_vec.get(parts_index as usize) {
            Some(part) if !part.is_empty() => {} // we already have this message part, not a problem
            Some(_) => {
                parts_vec[parts_index as usize] = msg[HEADER_LENGTH..HEADER_LENGTH + len].to_vec();
            }
            None => {
                // lets insert the message
                parts_vec.resize(parts_index as usize + 1, vec![]);
                parts_vec[parts_index as usize] = msg[HEADER_LENGTH..HEADER_LENGTH + len].to_vec();
            }
        }

        let has_all = parts_vec.iter().all(|current| !current.is_empty());
        let has_all = has_all && parts_vec.len() == *total_count as usize;

        if has_all {
            let mut data = vec![];
            for part in parts_vec.iter() {
                data.extend_from_slice(&part);
            }

            self.incomplete_message_map.remove(&prefix).unwrap();

            return Ok(Some(P2pMessage {
                typ: typ.into(),
                data_type,
                data,
            }));
        }

        Ok(None)
    }

    pub fn size(&self) -> u8 {
        let mut size: u8 = 0;

        for incomplete_message_parts in self.incomplete_message_map.values() {
            for part in incomplete_message_parts {
                if !part.is_empty() {
                    size += 1
                }
            }
        }

        size
    }

    pub fn reset(&mut self) {
        self.incomplete_message_map.clear();
    }

    pub(crate) fn is_valid_message(&self, msg: &[u8]) -> bool {
        // HEADER + 1B data + 2B CRC16
        if msg.len() < HEADER_LENGTH + 1 + CRC_LENGTH {
            return false;
        }
        log::trace!("after min len");

        let magic_bytes = &msg[MAGIC_BYTES_IDX..MAGIC_BYTES_LENGTH];
        if magic_bytes != MAGIC_BYTES {
            return false;
        }
        log::trace!("after magic bytes");

        let len = &msg[LENGTH_IDX];
        let part_num = &msg[PART_NUMBER_IDX];
        let total_count = &msg[TOTAL_COUNT_IDX];

        if *part_num > *total_count {
            return false;
        }
        log::trace!("after partnum > total_count check");

        // max data length can be (255-header length-2B CRC)
        let max_length = crate::MAX_LORA_MESSAGE_SIZE - HEADER_LENGTH - CRC_LENGTH;
        if *len as usize > max_length {
            return false;
        }
        log::trace!("after max len");

        // claimed len is different from actual remaining data bytes
        if *len as usize != msg.len() - HEADER_LENGTH - CRC_LENGTH {
            return false;
        }

        log::trace!("after actual len check");

        // check crc
        // [0, 1, 2, 3, 4]
        let i = msg.len() - 2;
        log::trace!("i = {}", i);
        let expected_crc = &msg[i..];
        let data = &msg[..i];
        log::trace!("expected_crc = {:02x?}, data = {:02x?}", expected_crc, data);
        let actual_crc = self.crc.checksum(data);
        if actual_crc.to_be_bytes() != expected_crc {
            log::trace!(
                "expected_crc = {:02x?}, actual_crc = {:02x?}",
                expected_crc,
                actual_crc.to_be_bytes()
            );
            return false;
        }

        true
    }

    // pub(crate) fn data_to_p2p_message(data: Vec<u8>) -> P2pMessage {}
}

/// Takes care of splitting OVerline logical chunks of data [TODO link to Overline message]() to
/// chunks transmittable using LoRa with a header allowing receiver to assemble the logical message
/// back from received parts
pub struct MessageSlicer {
    rng: SmallRng,
    crc: Crc<u16>,
}

impl MessageSlicer {
    pub fn new(initial_seed: u64) -> Self {
        MessageSlicer {
            rng: SmallRng::seed_from_u64(initial_seed),
            crc: Crc::<u16>::new(&CRC_16_IBM_SDLC),
        }
    }

    pub fn slice(
        &mut self,
        data_bytes: &[u8],
        message_type: overline::MessageType,
        data_type: u8,
    ) -> Result<Vec<P2pMessagePart>, Error> {
        // FIXME implement correct slicing - add typ, data_type, asemble hash from rnd and prefix,
        // add magic bytes
        if data_bytes.len() > MAX_OVERLINE_MESSAGE_LENGTH {
            return Err(Error::TooLong);
        }

        let mut prefix = Prefix::new();
        prefix.resize(3, 0);
        self.rng.fill(&mut prefix[0..3]);

        let mut res = vec![];
        let chunks = data_bytes.chunks(crate::MAX_LORA_MESSAGE_SIZE - HEADER_LENGTH - CRC_LENGTH);
        let total_count = chunks.len();
        let typ = message_type.into();

        for (i, part_bytes) in chunks.enumerate() {
            let mut rnd_part = RndPart::new();
            rnd_part.resize(3, 0);
            self.rng.fill(&mut rnd_part[0..3]);
            let mut p = P2pMessagePart::new();
            p.extend_from_slice(&MAGIC_BYTES);
            p.extend_from_slice(&rnd_part);
            p.extend_from_slice(&prefix);
            p.push(i as u8 + 1); // part_num
            p.push(total_count as u8); // total_count
            p.push(part_bytes.len() as u8); // length
            p.push(typ);
            p.push(data_type);
            p.extend_from_slice(&part_bytes); // data
            let crc = self.crc.checksum(p.as_slice());
            p.extend_from_slice(&crc.to_be_bytes()[..]);
            res.push(p);
        }

        Ok(res)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_pool_try_insert_1_of_1() {
        let mut p = MessagePool::default();
        let msg1 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x01, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x31, 0x02,
        ];

        let res = p.try_insert(msg1).unwrap().unwrap();
        assert_eq!(
            res,
            P2pMessage {
                typ: 0x01.into(),
                data_type: 0x01,
                data: [0xc0, 0xff, 0xee].to_vec()
            }
        );
    }

    #[test]
    fn test_pool_try_insert_2nd_of_3() {
        let mut p = MessagePool::default();
        let msg1 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x03, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x8c, 0x69,
        ];

        let res = p.try_insert(msg1).unwrap();
        assert_eq!(res, None);
    }

    #[test]
    fn test_pool_try_insert_1_and_2_of_2() {
        let mut p = MessagePool::default();
        let msg1 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x99, 0x6c,
        ];
        let msg2 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc1,
            0xff, 0xee, // crc
            0x49, 0x60,
        ];

        p.try_insert(msg1).unwrap();
        let res = p.try_insert(msg2).unwrap().unwrap();
        assert_eq!(
            res,
            P2pMessage {
                typ: 0x01.into(),
                data_type: 0x01,
                data: [0xc0, 0xff, 0xee, 0xc1, 0xff, 0xee].to_vec()
            }
        );
    }

    #[test]
    fn test_pool_try_insert_2_and_1_of_2() {
        let mut p = MessagePool::default();
        let msg1 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x99, 0x6c,
        ];
        let msg2 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc1,
            0xff, 0xee, // crc
            0x49, 0x60,
        ];

        assert_eq!(None, p.try_insert(msg2).unwrap());
        let res = p.try_insert(msg1).unwrap().unwrap();
        assert_eq!(
            res,
            P2pMessage {
                typ: 0x01.into(),
                data_type: 0x01,
                data: [0xc0, 0xff, 0xee, 0xc1, 0xff, 0xee].to_vec()
            }
        );
    }

    #[test]
    fn test_pool_size() {
        let mut p = MessagePool::default();
        assert_eq!(p.size(), 0);
        // messages with 5 different prefixes
        let msg1 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x01, 0x01, 0x01, 0xc0,
            0xd6, 0x4e,
        ];
        assert_eq!(None, p.try_insert(msg1).unwrap());
        assert_eq!(p.size(), 1);
        let msg2 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc1,
            0xff, 0xee, // crc
            0x7d, 0x2b,
        ];
        assert_eq!(None, p.try_insert(msg2).unwrap());
        assert_eq!(p.size(), 2);
        let msg1_2 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x02, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0xd0,
            0x4d, 0x5c,
        ];
        assert_eq!(
            Some(P2pMessage {
                typ: overline::MessageType::Data,
                data_type: 0x01,
                data: [0xc0, 0xd0].to_vec()
            }),
            p.try_insert(msg1_2).unwrap()
        );
        // with ^^ try_insert, msg1 was joined with msg1_2 so pool should have just one part
        assert_eq!(p.size(), 1)
    }

    #[test]
    fn test_pool_reset() {
        let mut p = MessagePool::default();
        assert_eq!(p.size(), 0);
        let msg1 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x99, 0x6c,
        ];
        assert_eq!(None, p.try_insert(msg1).unwrap());
        assert_eq!(p.size(), 1);
        let msg2 = vec![
            //   magic |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc1,
            0xff, 0xee, // crc
            0x7d, 0x2b,
        ];
        assert_eq!(None, p.try_insert(msg2.clone()).unwrap());
        assert_eq!(p.size(), 2);

        p.reset();
        assert_eq!(p.size(), 0);

        assert_eq!(None, p.try_insert(msg2).unwrap());
        assert_eq!(p.size(), 1);

        p.reset();
        assert_eq!(p.size(), 0);
    }

    #[test]
    fn test_is_valid_message_missing_magic_bytes() {
        let p = MessagePool::default();
        //                             hash                             | num | tot | len | data
        assert!(!p.is_valid_message(&[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x04, 0x01, 0x02]));
    }

    #[test]
    fn test_is_valid_message_shorter_than_possible() {
        let p = MessagePool::default();
        //                             prefix               | num | tot | len | data
        assert!(!p.is_valid_message(&[0xff, 0xff, 0xff, 0xff, 0x04, 0x01, 0x02]));
    }

    #[test]
    fn test_is_valid_message_wrong_num() {
        let p = MessagePool::default();
        assert!(!p.is_valid_message(&[
            // magic   | hash                             | num | tot | len | typ  |dtyp| data
            0xAA, 0xCC, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x02, 0x01, 0x02, 0x01, 0x01, 0xff,
            0xff, 0x4b, 0x8c // crc16
        ]));
    }

    #[test]
    fn test_is_valid_message_wrong_len() {
        let p = MessagePool::default();
        assert!(!p.is_valid_message(&[
            // magic   | hash                             | num | tot | len | typ  |dtyp| data
            0xAA, 0xCC, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x01, 0x01, 0x01, 0x01, 0xff,
            0xff, 0xfe, 0x2e // crc16
        ]));
    }

    #[test]
    fn test_slicer_single_message() {
        let mut s = MessageSlicer::new(0xdead_beef_cafe_d00d);
        let parts = s
            .slice(&[0xc0, 0xff, 0xee], overline::MessageType::Data, 0x01)
            .unwrap();
        assert_eq!(parts.len(), 1);
        assert_eq!(
            parts[0],
            //  magic b  | rnd part        |prefix            |p_n|   tot |part_l|typ|  dtyp|data ->
            &[
                0xAA, 0xCC, 0x2a, 0x73, 0x5c, 0x3c, 0xce, 0x55, 0x01, 0x01, 0x03, 0x01, 0x01, 0xc0,
                0xff, 0xee, // crc->
                0xb7, 0xd3
            ]
        );

        // try to insert if we get the same message
        let mut p = MessagePool::default();
        assert_eq!(
            Some(P2pMessage {
                typ: overline::MessageType::Data,
                data_type: 0x01,
                data: [0xc0, 0xff, 0xee].to_vec()
            }),
            p.try_insert(parts[0].clone()).unwrap()
        );
    }

    #[test]
    fn test_slicer_two_parts() {
        let mut s = MessageSlicer::new(0xdead_beef_cafe_d00d);
        let mut test_data_message = vec![];
        for b in
            core::iter::repeat(0xff).take(crate::MAX_LORA_MESSAGE_SIZE - HEADER_LENGTH - CRC_LENGTH)
        {
            test_data_message.push(b);
        }
        test_data_message.extend_from_slice(&[0xc0, 0xff, 0xee]); // this data should end up in the second part
        let parts = s
            .slice(&test_data_message, overline::MessageType::Data, 0x01)
            .unwrap();
        assert_eq!(parts.len(), 2);
        // TODO test part 1
        assert_eq!(
            parts[1],
            //  magic b  | rnd part        |prefix            |p_n|   tot |part_l|typ|  dtyp|data ->
            &[
                0xAA, 0xCC, 0x2b, 0x7a, 0xeb, 0x3c, 0xce, 0x55, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc0,
                0xff, 0xee, // crc->
                0xd1, 0x72
            ]
        );
    }

    #[test]
    fn test_slicer_too_long_data() {
        let mut s = MessageSlicer::new(0xdead_beef_cafe_d00d);
        let mut test_data_message = vec![];
        for b in core::iter::repeat(0xff).take(MAX_OVERLINE_MESSAGE_LENGTH + 1) {
            test_data_message.push(b);
        }
        assert_eq!(
            Err(Error::TooLong),
            s.slice(&test_data_message, overline::MessageType::Other, 0x01)
        );
    }
}
