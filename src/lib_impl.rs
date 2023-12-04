// TODO Remaining tasks from .plan
// * flush out unfinished messages from MessagePool after some time?
// * - else what happens when a lost of unreceived parts blocks out the Pool for newer ones
// * 7th message with 4th unmatching prefix comes in
// * parts_vec could take Option<P2pMessagePart> for nicer api and lower alloc size with resize_default

use std::collections::HashMap;

use crc16::{State, X_25};
use rand::prelude::*;

type Prefix = Vec<u8>;
type RndPart = Vec<u8>;
pub type P2pMessagePart = crate::WirelessMessagePart;

const MAX_P2P_MESSAGE_PART_COUNT: usize = u8::MAX as usize;
const MAX_OVERLINE_MESSAGE_LENGTH: usize = 240 * MAX_P2P_MESSAGE_PART_COUNT;
const MAX_UNFINISHED_P2P_MESSAGE_COUNT: usize = u8::MAX as usize;

/// Represents a raw p2p message constructed back from chunks
/// This has yet to be parsed into a typed [overline
/// message](../overline/enum.OverlineMessageType.html)
#[derive(Debug, PartialEq)]
pub struct P2pMessage {
    typ: super::MessageType,
    data_type: u8,
    data: Vec<u8>,
}

impl P2pMessage {
    pub fn typ(&self) -> super::MessageType {
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
    MalformedMessage(crate::ValidationError),
    TooLong,
}

/// Holds parts of multipart messages before all parts have arrived
///
/// Messages in vector under each prefix key are inserted at the correct index - are kept sorted
/// example of the map
///
/// ```ignore
/// {
///     '3B prefix': \[\[part_1\], \[part_2\], \[part3\]\], // these are ordered
///     ...
///     ...
/// }
/// ```
pub struct MessagePool {
    /// Contains parts of the raw P2pMessage. Parts are stored without the prefix
    incomplete_message_map: HashMap<Prefix, Vec<P2pMessagePart>>,
    network: crate::network::Network,
}

impl Default for MessagePool {
    fn default() -> Self {
        Self {
            network: crate::network::DEFAULT,
            incomplete_message_map: Default::default(),
        }
    }
}

impl MessagePool {
    /// Try insert another part of sliced message. Will return `None` if this is not last (or the
    /// only) message, else it will return just the data of this message (stripped of all the now
    /// unnecessart protocol meta data)
    pub fn try_insert(&mut self, msg: P2pMessagePart) -> Result<Option<P2pMessage>, Error> {
        if let Err(e) = crate::is_valid_message(self.network, &msg) {
            return Err(Error::MalformedMessage(e));
        }

        let part_num = &msg[crate::PART_NUMBER_IDX];
        let total_count = &msg[crate::TOTAL_COUNT_IDX];
        let len = msg[crate::LENGTH_IDX] as usize;
        let typ = msg[crate::MSG_TYPE_IDX];
        let data_type = msg[crate::DATA_TYPE_IDX];

        // if this is part #1 of total count = 1, return immediately
        if *part_num == 1 as u8 && *total_count == 1 as u8 {
            return Ok(Some(P2pMessage {
                typ: typ.into(),
                data_type,
                data: msg[crate::HEADER_LENGTH..crate::HEADER_LENGTH + len].to_vec(),
            }));
        }

        log::trace!(
            "after part_num = {}, total_count = {}",
            part_num,
            total_count
        );

        let prefix = msg[crate::PREFIX_IDX..crate::PREFIX_IDX + crate::PREFIX_LENGTH].to_vec();

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
                parts_vec[parts_index as usize] =
                    msg[crate::HEADER_LENGTH..crate::HEADER_LENGTH + len].to_vec();
            }
            None => {
                // lets insert the message
                parts_vec.resize(parts_index as usize + 1, vec![]);
                parts_vec[parts_index as usize] =
                    msg[crate::HEADER_LENGTH..crate::HEADER_LENGTH + len].to_vec();
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

    /// Returns how many message parts (regardless of which message is the part of) are currently
    /// being held by the internal state.
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

    /// Resets the internal state of the pool. This equals constructing of a new pool - all the
    /// message parts of unfinished messages inserted before calling of this method will be lost.
    pub fn reset(&mut self) {
        self.incomplete_message_map.clear();
    }

    // pub(crate) fn data_to_p2p_message(data: Vec<u8>) -> P2pMessage {}
}

/// Takes care of splitting a lengh-wise theoretically unlimited message into to
/// chunks transmittable using oRouter with a header allowing receiver to assemble the logical message
/// back from received parts (using [`crate::MessagePool`]).
pub struct MessageSlicer {
    rng: SmallRng,
    network: crate::network::Network,
}

impl MessageSlicer {
    /// `initial_seed` is a seed for rng for generating slice prefixes. Generate this using a
    /// system source of randomness
    pub fn new(initial_seed: u64, network: crate::network::Network) -> Self {
        MessageSlicer {
            rng: SmallRng::seed_from_u64(initial_seed),
            network,
        }
    }

    /// splits `data_bytes` to wireless message parts
    pub fn slice(
        &mut self,
        data_bytes: &[u8],
        message_type: super::MessageType,
        data_type: u8,
    ) -> Result<Vec<P2pMessagePart>, Error> {
        // FIXME implement correct slicing - add typ, data_type, asemble hash from rnd and prefix,
        // add network bytes
        if data_bytes.len() > MAX_OVERLINE_MESSAGE_LENGTH {
            return Err(Error::TooLong);
        }

        let mut prefix = Prefix::new();
        prefix.resize(3, 0);
        self.rng.fill(&mut prefix[0..3]);

        let mut res = vec![];
        let chunks = data_bytes
            .chunks(crate::MAX_LORA_MESSAGE_SIZE - crate::HEADER_LENGTH - crate::CRC_LENGTH);
        let total_count = chunks.len();
        let typ = message_type.into();

        for (i, part_bytes) in chunks.enumerate() {
            let mut rnd_part = RndPart::new();
            rnd_part.resize(3, 0);
            self.rng.fill(&mut rnd_part[0..3]);
            let mut p = P2pMessagePart::new();
            p.extend_from_slice(&self.network);
            p.extend_from_slice(&rnd_part);
            p.extend_from_slice(&prefix);
            p.push(i as u8 + 1); // part_num
            p.push(total_count as u8); // total_count
            p.push(part_bytes.len() as u8); // length
            p.push(typ);
            p.push(data_type);
            p.extend_from_slice(&part_bytes); // data
            let crc = State::<X_25>::calculate(p.as_slice());
            p.extend_from_slice(&crc.to_be_bytes()[..]);
            res.push(p);
        }

        Ok(res)
    }
}

#[cfg(test)]
mod tests {
    use super::super::MessageType;
    use super::*;
    #[test]
    fn test_pool_try_insert_1_of_1() {
        let mut p = MessagePool {
            network: crate::network::DEFAULT,
            ..Default::default()
        };
        let msg1 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
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
        let mut p = MessagePool {
            network: crate::network::DEFAULT,
            ..Default::default()
        };
        let msg1 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x03, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x8c, 0x69,
        ];

        let res = p.try_insert(msg1).unwrap();
        assert_eq!(res, None);
    }

    #[test]
    fn test_pool_try_insert_1_and_2_of_2() {
        let mut p = MessagePool {
            network: crate::network::DEFAULT,
            ..Default::default()
        };
        let msg1 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x99, 0x6c,
        ];
        let msg2 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
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
        let mut p = MessagePool {
            network: crate::network::TEST,
            ..Default::default()
        };
        let msg1 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xCC, 0xAA, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x16, 0xba,
        ];
        let msg2 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xCC, 0xAA, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc1,
            0xff, 0xee, // crc
            0xc6, 0xb6,
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
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x01, 0x01, 0x01, 0xc0,
            0xd6, 0x4e,
        ];
        assert_eq!(None, p.try_insert(msg1).unwrap());
        assert_eq!(p.size(), 1);
        let msg2 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc1,
            0xff, 0xee, // crc
            0x7d, 0x2b,
        ];
        assert_eq!(None, p.try_insert(msg2).unwrap());
        assert_eq!(p.size(), 2);
        let msg1_2 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x02, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0xd0,
            0x4d, 0x5c,
        ];
        assert_eq!(
            Some(P2pMessage {
                typ: MessageType::Data,
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
            // network |prefix                             | num | tot | len |typ|  dtyp| data
            0xAA, 0xCC, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x01, 0x02, 0x03, 0x01, 0x01, 0xc0,
            0xff, 0xee, // crc
            0x99, 0x6c,
        ];
        assert_eq!(None, p.try_insert(msg1).unwrap());
        assert_eq!(p.size(), 1);
        let msg2 = vec![
            // network |prefix                             | num | tot | len |typ|  dtyp| data
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
    fn test_slicer_single_message() {
        let mut s = MessageSlicer::new(0xdead_beef_cafe_d00d, crate::network::DEFAULT);
        let parts = s
            .slice(&[0xc0, 0xff, 0xee], MessageType::Data, 0x01)
            .unwrap();
        assert_eq!(parts.len(), 1);
        assert_eq!(
            parts[0],
            // network     | rnd part        |prefix            |p_n|   tot |part_l|typ|  dtyp|data ->
            &[
                0xAA, 0xCC, 0x2a, 0x73, 0x5c, 0x3c, 0xce, 0x55, 0x01, 0x01, 0x03, 0x01, 0x01, 0xc0,
                0xff, 0xee, // crc->
                0xb7, 0xd3
            ]
        );

        // try to insert if we get the same message
        let mut p = MessagePool {
            network: crate::network::DEFAULT,
            ..Default::default()
        };
        assert_eq!(
            Some(P2pMessage {
                typ: MessageType::Data,
                data_type: 0x01,
                data: [0xc0, 0xff, 0xee].to_vec()
            }),
            p.try_insert(parts[0].clone()).unwrap()
        );
    }

    #[test]
    fn test_slicer_two_parts() {
        let mut s = MessageSlicer::new(0xdead_beef_cafe_d00d, crate::network::TEST);
        let mut test_data_message = vec![];
        for b in core::iter::repeat(0xff)
            .take(crate::MAX_LORA_MESSAGE_SIZE - crate::HEADER_LENGTH - crate::CRC_LENGTH)
        {
            test_data_message.push(b);
        }
        test_data_message.extend_from_slice(&[0xc0, 0xff, 0xee]); // this data should end up in the second part
        let parts = s
            .slice(&test_data_message, MessageType::Data, 0x01)
            .unwrap();
        assert_eq!(parts.len(), 2);
        // TODO test part 1
        assert_eq!(
            parts[1],
            //  network b  | rnd part        |prefix            |p_n|   tot |part_l|typ|  dtyp|data ->
            &[
                0xCC, 0xAA, 0x2b, 0x7a, 0xeb, 0x3c, 0xce, 0x55, 0x02, 0x02, 0x03, 0x01, 0x01, 0xc0,
                0xff, 0xee, // crc->
                0x5e, 0xa4
            ]
        );
    }

    #[test]
    fn test_slicer_too_long_data() {
        let mut s = MessageSlicer::new(0xdead_beef_cafe_d00d, crate::network::DEFAULT);
        let mut test_data_message = vec![];
        for b in core::iter::repeat(0xff).take(MAX_OVERLINE_MESSAGE_LENGTH + 1) {
            test_data_message.push(b);
        }
        assert_eq!(
            Err(Error::TooLong),
            s.slice(&test_data_message, MessageType::Other, 0x01)
        );
    }
}
