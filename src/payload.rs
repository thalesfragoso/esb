use bbqueue::{
    framed::{FrameGrantR, FrameGrantW},
    ArrayLength,
};
use core::ops::{Deref, DerefMut};

// | SW USE                        |               ACTUAL DMA PART                                    |
// | rssi - 1 byte | pipe - 1 byte | length - 1 byte | pid_no_ack - 1 byte | payload - 1 to 252 bytes |

/// The non-payload portion of an ESB packet
pub struct EsbHeader {
    rssi: u8,
    pipe: u8,
    length: u8,
    pid_no_ack: u8,
}

/// The "packed" representation of an [`EsbHeader`]
pub(crate) struct HeaderBytes(pub(crate) [u8; 4]);

impl EsbHeader {
    /// convert into a packed representation meant for internal
    /// data queuing purposes
    fn into_bytes(self) -> HeaderBytes {
        HeaderBytes([
            self.rssi,
            self.pipe,
            // DO NOT REORDER!
            self.length,
            self.pid_no_ack,
        ])
    }

    /// convert from a packed representation
    pub(crate) fn from_bytes(bytes: HeaderBytes) -> Self {
        Self {
            rssi: bytes.0[Self::rssi_idx()],
            pipe: bytes.0[Self::pipe_idx()],
            length: bytes.0[Self::length_idx()],
            pid_no_ack: bytes.0[Self::pid_no_ack_idx()],
        }
    }

    /// Accessor for the Pipe ID of the packet
    pub fn pid(&self) -> u8 {
        self.pid_no_ack >> 1
    }

    /// Accessor for the no-ack field of the packet
    pub fn no_ack(&self) -> bool {
        self.pid_no_ack & 1 != 1
    }

    /// Accessor for the length (in bytes) of the payload
    pub fn payload_len(&self) -> usize {
        usize::from(self.length)
    }

    /// Byte index of the RSSI field
    const fn rssi_idx() -> usize {
        0
    }

    /// Byte index of the pipe field
    const fn pipe_idx() -> usize {
        1
    }

    /// Byte index of the payload length field
    const fn length_idx() -> usize {
        // DO NOT CHANGE! HW DEPENDANT
        2
    }

    /// Byte index of the pid_no_ack field
    const fn pid_no_ack_idx() -> usize {
        // DO NOT CHANGE! HW DEPENDANT
        3
    }

    /// Size of the header (packed) in bytes
    pub(crate) const fn header_size() -> usize {
        core::mem::size_of::<HeaderBytes>()
    }

    /// Offset of the bytes needed for DMA processing
    const fn dma_payload_offset() -> usize {
        2
    }
}

/// A handle representing a grant of a readable packet
///
/// This exposes the bytes of a payload that have either
/// been sent FROM the app, and are being read by the RADIO,
/// or a payload that has send FROM the Radio, and is being
/// read by the app
pub struct PayloadR<N: ArrayLength<u8>> {
    grant: FrameGrantR<'static, N>,
}

impl<N> PayloadR<N>
where
    N: ArrayLength<u8>,
{
    /// Create a wrapped Payload Grant from a raw BBQueue Framed Grant
    pub(crate) fn new(raw_grant: FrameGrantR<'static, N>) -> Self {
        Self { grant: raw_grant }
    }

    /// Obtain a copy of the header encoded in the current grant
    pub fn get_header(&self) -> EsbHeader {
        const LEN: usize = EsbHeader::header_size();
        let mut bytes = [0u8; LEN];
        bytes.copy_from_slice(&self.grant[..LEN]);
        EsbHeader::from_bytes(HeaderBytes(bytes))
    }

    /// Obtain a pointer to the data to provide to the RADIO DMA.
    ///
    /// This includes part of the header, as well as the full payload
    pub(crate) fn dma_pointer(&self) -> *const u8 {
        (&self.grant[EsbHeader::dma_payload_offset()..]).as_ptr()
    }

    /// An accessor function for the pipe of the current Payload
    pub fn pipe(&self) -> u8 {
        self.grant[EsbHeader::pipe_idx()]
    }

    /// This function marks the packet as read, and restores the space
    /// in the buffer for re-use.
    ///
    /// If this function is NOT explicitly called (e.g. the grant is just)
    /// implicitly dropped. The packet will not be released, and the next
    /// PayloadR grant will contain the *same* packet.
    pub fn release(self) {
        self.grant.release()
    }
}

impl<N> Deref for PayloadR<N>
where
    N: ArrayLength<u8>,
{
    type Target = [u8];

    /// Provide read only access to the payload of a grant
    fn deref(&self) -> &Self::Target {
        &self.grant[EsbHeader::header_size()..]
    }
}

pub struct PayloadW<N: ArrayLength<u8>> {
    grant: FrameGrantW<'static, N>,
}

impl<N> PayloadW<N>
where
    N: ArrayLength<u8>,
{
    /// Update the header contained within this grant.
    ///
    /// This can be used to modify the pipe, length, etc. of the
    /// packet.
    pub fn update_header(&mut self, header: EsbHeader) {
        self.grant[..EsbHeader::header_size()].copy_from_slice(&header.into_bytes().0);
    }

    /// Obtain a writable grant from the application side.
    ///
    /// This method should only be used from within `EsbApp`.
    pub(crate) fn new_from_app(mut raw_grant: FrameGrantW<'static, N>, header: EsbHeader) -> Self {
        raw_grant[..EsbHeader::header_size()].copy_from_slice(&header.into_bytes().0);
        Self { grant: raw_grant }
    }

    /// Obtain a writable grant from the RADIO/interrupt side.
    ///
    /// This method should only be used from within `EsbIrq`.
    pub(crate) fn new_from_radio(raw_grant: FrameGrantW<'static, N>) -> Self {
        Self { grant: raw_grant }
    }

    pub(crate) fn dma_pointer(&mut self) -> *mut u8 {
        (&mut self.grant[EsbHeader::dma_payload_offset()..]).as_mut_ptr()
    }

    /// Update the pipe field.
    ///
    /// Pipe must be between 0 and 7, inclusive.
    #[inline]
    pub(crate) fn set_pipe(&mut self, pipe: u8) {
        self.grant[EsbHeader::pipe_idx()] = pipe;
    }

    /// Update the rssi field.
    #[inline]
    pub(crate) fn set_rssi(&mut self, rssi: u8) {
        self.grant[EsbHeader::rssi_idx()] = rssi;
    }

    /// An accessor function to get the pipe id of the current grant
    pub fn pipe(&self) -> u8 {
        self.grant[EsbHeader::pipe_idx()]
    }

    /// An accessor function to get the maximum size of the payload of the current grant
    pub fn payload_len(&self) -> usize {
        self.grant[EsbHeader::length_idx()] as usize
    }

    /// Commit the entire granted packet and payload
    ///
    /// If this function or `commit` are not explicitly called, e.g.
    /// the `PayloadW` is implicitly dropped, the packet will not be
    /// sent.
    pub fn commit_all(self) {
        let payload_len = self.payload_len();
        self.grant.commit(payload_len + EsbHeader::header_size())
    }

    /// Commit the packed, including the first `used` bytes of the payload
    ///
    /// If this function or `commit_all` are not explicitly called, e.g.
    /// the `PayloadW` is implicitly dropped, the packet will not be
    /// sent.
    ///
    /// If `used` is larger than the maximum size of the grant (or of the
    /// ESB protocol), the packet will be truncated.
    pub fn commit(mut self, used: usize) {
        let payload_len = self.payload_len().min(used);
        self.grant[EsbHeader::length_idx()] = payload_len as u8;

        self.grant.commit(payload_len + EsbHeader::header_size())
    }
}

impl<N> Deref for PayloadW<N>
where
    N: ArrayLength<u8>,
{
    type Target = [u8];

    /// provide read only access to the payload portion of the grant
    fn deref(&self) -> &Self::Target {
        &self.grant[EsbHeader::header_size()..]
    }
}

impl<N> DerefMut for PayloadW<N>
where
    N: ArrayLength<u8>,
{
    /// provide read/write access to the payload portion of the grant
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.grant[EsbHeader::header_size()..]
    }
}
