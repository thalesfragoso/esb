use crate::Error;
use bbqueue::framed::{FrameGrantR, FrameGrantW};
use core::ops::{Deref, DerefMut};

// | SW USE                        |               ACTUAL DMA PART                                    |
// | rssi - 1 byte | pipe - 1 byte | length - 1 byte | pid_no_ack - 1 byte | payload - 1 to 252 bytes |

/// A builder for an `EsbHeader` structure
///
/// The builder is converted into an `EsbHeader` by calling the
/// `check()` method.
///
/// ## Example
///
/// ```rust
/// use esb::EsbHeaderBuilder;
///
/// let header_result = EsbHeaderBuilder::default()
///     .max_payload(252)
///     .pid(0)
///     .pipe(0)
///     .no_ack(true)
///     .check();
///
/// assert!(header_result.is_ok());
/// ```
///
/// ## Default Header Contents
///
/// By default, the following settings will be used:
///
/// | Field     | Default Value |
/// | :---      | :---          |
/// | pid       | 0             |
/// | no_ack    | true          |
/// | length    | 0             |
/// | pipe      | 0             |
///
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct EsbHeaderBuilder(EsbHeader);

impl Default for EsbHeaderBuilder {
    fn default() -> Self {
        EsbHeaderBuilder(EsbHeader {
            rssi: 0,
            pid_no_ack: 0,
            length: 0,
            pipe: 0,
        })
    }
}

impl EsbHeaderBuilder {
    /// Set the pipe. Must be in the range 0..=7.
    pub fn pipe(mut self, pipe: u8) -> Self {
        self.0.pipe = pipe;
        self
    }

    /// Set the max payload. Must be in the range 0..=252.
    pub fn max_payload(mut self, max_payload: u8) -> Self {
        self.0.length = max_payload;
        self
    }

    /// Enable/disable acknowledgment
    pub fn no_ack(mut self, no_ack: bool) -> Self {
        // TODO(AJM): We should probably just call this
        // method "ack", or "enable_ack", because "no_ack"
        // is really confusing.
        if no_ack {
            self.0.pid_no_ack &= 0b1111_1110;
        } else {
            self.0.pid_no_ack |= 0b0000_0001;
        }
        self
    }

    /// Set the pid. Must be in the range 0..=3.
    pub fn pid(mut self, pid: u8) -> Self {
        // TODO(AJM): Do we want the user to set the pid? isn't this an
        // internal "retry" counter?
        self.0.pid_no_ack &= 0b0000_0001;
        self.0.pid_no_ack |= pid << 1;
        self
    }

    /// Finalize the header.
    ///
    /// If the set parameters are out of range, an error will be returned.
    pub fn check(self) -> Result<EsbHeader, Error> {
        let bad_length = self.0.length > 252;
        let bad_pipe = self.0.pipe > 7;

        // This checks if "pid" > 3, where pid_no_ack is pid << 1.
        let bad_pid = self.0.pid_no_ack > 0b0000_0111;

        if bad_length || bad_pid || bad_pipe {
            return Err(Error::InvalidParameters);
        }

        Ok(self.0)
    }
}

/// The non-payload portion of an ESB packet
///
/// This is typically used to create a Packet Grant using methods
/// from the [`EsbApp`](struct.EsbApp.html) or [`EsbIrq`](struct.EsbIrq.html)
/// interfaces.
///
/// ## Example
///
/// ```rust
/// use esb::EsbHeader;
///
/// let builder_result = EsbHeader::build()
///     .max_payload(252)
///     .pid(0)
///     .pipe(1)
///     .no_ack(true)
///     .check();
///
/// let new_result = EsbHeader::new(
///     252,
///     0,
///     1,
///     true,
/// );
///
/// assert_eq!(builder_result, new_result);
/// ```
///
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub struct EsbHeader {
    rssi: u8,
    // TODO(AJM): We can probably combine the 3 bits of pipe
    // into the pid_no_ack field to save another byte of space.
    // We just need to mask it out in EsbIrq before handing it
    // to the radio to process.
    pipe: u8,
    pub(crate) length: u8,
    pid_no_ack: u8,
}

/// The "packed" representation of an [`EsbHeader`]
pub(crate) struct HeaderBytes(pub(crate) [u8; 4]);

impl EsbHeader {
    /// Create a new packet header using a builder pattern
    ///
    /// See the docs for [`EsbBuilder`](struct.EsbHeaderBuilder.html) for more
    /// information.
    pub fn build() -> EsbHeaderBuilder {
        EsbHeaderBuilder::default()
    }

    /// Create a new packet header
    ///
    /// Notes on valid values:
    ///
    /// * `max_payload_length` must be between 0 and 252 bytes, inclusive.
    /// * `pid` must be between 0 and 3, inclusive.
    /// * `pipe` must be between 0 and 7, inclusive.
    pub fn new(max_payload_length: u8, pid: u8, pipe: u8, no_ack: bool) -> Result<Self, Error> {
        EsbHeaderBuilder::default()
            .max_payload(max_payload_length)
            .pid(pid)
            .pipe(pipe)
            .no_ack(no_ack)
            .check()
    }

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
    pub fn pid(self) -> u8 {
        self.pid_no_ack >> 1
    }

    /// Accessor for the no-ack field of the packet
    pub fn no_ack(self) -> bool {
        self.pid_no_ack & 1 != 1
    }

    /// Accessor for the length (in bytes) of the payload
    pub fn payload_len(self) -> usize {
        usize::from(self.length)
    }

    /// Accessor for the rssi of the payload
    pub fn rssi(self) -> u8 {
        self.rssi
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
pub struct PayloadR<const N: usize> {
    grant: FrameGrantR<'static, N>,
}

impl<const N: usize> PayloadR<N> {
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

    /// Utility method to use with the CCM peripheral present in Nordic's devices. This gives a
    /// slice starting from the pipe field of the header.
    pub fn ccm_slice(&self) -> &[u8] {
        &self.grant[EsbHeader::pipe_idx()..]
    }

    /// An accessor function for the pipe of the current grant
    pub fn pipe(&self) -> u8 {
        self.grant[EsbHeader::pipe_idx()]
    }

    /// An accessor function to get the pipe id of the current grant
    pub fn pid(&self) -> u8 {
        self.grant[EsbHeader::pid_no_ack_idx()] >> 1
    }

    /// An accessor function for the no-ack field of the current grant
    pub fn no_ack(&self) -> bool {
        self.grant[EsbHeader::pid_no_ack_idx()] & 1 != 1
    }

    /// An accessor function to get the size of the payload of the current grant
    pub fn payload_len(&self) -> usize {
        self.grant[EsbHeader::length_idx()] as usize
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

    /// Set whether the payload should automatically release on drop
    #[inline(always)]
    pub fn auto_release(&mut self, is_auto: bool) {
        self.grant.auto_release(is_auto);
    }
}

impl<const N: usize> Deref for PayloadR<N> {
    type Target = [u8];

    /// Provide read only access to the payload of a grant
    fn deref(&self) -> &Self::Target {
        &self.grant[EsbHeader::header_size()..]
    }
}

impl<const N: usize> DerefMut for PayloadR<N> {
    /// provide read/write access to the payload portion of the grant
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.grant[EsbHeader::header_size()..]
    }
}

pub struct PayloadW<const N: usize> {
    grant: FrameGrantW<'static, N>,
}

impl<const N: usize> PayloadW<N> {
    /// Update the header contained within this grant.
    ///
    /// This can be used to modify the pipe, length, etc. of the
    /// packet.
    ///
    /// ## NOTE:
    ///
    /// The `length` of the packet can not be increased, only shrunk. If a larger
    /// payload is needed, you must drop the current payload grant, and obtain a new
    /// one. If the new header has a larger `length` than the current `length`, then
    /// it will be truncated.
    pub fn update_header(&mut self, mut header: EsbHeader) {
        // TODO(AJM): Technically, we could drop the current grant, and request a larger one
        // here, and it would totally work. However for now, let's just truncate, because growing
        // the buffer would first have to be implemented in BBQueue.

        // `length` must always be 0..=252 (checked by constructor), so `u8` cast is
        // appropriate here
        let payload_max = self.grant.len().saturating_sub(EsbHeader::header_size());
        header.length = header.length.min(payload_max as u8);
        self.grant[..EsbHeader::header_size()].copy_from_slice(&header.into_bytes().0);
    }

    /// Utility method to use with the CCM peripheral present in Nordic's devices. This gives a
    /// slice starting from the pipe field of the header.
    ///
    /// # Safety
    ///
    /// This gives raw mutable access to the header part of the packet, modification on these parts
    /// are not checked, the user must ensure that:
    ///
    /// * The pipe field remains in a valid range.
    /// * The pid_no_ack remains valid and accepted by the esb protocol.
    /// * The length field remains smaller or equal than the length used when requesting this
    ///   particular PayloadW.
    ///
    /// This method is not a recommended way to update the header, it is here to provide a way to
    /// use this abstraction with the CCM hardware peripheral.
    /// The length field present in this slice contains the payload length requested during the
    /// creation of this type, that is, the maximum payload size that this particular grant can
    /// contain. When using this slice to store the output of the CCM operation, the CCM peripheral
    /// will modify this field, the user must ensure that this field remains in a valid range.
    pub unsafe fn ccm_slice(&mut self) -> &mut [u8] {
        &mut self.grant[EsbHeader::pipe_idx()..]
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

    /// An accessor function to get the pipe id of the current grant
    pub fn pid(&self) -> u8 {
        self.grant[EsbHeader::pid_no_ack_idx()] >> 1
    }

    /// An accessor function for the no-ack field of the current grant
    pub fn no_ack(&self) -> bool {
        self.grant[EsbHeader::pid_no_ack_idx()] & 1 != 1
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

    /// Set the amount to automatically commit on drop
    ///
    /// If `None` is given, then the packet will not be commited. If `Some(0)`
    /// is given, then an empty packet will be committed automatically
    pub fn to_commit(&mut self, amt: Option<usize>) {
        if let Some(amt) = amt {
            let payload_max = self.grant.len().saturating_sub(EsbHeader::header_size());
            let payload_len = payload_max.min(amt);
            self.grant[EsbHeader::length_idx()] = payload_len as u8;
            self.grant.to_commit(payload_len + EsbHeader::header_size());
        } else {
            self.grant.to_commit(0);
        }
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

impl<const N: usize> Deref for PayloadW<N> {
    type Target = [u8];

    /// provide read only access to the payload portion of the grant
    fn deref(&self) -> &Self::Target {
        &self.grant[EsbHeader::header_size()..]
    }
}

impl<const N: usize> DerefMut for PayloadW<N> {
    /// provide read/write access to the payload portion of the grant
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.grant[EsbHeader::header_size()..]
    }
}
