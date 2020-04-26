use core::{
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    ptr, slice,
};
use generic_array::{typenum::Unsigned, ArrayLength, GenericArray};
use zerocopy::{AsBytes, FromBytes};

/// Represents the payload from ESB protocol, its maximum length is determined by
/// the generic param N. The protocol supports a payload with a maximum of 252 bytes.
pub struct Payload<N: ArrayLength<MaybeUninit<u8>> + Unsigned> {
    pub(crate) buf: GenericArray<MaybeUninit<u8>, N>,
    pub(crate) len: u8,
    // Should we take care of the pid or leave it to the user ?
    pub(crate) pid_and_no_ack: u8,
    pub(crate) pipe: u8,
}

impl<N: ArrayLength<MaybeUninit<u8>> + Unsigned> Payload<N> {
    /// Creates a payload from a slice, copying the data up to the maximum size of the payload.
    ///
    /// # Panics
    ///
    /// This function will panic if the generic param `N` (payload length) is bigger than 252 bytes
    /// or if the `pid` is bigger than 3.
    pub fn copy_from_slice(data: &[u8], pid: u8, no_ack: bool, pipe: u8) -> Self {
        assert!(N::USIZE < 253 && pid < 4);
        // NOTE(unsafe) Generic Array is the same as a [T; N] in memory
        #[allow(clippy::uninit_assumed_init)]
        let mut buf: GenericArray<MaybeUninit<u8>, N> =
            unsafe { MaybeUninit::uninit().assume_init() };
        let count = data.len().min(N::USIZE);

        // NOTE(unsafe) `&[MaybeUninit<u8>]` behaves like `&[u8]` and we also checked for the size
        unsafe {
            ptr::copy_nonoverlapping(
                data.as_ptr(),
                buf.as_mut_slice().as_mut_ptr() as *mut u8,
                count,
            );
        }
        Self {
            buf,
            len: count as u8,
            pid_and_no_ack: (pid << 1) | if no_ack { 0x00 } else { 0x01 },
            pipe,
        }
    }
}

impl<N: ArrayLength<MaybeUninit<u8>> + Unsigned> Deref for Payload<N> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        // NOTE(unsafe) Safe as it uses the internal length of valid data
        unsafe {
            slice::from_raw_parts(self.buf.as_slice().as_ptr() as *const _, self.len as usize)
        }
    }
}

impl<N: ArrayLength<MaybeUninit<u8>> + Unsigned> DerefMut for Payload<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // NOTE(unsafe) Safe as it uses the internal length of valid data
        unsafe {
            slice::from_raw_parts_mut(
                self.buf.as_mut_slice().as_mut_ptr() as *mut _,
                self.len as usize,
            )
        }
    }
}

/// Addresses used for communication.
/// ESB uses up to eight pipes to address communication, each pipe has an unique address which is
/// composed by the base address and the prefix. Pipe 0 has an unique base and prefix, while the
/// other pipes share a base address but have different prefixes.
pub struct Addresses {
    pub(crate) base0: [u8; 4],
    pub(crate) base1: [u8; 4],
    pub(crate) prefixes0: [u8; 4],
    pub(crate) prefixes1: [u8; 4],
    pub(crate) rf_channel: u8,
}

impl Addresses {
    /// Creates a new instance of `Addresses`
    ///
    /// * `base0` - Base address for pipe 0.
    /// * `base1` - Base address for pipe 1-7.
    /// * `prefixes0` - Prefixes for pipes 0-3, in order.
    /// * `prefixes1` - Prefixes for pipes 4-7, in order.
    /// * `rf_channel` - Channel to be used by the radio hardware (must be between 0 and 100).
    ///
    /// # Panics
    ///
    /// This function will panic if `rf_channel` is bigger than 100.
    pub fn new(
        base0: [u8; 4],
        base1: [u8; 4],
        prefixes0: [u8; 4],
        prefixes1: [u8; 4],
        rf_channel: u8,
    ) -> Self {
        assert!(rf_channel <= 100);
        Self {
            base0,
            base1,
            prefixes0,
            prefixes1,
            rf_channel,
        }
    }
}

/// Type to be used by the ESB stack to perform radio transfers.
pub struct Buffer<N: ArrayLength<u8> + Unsigned> {
    pub(crate) inner: GenericArray<u8, N>,
}

impl<N: ArrayLength<u8> + Unsigned> Buffer<N> {
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        Self {
            inner: GenericArray::default(),
        }
    }
}

impl<N: ArrayLength<u8> + Unsigned> Deref for Buffer<N> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.inner.as_slice()
    }
}

impl<N: ArrayLength<u8> + Unsigned> DerefMut for Buffer<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.as_mut_slice()
    }
}

/*
#[derive(AsBytes, FromBytes)]
#[repr(C)]
pub struct Pcf {
    payload_length: u8,
    pid_noack: u8,
}

impl Pcf {
    pub fn new(pid: u8, no_ack: bool) -> Self {
        Self {
            // This will be changed later by the stack
            payload_length: 0,
            pid_noack: pid << 1 | if no_ack { 0x00 } else { 0x01 },
        }
    }

    pub fn pid(&self) -> u8 {
        self.pid_noack >> 1
    }

    pub fn noack(&self) -> bool {
        // This is weird, Nordic uses the `no-ack` bit as "active-low"
        self.pid_noack & 0x01 != 0x01
    }
}
*/
