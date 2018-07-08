use std::fmt;

use parking_lot::RwLockWriteGuard;

/// Non-instantiable type used to refer to HLE Resume Tokens from C++.
///
/// These tokens are used to unpark the HLE thread on the C++ side using
/// citra_socu_unpark.
pub enum HLEResumeToken {}
pub type HLEResumeTokenRef = &'static mut HLEResumeToken;

extern "C" {
    fn citra_socu_unpark(token: &mut HLEResumeToken);
}

impl fmt::Debug for HLEResumeTokenRef {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        write!(fmt, "HLEResumeToken")
    }
}

impl HLEResumeToken {
    /// Unparks the paused HLE thread while taking ownership of the result container's write lock.
    fn unpark(&mut self, results: RwLockWriteGuard<::SOCResultContainer>) {
        // Drop the write guard to prevent a potential deadlock when reentering through socu_consume
        drop(results);
        unsafe { citra_socu_unpark(self) }
    }
}

pub trait HLETryUnpark {
    /// Tries to unpark the paused HLE thread while taking ownership of the result container's write lock.
    fn try_unpark(self, results: RwLockWriteGuard<::SOCResultContainer>);
}

impl HLETryUnpark for Option<HLEResumeTokenRef> {
    fn try_unpark(self, results: RwLockWriteGuard<::SOCResultContainer>) {
        if let Some(token) = self {
            token.unpark(results)
        }
    }
}
