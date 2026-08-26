//! Logging macros that compile away without a logging backend.
//!
//! `log` itself is `no_std`-compatible, so this is not about portability: it is
//! about not forcing a facade on a firmware target that already has one. An
//! embedded consumer typically wants `defmt` and nothing else, and a `log`
//! dependency it cannot switch off is a dependency it has to justify.
//!
//! Three arms, in order of preference: `log` if enabled, otherwise `defmt` if
//! enabled, otherwise nothing.
//!
//! With neither feature the macros still expand to `format_args!`, so the
//! format string and its arguments are type-checked and a broken `trace!` is
//! caught in every configuration rather than only in the ones that log. The
//! arguments are therefore still *evaluated*; keep anything expensive out of
//! them and log whole values rather than computing summaries inline.
//!
//! Format strings must be valid for both backends, which in practice means
//! plain `{}`. `defmt` does not accept `core::fmt` specifiers such as `{:02X?}`,
//! so byte slices go through [`Hex`], which renders the same way either way.

#[cfg(feature = "log")]
macro_rules! trace {
	($($arg:tt)*) => { log::trace!($($arg)*) };
}

#[cfg(all(feature = "defmt", not(feature = "log")))]
macro_rules! trace {
	($($arg:tt)*) => { defmt::trace!($($arg)*) };
}

#[cfg(not(any(feature = "log", feature = "defmt")))]
macro_rules! trace {
	($($arg:tt)*) => {
		let _ = format_args!($($arg)*);
	};
}

#[cfg(feature = "log")]
macro_rules! debug {
	($($arg:tt)*) => { log::debug!($($arg)*) };
}

#[cfg(all(feature = "defmt", not(feature = "log")))]
macro_rules! debug {
	($($arg:tt)*) => { defmt::debug!($($arg)*) };
}

#[cfg(not(any(feature = "log", feature = "defmt")))]
macro_rules! debug {
	($($arg:tt)*) => {
		let _ = format_args!($($arg)*);
	};
}

/// A byte slice rendered as uppercase hex, for either logging backend.
///
/// `{:02X?}` is a `core::fmt` specifier and `defmt` rejects it, so the
/// formatting lives in a type that both backends can render through a plain
/// `{}` instead of in the format string.
pub(crate) struct Hex<'a>(pub &'a [u8]);

impl core::fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("[")?;
        for (index, byte) in self.0.iter().enumerate() {
            if index > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{byte:02X}")?;
        }
        f.write_str("]")
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Hex<'_> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{=[u8]:02X}", self.0)
    }
}

pub(crate) use {debug, trace};
