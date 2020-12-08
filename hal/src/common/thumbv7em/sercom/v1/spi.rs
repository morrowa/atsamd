use crate::clock;
use crate::hal::spi::{FullDuplex, Mode, Phase, Polarity};
use crate::sercom::pads::*;
use crate::spi_common::CommonSpi;
use crate::target_device::sercom0::SPIM;
use crate::target_device::{MCLK, SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5};
#[cfg(feature = "min-samd51n")]
use crate::target_device::{SERCOM6, SERCOM7};
use crate::time::Hertz;
use crate::typelevel::*;
use core::any::Any;

#[derive(Debug)]
pub enum Error {
    Overrun,
}

// The type infrastructure already exists to constrain which Pins can be passed: just use the Map
// type, and it will automatically enforce correct behavior for the specific target uC.
// So now we have 2x2 configurations:
// * Duplex vs. simplex
// * Manual vs. automatic CS control
// Each of the 4 possible configurations needs a different set of Pins & Pads.
// Each of the 4 possible configurations must have a unique type for each SERCOM instance so that
// type-level functions work as expected.
//
// There could possibly be one type which uses the OptionalPad type for storage and enforces Map<>
// at construction time, i.e.

/*
pub struct Sercom0SPIMPadout<MISO, MOSI, SCK, CS> {
    _miso: MISO,
    _mosi: MOSI,
    _sck: SCK,
    _cs: CS,
}

impl<PIN0, PIN1, PIN2, PIN3> Sercom0SPIMPadout<PIN0, PIN1, PIN2, PIN3>
    where PIN0: Map<Sercom0, Pad0>,
          PIN1: Map<Sercom0, Pad3>,
          PIN2: Map<Sercom0, Pad1>,
          PIN3: Map<Sercom0, Pad2>
{
    fn new(miso: PIN0, mosi: PIN1, sck: PIN2, cs: PIN3) -> Sercom0SPIMPadout<PIN0, PIN1, PIN2, PIN3> {
        Sercom0SPIMPadout { _miso: miso, _mosi: mosi, _sck: sck, _cs: cs }
    }
}

 */

// Duplex without CS
/*
impl<MISO, MOSI, SCK> From<Sercom0SPIMPadout<MISO, MOSI, SCK, NoneT>> for SPIMaster0
    where MISO: AnyPad,
          MOSI: AnyPad,
          SCK: AnyPad,
{
    fn from(_: Sercom0SPIMPadout<MISO, MOSI, SCK, NoneT>) -> Self {
        unimplemented!()
    }
}
 */

// Simplex without CS
/*
impl<MOSI, SCK> From<Sercom0SPIMPadout<NoneT, MOSI, SCK, NoneT>> for SPIMaster0
where MOSI: AnyPad,
SCK: AnyPad,
{
    fn from(_: Sercom0SPIMPadout<NoneT, MOSI, SCK, NoneT>) -> Self {
        unimplemented!()
    }
}
*/

// If this trait owns everything, why not just make it the SPIMaster?
// Because then we'd have to copy the implementations of traits like FullDuplex and Simplex over
// and over again inside of a macro. By defining a "config" or builder struct, it can go in the
// macro and the actual implementations can live in normal structs that are easier to read.
pub trait SPIMasterPadout {
    type MISO: SPIMasterRXEN;
    type MOSI: AnyPad;
    type SCK: AnyPad;
    type CS: SPIMasterMSSEN;
    type Sercom: Sercom;

    // TODO: methods to get register values for dipo, dopo
    // possibly implemented as other traits (like dipodopo was before)
    fn spi(&self) -> &SPIM;
    fn spi_mut(&mut self) -> &SPIM;
    fn free(self) -> (Self::Sercom, Self::MISO, Self::MOSI, Self::SCK, Self::CS);
}

// TODO: macro Padout impls using Map<> to constrain valid pins

pub trait SPIMasterRXEN: OptionalPad {
    fn rxen() -> u8;
}

impl SPIMasterRXEN for NoneT {
    fn rxen() -> u8 { 0 }
}

impl<T: SomePad> SPIMasterRXEN for T {
    fn rxen() -> u8 { 1 }
}

pub trait SPIMasterMSSEN: OptionalPad {
    fn mssen() -> u8;
}

impl SPIMasterMSSEN for NoneT {
    fn mssen() -> u8 { 0 }
}

impl<T: SomePad> SPIMasterMSSEN for T {
    fn mssen() -> u8 { 1 }
}

pub struct SPIMasterv2<P: SPIMasterPadout> {
    padout: P,
}

impl<P: SPIMasterPadout> SPIMasterv2<P> {
    fn new<T: Into<P>>(pads: T) -> Self {
        let pads = pads.into();
        let _ = P::MISO::rxen();
        let _ = P::CS::mssen();
        // TODO: reset and initialize the SPIM block
        SPIMasterv2 { padout: pads }
    }

    // convenience
    fn spi(&self) -> &SPIM { self.padout.spi() }
    fn spi_mut(&mut self) -> &SPIM { self.padout.spi_mut() }
}

// Full duplex
impl<P: SPIMasterPadout> FullDuplex<u8> for SPIMasterv2<P>
    where P::MISO: SomePad
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let status = self.spi().status.read();
        if status.bufovf().bit_is_set() {
            return Err(nb::Error::Other(Error::Overrun));
        }

        let intflag = self.spi().intflag.read();
        // rxc is receive complete
        if intflag.rxc().bit_is_set() {
            Ok(self.spi().data.read().data().bits() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
        let intflag = self.spi().intflag.read();
        // dre is data register empty
        if intflag.dre().bit_is_set() {
            self.spi_mut().data.write(|w| unsafe { w.data().bits(byte as u32) });
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

// TODO: implement Simplex
// does not actually have to force NonePad, because we can ignore data that gets read
// (unless there's a DMA set up, but that's something else entirely, I suppose)


/// The SercomSPIPadout trait defines a way to get the data in and data out pin out
/// values for a given TODO configuration. You should not implement
/// this trait for yourself; only the implementations in the sercom module make
/// sense.
pub trait DipoDopo {
    fn dipo_dopo(&self) -> (u8, u8);
}

/// Define an SPIMasterX type for the given Sercom number.
///
/// Also defines the valid "pad to spi function" mappings for this instance so
/// that construction is restricted to correct configurations.
macro_rules! spi_master {
    (
        $Type:ident: ($Sercom:ident, $SERCOM:ident, $powermask:ident, $clock:ident, $apmask:ident)
    ) => {

        $crate::paste::item! {
            /// A pad mapping configuration for the SERCOM in SPI master mode.
            ///
            /// This type can only be constructed using the From implementations
            /// in this module, which are restricted to valid configurations.
            ///
            /// Defines which sercom pad is mapped to which SPI function.
            pub struct [<$Type Padout>]<MISO, MOSI, SCK> {
                _miso: MISO,
                _mosi: MOSI,
                _sck: SCK,
            }
        }

        $crate::paste::item! {
            /// A pad mapping configuration for the SERCOM in SPI master mode with
            /// hardware control of the chip select (aka slave select) pin.
            ///
            /// This type can only be constructed using the From implementations
            /// in this module, which are restricted to valid configurations.
            ///
            /// Defines which sercom pad is mapped to which SPI function.
            pub struct [<$Type HWCSPadout>]<MISO, MOSI, SCK, CS> {
                _miso: MISO,
                _mosi: MOSI,
                _sck: SCK,
                _cs: CS,
            }
        }

        /// Define a From instance for a tuple of SercomXPadX instances that
        /// converts them into an SPIMasterXPadout instance.
        ///
        /// Also defines a DipoDopo instance for the constructed padout instance
        /// that returns the values used to configure the sercom pads for the
        /// appropriate function in the sercom register file.
        macro_rules! padout {
            ($dipo_dopo:expr => $pad0:ident, $pad1:ident, $pad2:ident, $pad3:ident) => {
                $crate::paste::item! {
                    /// Convert from a tuple of (MISO, MOSI, SCK, CS) to SPIMasterXHWCSPadout
                    impl<PIN0, PIN1, PIN2, PIN3> From<([<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>, [<$Sercom $pad3>]<PIN3>)> for [<$Type HWCSPadout>]<[<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>, [<$Sercom $pad3>]<PIN3>>
                    where
                        PIN0: Map<$Sercom, $pad0>,
                        PIN1: Map<$Sercom, $pad1>,
                        PIN2: Map<$Sercom, $pad2>,
                        PIN3: Map<$Sercom, $pad3>,
                    {
                        fn from(pads: ([<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>, [<$Sercom $pad3>]<PIN3>)) -> [<$Type HWCSPadout>]<[<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>, [<$Sercom $pad3>]<PIN3>> {
                            [<$Type HWCSPadout>] { _miso: pads.0, _mosi: pads.1, _sck: pads.2, _cs: pads.3 }
                        }
                    }

                    impl<PIN0, PIN1, PIN2, PIN3> DipoDopo for [<$Type HWCSPadout>]<[<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>, [<$Sercom $pad3>]<PIN3>>
                    where
                        PIN0: Map<$Sercom, $pad0>,
                        PIN1: Map<$Sercom, $pad1>,
                        PIN2: Map<$Sercom, $pad2>,
                        PIN3: Map<$Sercom, $pad3>,
                    {
                        fn dipo_dopo(&self) -> (u8, u8) {
                            $dipo_dopo
                        }
                    }
                }
            };
            ($dipo_dopo:expr => $pad0:ident, $pad1:ident, $pad2:ident) => {
                $crate::paste::item! {
                    /// Convert from a tuple of (MISO, MOSI, SCK) to SPIMasterXPadout
                    impl<PIN0, PIN1, PIN2> From<([<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>)> for [<$Type Padout>]<[<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>>
                    where
                        PIN0: Map<$Sercom, $pad0>,
                        PIN1: Map<$Sercom, $pad1>,
                        PIN2: Map<$Sercom, $pad2>,
                    {
                        fn from(pads: ([<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>)) -> [<$Type Padout>]<[<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>> {
                            [<$Type Padout>] { _miso: pads.0, _mosi: pads.1, _sck: pads.2 }
                        }
                    }

                    impl<PIN0, PIN1, PIN2> DipoDopo for [<$Type Padout>]<[<$Sercom $pad0>]<PIN0>, [<$Sercom $pad1>]<PIN1>, [<$Sercom $pad2>]<PIN2>>
                    where
                        PIN0: Map<$Sercom, $pad0>,
                        PIN1: Map<$Sercom, $pad1>,
                        PIN2: Map<$Sercom, $pad2>,
                    {
                        fn dipo_dopo(&self) -> (u8, u8) {
                            $dipo_dopo
                        }
                    }
                }
            };
        }

        // dipo In master operation, DI is MISO Pad number 0-3
        // dopo 0 MOSI PAD 0
        // dopo 2 MOSI PAD 3
        // SCK can only be on PAD 1
        // (dipo,dopo) => (MISO, MOSI, SCK)
        padout!((0, 2) => Pad0, Pad3, Pad1);
        padout!((2, 0) => Pad2, Pad0, Pad1);
        padout!((2, 2) => Pad2, Pad3, Pad1);
        padout!((3, 0) => Pad3, Pad0, Pad1);

        // dipo In master w/ HWCS, DI is MISO Pad 0 or 3
        // dopo 0 MOSI Pad 0
        // dopo 2 MOSI Pad 3
        // SCK can only be on Pad 1
        // CS can only be on Pad 2
        // (dipo,dopo) => (MISO, MOSI, SCK, CS)
        padout!((0, 2) => Pad0, Pad3, Pad1, Pad2);
        padout!((3, 0) => Pad3, Pad0, Pad1, Pad2);

        $crate::paste::item! {
        // TODO: make this generic on the Padout and enable/disable HW CS based on the Padout type?
            /// SPIMasterX represents the corresponding SERCOMX instance
            /// configured to act in the role of an SPI Master.
            /// Objects of this type implement the HAL `FullDuplex` and blocking
            /// SPI traits.
            ///
            /// This type is generic over any valid pad mapping where there is
            /// a defined "data in pin out data out pin out" implementation.
            pub struct $Type<MISO, MOSI, SCK> {
                padout: [<$Type Padout>]<MISO, MOSI, SCK>,
                sercom: $SERCOM,
            }

            impl<MISO, MOSI, SCK> CommonSpi for $Type<MISO, MOSI, SCK> {
                /// Helper for accessing the spi member of the sercom instance
                fn spi(&self) -> &SPIM {
                    &self.sercom.spim()
                }

                /// Helper for accessing the spi member of the sercom instance
                fn spi_mut(&mut self) -> &SPIM {
                    &self.sercom.spim()
                }
            }

            impl<MISO, MOSI, SCK> $Type<MISO, MOSI, SCK> {
                /// Power on and configure SERCOMX to work as an SPI Master operating
                /// with the specified frequency and SPI Mode.  The pinout specifies
                /// which pins are bound to the MISO, MOSI, SCK functions.
                pub fn new<F: Into<Hertz>, T: Into<[<$Type Padout>]<MISO, MOSI, SCK>>>(
                    clock:&clock::$clock,
                    freq: F,
                    mode: Mode,
                    sercom: $SERCOM,
                    mclk: &mut MCLK,
                    padout: T,
                ) -> Self where
                    [<$Type Padout>]<MISO, MOSI, SCK>: DipoDopo {
                    let padout = padout.into();

                    // Power up the peripheral bus clock.
                    // safe because we're exclusively owning SERCOM
                    mclk.$apmask.modify(|_, w| w.$powermask().set_bit());

                    // reset the sercom instance
                    sercom.spim().ctrla.modify(|_, w| w.swrst().set_bit());
                    // wait for reset to complete
                    while sercom.spim().syncbusy.read().swrst().bit_is_set()
                        || sercom.spim().ctrla.read().swrst().bit_is_set()
                    {}

                    // Put the hardware into spi master mode
                    sercom.spim().ctrla.modify(|_, w| w.mode().spi_master());
                    // wait for configuration to take effect
                    while sercom.spim().syncbusy.read().enable().bit_is_set() {}

                    // 8 bit data size and enable the receiver
                    unsafe {
                        sercom.spim().ctrlb.modify(|_, w|{
                            w.chsize().bits(0);
                            w.rxen().set_bit()
                        });
                    }

                    // set the baud rate
                    let baud = Self::calculate_baud(freq, clock.freq());
                    unsafe {
                        sercom.spim().baud.modify(|_, w| w.baud().bits(baud));

                        sercom.spim().ctrla.modify(|_, w| {
                            match mode.polarity {
                                Polarity::IdleLow => w.cpol().clear_bit(),
                                Polarity::IdleHigh => w.cpol().set_bit(),
                            };

                            match mode.phase {
                                Phase::CaptureOnFirstTransition => w.cpha().clear_bit(),
                                Phase::CaptureOnSecondTransition => w.cpha().set_bit(),
                            };

                            let (dipo, dopo) = padout.dipo_dopo();
                            w.dipo().bits(dipo);
                            w.dopo().bits(dopo);

                            // MSB first
                            w.dord().clear_bit()
                        });
                    }


                    sercom.spim().ctrla.modify(|_, w| w.enable().set_bit());
                    // wait for configuration to take effect
                    while sercom.spim().syncbusy.read().enable().bit_is_set() {}

                    Self {
                        padout,
                        sercom,
                    }
                }

                /// Set the baud rate
                pub fn set_baud<F: Into<Hertz>>(
                    &mut self,
                    freq: F,
                    clock:&clock::$clock
                ) {
                    self.disable();
                    let baud = Self::calculate_baud(freq, clock.freq());
                    unsafe {
                        self.spi_mut().baud.modify(|_, w| w.baud().bits(baud));
                    }
                    self.enable();
                }

                /// Tear down the SPI instance and yield the constituent pins and
                /// SERCOM instance.  No explicit de-initialization is performed.
                pub fn free(self) -> ([<$Type Padout>]<MISO, MOSI, SCK>, $SERCOM) {
                    (self.padout, self.sercom)
                }
            }

            impl<MISO, MOSI, SCK> FullDuplex<u8> for $Type<MISO, MOSI, SCK> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let status = self.spi().status.read();
                    if status.bufovf().bit_is_set() {
                        return Err(nb::Error::Other(Error::Overrun));
                    }

                    let intflag = self.spi().intflag.read();
                    // rxc is receive complete
                    if intflag.rxc().bit_is_set() {
                        Ok(self.spi().data.read().data().bits() as u8)
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
                    let intflag = self.spi().intflag.read();
                    // dre is data register empty
                    if intflag.dre().bit_is_set() {
                        self.spi_mut().data.write(|w| unsafe{w.data().bits(byte as u32)});
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            impl<MISO, MOSI, SCK> ::hal::blocking::spi::transfer::Default<u8> for $Type<MISO, MOSI, SCK> {}
            impl<MISO, MOSI, SCK> ::hal::blocking::spi::write::Default<u8> for $Type<MISO, MOSI, SCK> {}
            #[cfg(feature = "unproven")]
            impl<MISO, MOSI, SCK> ::hal::blocking::spi::write_iter::Default<u8> for $Type<MISO, MOSI, SCK> {}
        }
    };
}

spi_master!(SPIMaster0: (Sercom0, SERCOM0, sercom0_, Sercom0CoreClock, apbamask));
spi_master!(SPIMaster1: (Sercom1, SERCOM1, sercom1_, Sercom1CoreClock, apbamask));
spi_master!(SPIMaster2: (Sercom2, SERCOM2, sercom2_, Sercom2CoreClock, apbbmask));
spi_master!(SPIMaster3: (Sercom3, SERCOM3, sercom3_, Sercom3CoreClock, apbbmask));
spi_master!(SPIMaster4: (Sercom4, SERCOM4, sercom4_, Sercom4CoreClock, apbdmask));
spi_master!(SPIMaster5: (Sercom5, SERCOM5, sercom5_, Sercom5CoreClock, apbdmask));
#[cfg(feature = "min-samd51n")]
spi_master!(SPIMaster6: (Sercom6, SERCOM6, sercom6_, Sercom6CoreClock, apbdmask));
#[cfg(feature = "min-samd51n")]
spi_master!(SPIMaster7: (Sercom7, SERCOM7, sercom7_, Sercom7CoreClock, apbdmask));
