#![cfg_attr(not(test), no_std)]
#![cfg_attr(debug_assertions, forbid(unsafe_code))]
#![allow(clippy::zero_prefixed_literal)]
#![allow(clippy::unusual_byte_groupings)]
#![deny(clippy::missing_const_for_fn)]
#![deny(missing_debug_implementations)]
#![deny(missing_docs)]
//! # Introduction
//! This crate serves to model and solve the following couple of problems:
//!
//! 1. How can we model all the possible rotations that can happen to a cube,
//!     that leave all its edges parallel to the same axes as before?
//!
//! 2. How many and which points must we place on this cube's surface such that
//!     a) they are absolutely symmetric according to the previous rotations, and
//!     b) they have absolutely no mirror symmetry?
//!     In other words, how can we make sure that the cube will look the same from
//!     every possible orientation, while being different than its mirror image?
//!
//! This is known as
//! [_octahedral symmetry_](https://en.wikipedia.org/wiki/Octahedral_symmetry)
//! in the available literature. The linked article does contain an explanation,
//! but this documentation aims to be a smidge more accessible.
//!
//! ## Initial approach and restrictions
//! Let there be a cube centred around the origin, with an edge-length of 6; the
//! criterion for the length will shortly become apparent. The vertices of this
//! cube are the eight points `[±3, ±3, ±3]`.
//!
//! Our first order of business, obviously, is to place one point on some face
//! of this cube. Its faces are the set of points whose maximum coördinate by
//! absolute value is equal to `3`. Thus, it immediately follows that all
//! coördinates must be between `-3` and `3` inclusive.
//!
//! The next thing to do is to ensure that, if the cube is reflected along one
//! of its planes of symmetry, no point will coincide with itself. Thus, the
//! question is raised: What is the effect that a reflection has on the
//! coördinates of a point?
//!
//! The answer: If the plane if symmetry is horizontal or vertical, it negates
//! one of the coördinates. If it is diagonal, it swaps the values of
//! two coördinates.
//!
//! With that in mind, we immediately arrive at two fundamental restrictions: no
//! coördinate can be zero, or else negating it would not change it; and no
//! coördinate can be equal to another, or else swapping them would change
//! nothing.
//!
//! ## Encoding the transforms
//! Next question: What do rotations do to a point's coördinates? And the answer
//! is that _each even amount of reflections corresponds to a rotation_[^¹]. In
//! other words, each rotation can be decomposed to an even amount of negations
//! and swaps.
//!
//! This immediately answers both our questions. 3 coördinates can be ordered in
//! 6 different ways in total, and their possible combinations of signs are 8.
//! Thus, the possible transformations are 6×8 = 48 in total: 24 of them are
//! rotations, and the other 24 are so-called
//! _improper rotations_ or _rotoreflections_.
//!
//! [^¹]: This is a general property of geometry, irrespective of amount of
//! dimensions or possibly even Euclideanness.
//!
//! Apart from that, for each complete transformation, each elementary
//! reflection that comprises it needs only occur once, and they can always be
//! examined in the same order. This permits us to encode, in just 1 bit, the
//! application-or-not of each one of them, and thus fit all possible
//! transformations within 6 bits. Of those, the rotations can be discerned by
//! the fact that their binary representation will have an even amount of ones.
//!
//! ## Encoding coördinates
//! With regards to the encoding of the coördinates themselves, the choice of 6
//! for the size of the cube permits us to use integers for all 3 coördinates:
//! `±1`, `±2`, `±3`. By choosing a specific order and signs, each of the 48
//! possible combinations can be found.
//!
//! It is noteworthy that, if we assume one point to be a reference,
//! (`[1, 2, 3]` in our case,) every other point can be found by applying one
//! of those transformations –a different one each time– on it. This has the
//! extremely useful property that it permits us to encode transformations and
//! points in the exact same way. It also immediately separates those 48
//! possible points into two groups of 24: `[1, 2, 3]` and its possible
//! rotations, and `[3, 2, 1]` and its possible rotations. Each point belonging
//! to one of those groups can, with some suitable rotation, be made to coincide
//! with any other point of the same group; however, it is impossible for a
//! rotation to make it coincide with a point from the other group. Thus, each
//! of those two groups can comprise the solution to our original question.
//!
//! Of note: since each transformation retains the absolute values of the
//! coördinates of each point, their exact values are not important. The point
//! `[-400, 0.5, 14]` can be considered to be in `[-3, 1, 2]` and all operations
//! remain correct.
//!
//! # Nomenclature
//! In the rest of the documentation, each point is called a
//! “[`CubeSurfacePoint`]”, and each set of operations on it (swaps and/or sign
//! flips) is called a “Transformation” or a “[`Rotation`]”. The point
//! `[1, 2, 3]` as also commonly called a Reference Point, because we judge all
//! transformations relative to it.
//!
//! In the documentation, the two possible groups of 24 points are called
//! _Geometric Groups_. They are sometimes distinguished into the _Reference
//! Geometric Group_, ie the Point of Reference and the points in the same group
//! as it, and the _Opposite Geometric Group_, ie the mirror image of the
//! Reference Geometric Group.
//!
//! The afore-mentioned reflections to which each of the 6 bits corresponds (ie,
//! swapping the value of two coördinates or negating one of them) are
//! collectively referred to as _Elementary Reflections_.
//!
//! # Using the crate
//! Let there be three points on a cube, `x`, `y`, and `z`.
//!
//! ```
//! # use cube_rotations::CubeSurfacePoint::*;
//! let x = NegThreePosOnePosTwo;
//! let y = PosTwoPosThreeNegOne;
//! let z = PosThreePosOneNegTwo;
//! ```
//!
//! We eventually realise that `x`'s actual orientation is different:
//!
//! ```
//! # use cube_rotations::CubeSurfacePoint::*;
//! let x_actual = PosThreePosTwoPosOne;
//! ```
//! We want our cube to be rotated in such a way, that `x` ends up coïnciding
//! with `x_actual`. That means that each of those points has to be rotated in
//! the exact same way.
//!
//! To calculate the rotation, it suffices to perform one division:
//! `rotation = x_actual / x`.
//! Afterwards, the operations `r * x`, `r * y`, and `r * z` give us the results
//! we want. Please note that the operation `(x_actual / x) * x` gives just
//! `x_actual`, exactly as one would suppose from the notation.
//!
//! ```
//! # use cube_rotations::CubeSurfacePoint::*;
//! # let x = NegThreePosOnePosTwo;
//! # let y = PosTwoPosThreeNegOne;
//! # let z = PosThreePosOneNegTwo;
//! #
//! # let x_actual = PosThreePosTwoPosOne;
//! let rotation = x_actual / x;
//! ```
//! Here is the complete code, including validation of results:
//! ```
//! # use cube_rotations::CubeSurfacePoint::*;
//! let x = NegThreePosOnePosTwo;
//! let y = PosTwoPosThreeNegOne;
//! let z = PosThreePosOneNegTwo;
//!
//! let x_actual = PosThreePosTwoPosOne;
//!
//! let rotation = x_actual / x;
//!
//! assert_eq!(rotation * x, PosThreePosTwoPosOne);
//! assert_eq!(rotation * y, NegTwoNegOnePosThree);
//! assert_eq!(rotation * z, NegThreeNegTwoPosOne);
//! ```
//!
//! # Implementation details
//! This crate was implemented with the following criteria, in descending order
//! of importance:
//! 1. Correct structuring of API
//! 2. Serving as a proof-of-concept
//! 3. Needing as little memory as possible (RAM + program memory)
//! 4. Performance
//!
//! Performance is last as, for the use-case for which this crate was
//! implemented, those calculations are far outside of the critical path.
//!
//! # Safety and panics
//! Despite this code using `unsafe` internally, running `cargo build`
//! successfully is enough to guarantee the complete absence of panics or
//! undefined behaviour –henceforth “UB”– in this code. This is thanks to the
//! following properties:
//! * All functions are pure, ie stateless: this permits them to all be `const`.
//!     Additionally, no function has more than 2304 possible inputs. This means
//!     that all of them can be checked exhaustively in a relatively short amount of
//!     time, even under `miri`.
//! * All panics/unsafety in this crate have been corralled into one function,
//!     `unreachable_semichecked`. In debug mode, it panics; in release mode, it's
//!     UB; and it is the only function in the entire code-base that is permitted to
//!     do either of those things. As a result, as long as it remains indeed
//!     unreachable, the entire code-base is free from both panics and UB.
//! * Each function has a corresponding one that operates using a Look-Up Table—
//!     henceforth “LUT”. The LUT is populated at compile-time by calling the
//!     corresponding `const` function iteratively, and exhaustively. This means
//!     that every function ends up being called with every possible input during
//!     compilation: in other words, every possible code-path gets activated at
//!     compile-time. (A funny result of this is that the code builds
//!     faster with `--release` than without it.)
//! * As a combination of the above, any possible code-path leading to the
//!     `unreachable_semichecked` function will indeed call it during compile-time,
//!     resulting in a compilation error as long as debug assertions are enabled.
//!     Thus, the absence of compilation errors suffices to prove the complete
//!     absence of panics or UB in the entire code-base.
//! * Lastly, as a bonus: The test suite is so comprehensive that, as long as it
//!     is intact, even a purposeful sabotage of the crate would struggle to cause
//!     serious damage without test-breakage.
//!
//! That said, this crate has also been annotated with the following attribute:
//! ```rust
//! #![cfg_attr(debug_assertions, forbid(unsafe_code))]
//! ```
//! Thus, any down-stream user that prefers to forbid `unsafe` code entirely can
//! do so by including the following snippet in their `Cargo.toml` file:
//!
//! ```toml
//! [profile.dev.package.cube-rotations]
//! debug-assertions = true
//! ```
//! This, of course, opens the door for some missed optimisations. If there is
//! any way to achieve both of those things, either through regular compiler
//! optimisations or through PGO, that has not been investigated as of yet.
//!
//! # Look-up tables and proper rotations
//! The most basic and most broadly applicable data-types contained herein are
//! [`CubeSurfacePoint`] and [`Rotation`]. They can be used to model all
//! relevant operations, but offer no particular guarantees.
//!
//! Each of those two data-types is further split into two equally-sized,
//! compementary sub-sets, each offering a particular geometric guarantee.
//! Further, all of those 3 point-types also have wrapper data-types that
//! operate using look-up tables. In turn, all 7 of the point data-types are
//! generalised in the [`OctahedrallySymmetricPoint`] trait.
//!
//! The full list is as follows:
//! * [`ProperRotation`]: A subset of [`Rotation`]. Models a proper rotation, ie
//!     one that can happen in the real world.
//! * [`ImproperRotation`]: A subset of [`Rotation`] that models a
//!     rotoreflection. Complementary to [`ProperRotation`].
//! * [`ReferenceGroupPoint`]: A subset of [`CubeSurfacePoint`].
//!     Models a point in space that can be made to
//!     coincide with the Reference Point using just one rotation.
//! * [`OppositeGroupPoint`]: A subset of [`CubeSurfacePoint`]. Models a point
//!     in space that can be made to coincide with the Reference Point using just
//!     one rotoreflection. Complementary to [`ReferenceGroupPoint`].
//! * [`luts::CubeSurfacePoint::<false>`]: As per the basic
//!     [`CubeSurfacePoint`], but uses LUTs. Each LUT is at most 48 bytes in length,
//!     but some operations might need multiple look-ups.
//! * [`luts::CubeSurfacePoint::<true>`]: As per the basic
//!     [`CubeSurfacePoint`], but uses LUTs. Each LUT is up to 2304 bytes in length,
//!     and each operation is guaranteed to consist of a single look-up.
//! * [`luts::ReferenceGroupPoint`]: As per the basic
//!     [`ReferenceGroupPoint`], but uses LUTs. Each LUT is up to 576 bytes in
//!     length, and each operation is guaranteed to consist of a single look-up.
//! * [`luts::OppositeGroupPoint`]: As per the basic
//!     [`OppositeGroupPoint`], but uses LUTs. Each LUT is up to 576 bytes in
//!     length, and each operation is guaranteed to consist of a single look-up.
//! * [`OctahedrallySymmetricPoint`]: A trait that generalises the operation of
//!     all 7 afore-mentioned point data-types.
//!
//! The Geometric-Group-specific data-types of the [`luts`] module only operate
//! using big LUTs, for reasons further analysed in their documentation.

pub mod luts;
use core::ops::{Div, Mul, MulAssign};

/// Encodes the 48 possible points in space whose coördinates, in
/// ascending order of absolute value, are equal to `[1, 2, 3]`.
///
/// Uses 6 bits, and is therefore represented as a `u8`.
///
/// # Usage
/// ```
/// # use cube_rotations::*;
/// # use cube_rotations::CubeSurfacePoint::*;
/// let test = PosOnePosTwoPosThree;
/// assert_eq!(test as u8, 0);
/// ```
///
/// # Representation
/// The bits of each encoding correspond one-by-one to the
/// Elementary Reflections that have to happen to the Reference Point in order
/// to produce the result we want. The documentation for the [`Rotation`]
/// data-type contains more details.
///
/// # Geometric Groups
/// A very important property is that those points can be divided into
/// two geometric categories, depending on whether the amount of ones in their
/// binary representation is odd or even. In each group, each `CubeSurfacePoint`
/// can be made to coincide with any other point using just one rotation. For it
/// to coincide with a point from the other group, however, it'd also require a
/// reflection, or what's called an “improper rotation” or “rotoreflection”.
/// The [`ReferenceGroupPoint`] and [`OppositeGroupPoint`] data-types separate
/// the two, and respectively correspond to the [`ProperRotation`] and
/// [`ImproperRotation`] transformation data-types.
///
/// Below please find each possible point, along with the transformation to
/// which it corresponds. The signs for the rotations have been chosen in
/// accordance with the `nalgebra` crate.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
#[repr(u8)]
pub enum CubeSurfacePoint {
    /// The point `[1, 2, 3]`. Also the Point of Reference. Divided by itself,
    /// it yields –unsurprisingly enough– the identity operation.
    PosOnePosTwoPosThree = 00,
    /// The point `[1, 2, -3]`. Divided by the Reference Point, it yields a
    /// reflection through the `z = 0` plane. Its arithmetic representation is
    /// equal to 2<sup>0</sup>, thus it is the zeroth Elementary Reflection.
    PosOnePosTwoNegThree = 01,
    /// The point `[1, -2, 3]`. Divided by the Reference Point, it yields a
    /// reflection through the `y = 0` plane. Its arithmetic representation is
    /// equal to 2<sup>1</sup>, thus it is the first Elementary Reflection.
    PosOneNegTwoPosThree = 02,
    /// The point `[1, -2, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `x` axis.
    PosOneNegTwoNegThree = 03,
    /// The point `[-1, 2, 3]`. Divided by the Reference Point, it yields a
    /// reflection through the `x = 0` plane. Its arithmetic representation is
    /// equal to 2<sup>2</sup>, thus it is the second Elementary Reflection.
    NegOnePosTwoPosThree = 04,
    /// The point `[-1, 2, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `y` axis.
    NegOnePosTwoNegThree = 05,
    /// The point `[-1, -2, 3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `z` axis.
    NegOneNegTwoPosThree = 06,
    /// The point `[-1, -2, -3]`. Divided by the Reference Point, it yields a
    /// complete central inversion, ie a negation of all coördinates.
    NegOneNegTwoNegThree = 07,
    /// The point `[3, 2, 1]`. Divided by the Reference Point, it yields a
    /// reflection through the `x = z` plane. Its arithmetic representation is
    /// equal to 2<sup>3</sup>, thus it is the third Elementary Reflection.
    PosThreePosTwoPosOne = 08,
    /// The point `[3, 2, -1]`. Divided by the Reference Point, it yields a
    /// rotation of 90° around the `y` axis.
    PosThreePosTwoNegOne = 09,
    /// The point `[3, -2, 1]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `y = 0, z = x` axis.
    PosThreeNegTwoPosOne = 10,
    /// The point `[3, -2, -1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 90° with respect to the `y` axis.
    PosThreeNegTwoNegOne = 11,
    /// The point `[-3, 2, 1]`. Divided by the Reference Point, it yields a
    /// rotation of -90° around the `y` axis.
    NegThreePosTwoPosOne = 12,
    /// The point `[-3, 2, -1]`. Divided by the Reference Point, it yields a
    /// reflection through the `z = -x` plane.
    NegThreePosTwoNegOne = 13,
    /// The point `[-3, -2, 1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -90° with respect to the `y` axis.
    NegThreeNegTwoPosOne = 14,
    /// The point `[-3, -2, -1]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `y = 0, z = -x` axis.
    NegThreeNegTwoNegOne = 15,
    /// The point `[1, 3, 2]`. Divided by the Reference Point, it yields a
    /// reflection through the `z = y` plane. Its arithmetic representation is
    /// equal to 2<sup>4</sup>, thus it is the fourth Elementary Reflection.
    PosOnePosThreePosTwo = 16,
    /// The point `[1, 3, -2]`. Divided by the Reference Point, it yields a
    /// rotation of -90° around the `x` axis.
    PosOnePosThreeNegTwo = 17,
    /// The point `[1, -3, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 90° around the `x` axis.
    PosOneNegThreePosTwo = 18,
    /// The point `[1, -3, -2]`. Divided by the Reference Point, it yields a
    /// reflection through the `y = -z` plane.
    PosOneNegThreeNegTwo = 19,
    /// The point `[-1, 3, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `x = 0, y = z` axis.
    NegOnePosThreePosTwo = 20,
    /// The point `[-1, 3, -2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -90° with respect to the `x = y = z` axis.
    NegOnePosThreeNegTwo = 21,
    /// The point `[-1, -3, 2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 90° with respect to the `x` axis.
    NegOneNegThreePosTwo = 22,
    /// The point `[-1, -3, -2]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `x = 0, y = -z` axis.
    NegOneNegThreeNegTwo = 23,
    /// The point `[3, 1, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = y = z` axis.
    PosThreePosOnePosTwo = 24,
    /// The point `[3, 1, -2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = -y = -z` axis.
    PosThreePosOneNegTwo = 25,
    /// The point `[3, -1, 2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = y = -z` axis.
    PosThreeNegOnePosTwo = 26,
    /// The point `[3, -1, -2]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = -y = z` axis.
    PosThreeNegOneNegTwo = 27,
    /// The point `[-3, 1, 2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = -y = z` axis.
    NegThreePosOnePosTwo = 28,
    /// The point `[-3, 1, -2]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = y = -z` axis.
    NegThreePosOneNegTwo = 29,
    /// The point `[-3, -1, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = -y = -z` axis.
    NegThreeNegOnePosTwo = 30,
    /// The point `[-3, -1, -2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = y = z` axis.
    NegThreeNegOneNegTwo = 31,
    /// The point `[2, 1, 3]`. Divided by the Reference Point, it yields a
    /// reflection through the `y = x` plane. Its arithmetic representation is
    /// equal to 2<sup>5</sup>, thus it is the fifth and final
    /// Elementary Reflection.
    PosTwoPosOnePosThree = 32,
    /// The point `[2, 1, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `z = 0, x = y` axis.
    PosTwoPosOneNegThree = 33,
    /// The point `[2, -1, 3]`. Divided by the Reference Point, it yields a
    /// rotation of -90° around the `z` axis.
    PosTwoNegOnePosThree = 34,
    /// The point `[2, -1, -3]`. Divided by the Reference Point, it yields a
    /// reflection through the `x = -y` plane.
    PosTwoNegOneNegThree = 35,
    /// The point `[-2, 1, 3]`. Divided by the Reference Point, it yields a
    /// rotation of 90° around the `z` axis.
    NegTwoPosOnePosThree = 36,
    /// The point `[-2, 1, -3]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 90° with respect to the `z` axis.
    NegTwoPosOneNegThree = 37,
    /// The point `[-2, -1, 3]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -90° with respect to the `z` axis.
    NegTwoNegOnePosThree = 38,
    /// The point `[-2, -1, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `z = 0, x = -y` axis.
    NegTwoNegOneNegThree = 39,
    /// The point `[2, 3, 1]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = y = z` axis.
    PosTwoPosThreePosOne = 40,
    /// The point `[2, 3, -1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = -y = z` axis.
    PosTwoPosThreeNegOne = 41,
    /// The point `[2, -3, 1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = -y = -z` axis.
    PosTwoNegThreePosOne = 42,
    /// The point `[2, -3, -1]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = y = -z` axis.
    PosTwoNegThreeNegOne = 43,
    /// The point `[-2, 3, 1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = y = -z` axis.
    NegTwoPosThreePosOne = 44,
    /// The point `[-2, 3, -1]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = -y = -z` axis.
    NegTwoPosThreeNegOne = 45,
    /// The point `[-2, -3, 1]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = -y = z` axis.
    NegTwoNegThreePosOne = 46,
    /// The point `[-2, -3, -1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = y = z` axis.
    NegTwoNegThreeNegOne = 47,
}

/// A data-type that describes, within 6 bits, the transformation from one
/// [`CubeSurfacePoint`] to another.
///
/// # Usage
/// ```
/// #  {
/// # use cube_rotations::*;
/// # fn random_point() -> CubeSurfacePoint {
/// #   let x = rand::random::<u8>() % 48;
/// #   x.try_into().unwrap()
/// # }
/// # let beginning_point = random_point();
/// # let ending_point = random_point();
/// let rotation = ending_point / beginning_point;
/// assert_eq!(beginning_point * rotation, ending_point);
/// # }
/// ```
///
/// # Representation
/// For a complete correspondence between _arithmetic values_ and geometric
/// transformations, please refer to the documentation for
/// [`CubeSurfacePoint`]. In here, we will describe the correspondence of each
/// transformation with its _binary representation_.
///
/// The gist is that the three first bits describe the order in which the three
/// coördinates must be positioned, while the last three bits describe which of
/// their signs have to be flipped. It must be noted that the bits cannot be
/// examined in arbitrary order: Bit 3 must always be examined first, and the
/// three last bits must be examined last. Their order and meaning is as
/// follows:
///  * **3**: Swaps coöordinates 1 and 3.
///  * **4**: Swaps coöordinates 2 and 3.
///  * **5**: Swaps coöordinates 1 and 2.
///  * **2**: Flips the sign of coördinate 1.
///  * **1**: Flips the sign of coördinate 2.
///  * **0**: Flips the sign of coördinate 3.
///
/// Also note that this data-type encodes both proper and improper rotations.
/// The [`ProperRotation`] and [`ImproperRotation`] data-types separate the two.
///
/// # Multiplying/dividing rotations together
/// The various rotation data-types can be multiplied, and therefore
/// have had `Mul` and `Div` implemented between
/// them:
/// * The ordinary `Rotation` can only be multiplied with itself.
/// * In contrast, the `ProperRotation` and the `ImproperRotation` can be
///     multiplied both with themselves and with each other.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct Rotation {
    corresponding_point: CubeSurfacePoint,
}

/// Extracts the rotation that must occur so that the `divisor` point ends up
/// coinciding with `self`, ie the dividend.
impl Div for CubeSurfacePoint {
    /// A rotation, either proper or improper.
    type Output = Rotation;

    /// The operation occurs by finding the reciprocal of the `divisor`, and
    /// rotating it as described by `self`.
    fn div(self, divisor: Self) -> Self::Output {
        self.div(divisor)
    }
}

/// Rotates a copy of `self` per some `Rotation`.
impl Mul<Rotation> for CubeSurfacePoint {
    /// Output same as input.
    type Output = Self;

    /// The operation is performed by examining the bits of the `Rotation`
    /// one-by-one, then performing the corresponding elementary reflections on
    /// `self`.
    fn mul(self, rot: Rotation) -> Self::Output {
        self.mul(rot)
    }
}

/// Rotates `self` per some `Rotation`.
impl MulAssign<Rotation> for CubeSurfacePoint {
    /// The data-type doesn't change, so the result can be directly
    /// assigned.
    fn mul_assign(&mut self, rot: Rotation) {
        *self = *self * rot;
    }
}

/// Converts an `u8` to a `CubeSurfacePoint`, if it is within limits.
///
/// If it is not, the `u8` is returned as-is.
impl core::convert::TryFrom<u8> for CubeSurfacePoint {
    type Error = u8;

    fn try_from(input: u8) -> Result<Self, Self::Error> {
        Self::try_from_u8(input).ok_or(input)
    }
}

impl Rotation {
    /// Because [`Mul`] is not `const`.
    pub const fn mul(self, x: CubeSurfacePoint) -> CubeSurfacePoint {
        x.mul(self)
    }
}

/// Rotates a given `CubeSurfacePoint` according to `self`.
impl Mul<CubeSurfacePoint> for Rotation {
    /// Output same as input.
    type Output = CubeSurfacePoint;

    /// The operation is performed by examining `self`'s bits one by one, and
    /// performing on the other `surface_point` the corresponding elementary
    /// reflections.
    fn mul(self, cub_sur_pt: CubeSurfacePoint) -> Self::Output {
        cub_sur_pt * self
    }
}

/// Helper function for marking code that's dead, and therefore unreachable.
/// In other words, if this function gets actually called, something has
/// gone very wrong.
///
/// If debug assertions had been disabled, this function would be immediate
/// Undefined Behaviour. With the current compilation flags, however, it
/// merely panics unconditionally.
#[cfg(debug_assertions)]
#[cold]
const fn unreachable_semichecked<T>() -> T {
    debug_assert!(false, "Dead code was called!");
    panic!()
}

/// Helper function for marking code that's dead, and therefore unreachable.
/// In other words, if this function gets actually called, something has
/// gone very wrong.
///
/// If debug assertions had been enabled, this function would panic
/// unconditionally. With the current compilation flags, however, it
/// produces immediate Undefined Behaviour.
#[cfg(not(debug_assertions))]
#[cold]
const fn unreachable_semichecked<T>() -> T {
    debug_assert!(false, "Dead code was called!");
    unsafe { core::hint::unreachable_unchecked() }
}

impl CubeSurfacePoint {
    /// The point chosen as a Reference Point, according to which all rotations
    /// are judged.
    ///
    /// Its binary representation is 0 by definition.
    ///
    /// The point `[1, 2, 3]` has been chosen as the Reference Point for this
    /// crate, corresponding to `CubeSurfacePoint::PosOnePosTwoPosThree`.
    pub const REFERENCE_POINT: Self = Self::probs_from_u8(0);

    /// Counts how many ones there are in the binary representation of a certain
    /// [`CubeSurfacePoint`], so it can be judged in which geometric group it
    /// belongs.
    ///
    /// ```
    /// #  {
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// assert_eq!(PosTwoNegThreeNegOne.odd_ones(), false);
    /// # }
    /// ```
    pub const fn odd_ones(self) -> bool {
        (self as u8).count_ones() & 1 != 0
    }

    /// Equal to the negation of [`CubeSurfacePoint::odd_ones`] by definition.
    pub const fn even_ones(self) -> bool {
        !self.odd_ones()
    }

    const fn xor(self, bit_mask: u8) -> Self {
        Self::probs_from_u8(self as u8 ^ bit_mask)
    }

    /// Discriminates a given [`CubeSurfacePoint`] depending on the geometric
    /// group to which it belongs. If it belongs to the Reference Rotation
    /// Group, it returns an `Ok<ReferenceGroupPoint>`, else it returns an
    /// `Err<OppositeGroupPoint>`.
    #[inline(always)]
    pub const fn determine_group(
        self: CubeSurfacePoint,
    ) -> Result<ReferenceGroupPoint, OppositeGroupPoint> {
        use CubeSurfacePoint as Csp;

        macro_rules! test {
            ($type:ty; $($names:ident),+ $(,)?) => {
                match self {
                    $( // They must have both the same binary representation,
                        _ if (self as u8 == <$type>::$names as u8) &&
                                (self as u8 == Csp::$names as u8) => {
                            <$type>::$names
                        }, // and the same name.
                    )+
                    _ => unreachable_semichecked(),
                }
            };
        }
        if self.even_ones() {
            Ok(test!(ReferenceGroupPoint;
                PosOnePosTwoPosThree, PosOneNegTwoNegThree,
                NegOnePosTwoNegThree, NegOneNegTwoPosThree,
                PosThreePosTwoNegOne, PosThreeNegTwoPosOne,
                NegThreePosTwoPosOne, NegThreeNegTwoNegOne,
                PosOnePosThreeNegTwo, PosOneNegThreePosTwo,
                NegOnePosThreePosTwo, NegOneNegThreeNegTwo,
                PosThreePosOnePosTwo, PosThreeNegOneNegTwo,
                NegThreePosOneNegTwo, NegThreeNegOnePosTwo,
                PosTwoPosOneNegThree, PosTwoNegOnePosThree,
                NegTwoPosOnePosThree, NegTwoNegOneNegThree,
                PosTwoPosThreePosOne, PosTwoNegThreeNegOne,
                NegTwoPosThreeNegOne, NegTwoNegThreePosOne,
            ))
        } else {
            Err(test!(OppositeGroupPoint;
                PosOnePosTwoNegThree, PosOneNegTwoPosThree,
                NegOnePosTwoPosThree, NegOneNegTwoNegThree,
                PosThreePosTwoPosOne, PosThreeNegTwoNegOne,
                NegThreePosTwoNegOne, NegThreeNegTwoPosOne,
                PosOnePosThreePosTwo, PosOneNegThreeNegTwo,
                NegOnePosThreeNegTwo, NegOneNegThreePosTwo,
                PosThreePosOneNegTwo, PosThreeNegOnePosTwo,
                NegThreePosOnePosTwo, NegThreeNegOneNegTwo,
                PosTwoPosOnePosThree, PosTwoNegOneNegThree,
                NegTwoPosOneNegThree, NegTwoNegOnePosThree,
                PosTwoPosThreeNegOne, PosTwoNegThreePosOne,
                NegTwoPosThreePosOne, NegTwoNegThreeNegOne,
            ))
        }
    }

    /// The same as [`CubeSurfacePoint::determine_group()`], but returns an `Ok`
    /// value for `OppositeGroupPoint`s and an `Err` value otherwise.
    #[inline(always)]
    pub const fn determine_antigroup(
        self: CubeSurfacePoint,
    ) -> Result<OppositeGroupPoint, ReferenceGroupPoint> {
        match self.determine_group() {
            Ok(a) => Err(a),
            Err(a) => Ok(a),
        }
    }

    /// Used in implementing the division. Shouldn't be needed for end users;
    /// even if it is, it can be computed by
    /// `(REFERENCE_POINT / self) * REFERENCE_POINT`.
    const fn reciprocal(self) -> Self {
        let mut result = Self::probs_from_u8(self as u8 & 7);

        if self as u8 & 0b100_000 != 0 {
            result = result.swap_x_y();
        }

        if self as u8 & 0b010_000 != 0 {
            result = result.swap_y_z();
        }

        if self as u8 & 0b001_000 != 0 {
            result = result.swap_z_x();
        }

        result
    }

    /// Implements the fifth Elementary Reflection.
    const fn swap_x_y(&self) -> Self {
        let mut x = *self as u8;
        if x & 0b010_000 != 0 {
            x ^= 0b001_000;
        } else {
            x ^= 0b100_000;
        }
        if (x + 2) & 7 > 3 {
            x ^= 0b000_110;
        }
        Self::probs_from_u8(x)
    }

    /// Implements the fourth Elementary Reflection.
    const fn swap_y_z(&self) -> Self {
        let mut x = *self as u8;
        if x >= 0b100_000 {
            x ^= 0b001_000;
        } else {
            x ^= 0b010_000;
        }
        if (x + 1) & 3 > 1 {
            x ^= 0b000_011;
        }
        Self::probs_from_u8(x)
    }

    /// Implements the third Elementary Reflection.
    const fn swap_z_x(&self) -> Self {
        let mut x = *self as u8;
        if x < 0b010_000 {
            x ^= 0b001_000;
        } else {
            x ^= 0b111_000;
        }
        let x_ = (x & 5).wrapping_sub(1);
        if x_ & 7 < 4 {
            x ^= 0b000_101;
        }
        Self::probs_from_u8(x)
    }

    // For the curious: The second, first, and zeroth Elementary Reflections
    // are jointly implemented as simple bitwise XOR.

    /// Returns the point in the corresponding position of the opposite side of
    /// the cube.
    ///
    /// Basically, finds which coördinate has an absolute value of 3 and
    /// multiplies it by -1.
    ///
    /// ```
    /// #  {
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// assert_eq!(PosOnePosTwoPosThree.opposite(), PosOnePosTwoNegThree);
    /// # }
    /// ```
    pub const fn opposite(self) -> Self {
        if (self as u8) & 0b011_000 == 0 {
            self.xor(0b001)
        } else if (self as u8) & 0b101_000 == 0b001_000 {
            self.xor(0b100)
        } else {
            self.xor(0b010)
        }
    }

    /// Returns the other point found in the same edge of the same face of the
    /// cube.
    ///
    /// Basically, finds which coördinate has an absolute value of 1 and
    /// multiplies it by -1.
    ///
    /// ```
    /// #  {
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// assert_eq!(PosOnePosTwoPosThree.beside(), NegOnePosTwoPosThree);
    /// # }
    /// ```
    pub const fn beside(self) -> Self {
        if (self as u8) >= 24 && (self as u8) < 40 {
            self.xor(0b010)
        } else if (self as u8) & 0b001_000 != 0 {
            self.xor(0b001)
        } else {
            self.xor(0b100)
        }
    }

    /// Functionally identical to `self.opposite().beside()`, or
    /// (equivalently) `self.beside().opposite()`. Implemented separately
    /// for optimisation purposes, as
    /// 1. It was useful for our purposes,
    /// 2. It maintains the geometric group of its input, and
    /// 3. It is _way_ simpler to implement than either function separately.
    ///
    /// Corresponds to a rotation of 180° as seen from the face to which the
    /// point is _closest_ without being _on_.
    ///
    /// ```
    /// #  {
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// assert_eq!(PosOnePosTwoPosThree.opposite_then_beside(), NegOnePosTwoNegThree);
    /// # }
    /// ```
    pub const fn opposite_then_beside(self) -> Self {
        if (self as u8) >= 32 {
            self.xor(0b011)
        } else if (self as u8) < 16 {
            self.xor(0b101)
        } else {
            self.xor(0b110)
        }
    }

    /// Different name for [`CubeSurfacePoint::opposite_then_beside`].
    pub const fn beside_then_opposite(self) -> Self {
        self.opposite_then_beside()
    }

    /// Different name for [`CubeSurfacePoint::beside`].
    pub const fn flip_sign_of_1(self) -> Self {
        self.beside()
    }

    /// No idea if this if going to be useful, but it's so easy to implement
    /// that we might as well do so for completeness.
    pub const fn flip_sign_of_2(self) -> Self {
        if (self as u8) >= 32 {
            self.xor(0b100)
        } else if (self as u8) < 16 {
            self.xor(0b010)
        } else {
            self.xor(0b001)
        }
    }

    /// Different name for the [`CubeSurfacePoint::opposite`] function.
    pub const fn flip_sign_of_3(self) -> Self {
        self.opposite()
    }

    /// Different name for [`CubeSurfacePoint::opposite_then_beside`].
    ///
    /// Corresponds to a rotation of 180° as seen from the face to which the
    /// point is _closest_ without being _on_.
    pub const fn flip_1_and_3(self) -> Self {
        self.opposite_then_beside()
    }

    /// A rotation of 180° as seen from a face which the
    /// point is neither _on_ nor _close to_.
    pub const fn flip_2_and_3(self) -> Self {
        if (self as u8) >= 24 && (self as u8) < 40 {
            self.xor(0b101)
        } else if (self as u8) & 0b001_000 != 0 {
            self.xor(0b110)
        } else {
            self.xor(0b011)
        }
    }

    /// A rotation of 180° as seen from the face _on_ which the
    /// point is located.
    pub const fn flip_1_and_2(self) -> Self {
        if (self as u8) & 0b011_000 == 0 {
            self.xor(0b110)
        } else if (self as u8) & 0b101_000 == 0b001_000 {
            self.xor(0b011)
        } else {
            self.xor(0b101)
        }
    }

    /// Helper function for unwrapping an [`Option`](core::option::Option) if it
    /// actually exists. If it does not, it calls [`unreachable_semichecked`],
    /// either panicking or producing Undefined Behaviour depending on compiler
    /// optimisation flags.
    const fn const_unwrap_semichecked(x: Option<Self>) -> Self {
        debug_assert!(
            x.is_some(),
            "This value does not correspond to any cube surface point!"
        );
        if let Some(y) = x {
            y
        } else {
            unreachable_semichecked()
        }
    }

    /// Transforms its argument into a [`CubeSurfacePoint`], if it is within
    /// bounds. If it is not, it either panics or UBs depending on compiler
    /// optimisation flags. (See also [`unreachable_semichecked`].)
    const fn probs_from_u8(x: u8) -> Self {
        Self::const_unwrap_semichecked(Self::try_from_u8(x))
    }

    /// Checks if its argument is within the limit of legal values for
    /// [`CubeSurfacePoint`]s, and if so returns the `CubeSurfacePoint` to which
    /// it corresponds.
    ///
    /// Much more legible than (its alternative)[`try_from_u8_unreadable`], but
    /// uses `unsafe`. The two versions are nonetheless functionally identical:
    /// they both compile to the same assembly, ie a comparison and a no-op.
    #[cfg(not(debug_assertions))]
    const fn _try_from_u8_readable(x: u8) -> Option<Self> {
        if x < 48 {
            unsafe { core::mem::transmute(x) }
        } else {
            None
        }
    }

    /// Checks if its argument is within the limit of legal values for
    /// [`CubeSurfacePoint`]s, and if so returns the `CubeSurfacePoint` to which
    /// it corresponds.
    ///
    /// Fully safe, utterly unreadable, and functionally identical to
    /// `(x < 48).then (|| unsafe { core::mem::transmute(x) } )`—both versions
    /// compile to the same assembly, ie a comparison and a no-op.
    #[rustfmt::skip]
    #[inline(always)]
    const fn try_from_u8_unreadable(x: u8) -> Option<Self> {
        macro_rules! test {
            ($($names:ident $(= $_whatever:literal)?),+ $(,)?) => {
                match x {
                    $(
                        _ if x == CubeSurfacePoint::$names as u8 => {
                            Some(CubeSurfacePoint::$names)
                        },
                    )*
                    _ => None,
                }
            };
        }

        test!(
            PosOnePosTwoPosThree, PosOnePosTwoNegThree, PosOneNegTwoPosThree,
            PosOneNegTwoNegThree, NegOnePosTwoPosThree, NegOnePosTwoNegThree,
            NegOneNegTwoPosThree, NegOneNegTwoNegThree, PosThreePosTwoPosOne,
            PosThreePosTwoNegOne, PosThreeNegTwoPosOne, PosThreeNegTwoNegOne,
            NegThreePosTwoPosOne, NegThreePosTwoNegOne, NegThreeNegTwoPosOne,
            NegThreeNegTwoNegOne, PosOnePosThreePosTwo, PosOnePosThreeNegTwo,
            PosOneNegThreePosTwo, PosOneNegThreeNegTwo, NegOnePosThreePosTwo,
            NegOnePosThreeNegTwo, NegOneNegThreePosTwo, NegOneNegThreeNegTwo,
            PosThreePosOnePosTwo, PosThreePosOneNegTwo, PosThreeNegOnePosTwo,
            PosThreeNegOneNegTwo, NegThreePosOnePosTwo, NegThreePosOneNegTwo,
            NegThreeNegOnePosTwo, NegThreeNegOneNegTwo, PosTwoPosOnePosThree,
            PosTwoPosOneNegThree, PosTwoNegOnePosThree, PosTwoNegOneNegThree,
            NegTwoPosOnePosThree, NegTwoPosOneNegThree, NegTwoNegOnePosThree,
            NegTwoNegOneNegThree, PosTwoPosThreePosOne, PosTwoPosThreeNegOne,
            PosTwoNegThreePosOne, PosTwoNegThreeNegOne, NegTwoPosThreePosOne,
            NegTwoPosThreeNegOne, NegTwoNegThreePosOne, NegTwoNegThreeNegOne,
        )
    }

    /// A helper `const` function for turning `u8`s into `CubeSurfacePoint`s.
    #[inline(always)]
    pub const fn try_from_u8(x: u8) -> Option<Self> {
        Self::try_from_u8_unreadable(x)
    }

    /// Because [`Mul`] is not `const`.
    pub const fn mul(self, rot: Rotation) -> Self {
        let rot = rot.corresponding_point as u8;
        let mut result = self;

        if rot & 0b001_000 != 0 {
            result = result.swap_z_x();
        }

        if rot & 0b010_000 != 0 {
            result = result.swap_y_z();
        }

        if rot & 0b100_000 != 0 {
            result = result.swap_x_y();
        }

        // As previously mentioned, this implements the second, first, and
        // zeroth Elementary Reflections, conditionally, using one bit-wise XOR.
        let rot = rot & 7;

        Self::probs_from_u8(result as u8 ^ rot)
    }

    /// Because [`Div`] is not `const`.
    pub const fn div(self, divisor: Self) -> Rotation {
        // No idea if this can be made faster. Was not important for our
        // use-case.
        let reciprocal = divisor.reciprocal();
        let corresponding_point = Rotation {
            corresponding_point: self,
        }
        .mul(reciprocal);
        Rotation {
            corresponding_point,
        }
    }

    /// Returns the `CubeSurfacePoint` which is positioned 90°
    /// from `self` –clockwise or anti-clockwise depending on the `const`
    /// parametre of the same name– as viewed from the edge on which both are
    /// located.
    pub const fn one_right_angle<const CLOCKWISE: bool>(self) -> Self {
        let mut result = self;
        let x = self as u8;
        let mask = if x & 0b011_000 == 0 {
            result = result.swap_x_y();
            if (x & 0b000_001 == 0) == CLOCKWISE {
                0b000_010
            } else {
                0b000_100
            }
        } else if x & 0b101_000 == 0b001_000 {
            result = result.swap_y_z();
            if (x & 0b000_100 == 0) == CLOCKWISE {
                0b000_001
            } else {
                0b000_010
            }
        } else {
            result = result.swap_z_x();
            if (x & 0b000_010 == 0) == CLOCKWISE {
                0b000_100
            } else {
                0b000_001
            }
        };
        result.xor(mask)
    }

    /// Returns the `CubeSurfacePoint` which is positioned 90° clockwise
    /// from `self`, as viewed from the edge on which both are located.
    /// ```
    /// #  {
    /// # use cube_rotations::*;
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// # fn random_point() -> CubeSurfacePoint {
    /// #   let x = rand::random::<u8>() % 48;
    /// #   x.try_into().unwrap()
    /// # }
    /// # let random_point = random_point();
    /// assert_eq!(random_point.direction(),
    ///         random_point.one_right_angle_cw().direction())
    /// # }
    /// ```
    pub const fn one_right_angle_cw(self) -> Self {
        self.one_right_angle::<true>()
    }

    /// Returns the `CubeSurfacePoint` which is positioned 90° anti-clockwise
    /// from `self`, as viewed from the edge on which both are located.
    /// ```
    /// #  {
    /// # use cube_rotations::*;
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// # fn random_point() -> CubeSurfacePoint {
    /// #   let x = rand::random::<u8>() % 48;
    /// #   x.try_into().unwrap()
    /// # }
    /// # let random_point = random_point();
    /// assert_eq!(random_point.direction(),
    ///         random_point.one_right_angle_acw().direction())
    /// # }
    /// ```
    pub const fn one_right_angle_acw(self) -> Self {
        self.one_right_angle::<false>()
    }

    /// Returns the `CubeSurfacePoint` which is positioned (90 * `n`)°
    /// from `self` –clockwise or anti-clockwise depending on the `const`
    /// parametre of the same name–, as viewed from the edge on which both are
    /// located.
    pub const fn n_right_angles<const CLOCKWISE: bool>(self, angle: u8) -> Self {
        let angle = angle & 0b11;

        let one_or_three = if CLOCKWISE { 1 } else { 3 };
        let three_or_one = if CLOCKWISE { 3 } else { 1 };

        match angle {
            0 => self,
            _ if angle == one_or_three => self.one_right_angle_cw(),
            _ if angle == three_or_one => self.one_right_angle_acw(),
            2 => self.flip_1_and_2(),
            _ => {
                debug_assert!(false, "A 2-bit number cannot be larger than 3!");
                unreachable_semichecked()
            }
        }
    }

    /// Returns the `CubeSurfacePoint` which is positioned (90 * `n`)°
    /// clockwise
    /// from `self`, as viewed from the edge on which both are located.
    ///
    /// ```
    /// #  {
    /// # use cube_rotations::*;
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// let surface_point = PosOnePosTwoPosThree;
    /// let rotated_1 = surface_point
    ///         .one_right_angle_cw()
    ///         .one_right_angle_cw();
    /// let rotated_2 = surface_point
    ///         .n_right_angles_cw(2);
    /// assert_eq!(rotated_1, rotated_2);
    /// # }
    /// ```
    pub const fn n_right_angles_cw(self, angle: u8) -> CubeSurfacePoint {
        self.n_right_angles::<true>(angle)
    }

    /// Returns the `CubeSurfacePoint` which is positioned (90 * `n`)°
    /// anti-clockwise
    /// from `self`, as viewed from the edge on which both are located.
    ///
    /// ```
    /// #  {
    /// # use cube_rotations::*;
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// let surface_point = PosOnePosTwoPosThree;
    /// let rotated_1 = surface_point
    ///         .one_right_angle_cw()
    ///         .one_right_angle_cw()
    ///         .one_right_angle_cw();
    /// let rotated_2 = surface_point
    ///         .n_right_angles_acw(1);
    /// assert_eq!(rotated_1, rotated_2);
    /// # }
    /// ```
    pub const fn n_right_angles_acw(self, angle: u8) -> CubeSurfacePoint {
        self.n_right_angles::<false>(angle)
    }

    /// Returns the spatial direction towards which `self` is oriented.
    ///
    /// Essentially finds which coördinate has an absolute value of 3, and
    /// judges by that.
    ///
    /// ```
    /// #  {
    /// # use cube_rotations::CubeSurfacePoint::*;
    /// # use cube_rotations::Direction::*;
    /// assert_eq!(PosOnePosTwoPosThree.direction(), Up);
    /// assert_eq!(PosOnePosTwoNegThree.direction(), Down);
    /// assert_eq!(PosOnePosThreePosTwo.direction(), Front);
    /// assert_eq!(PosOneNegThreePosTwo.direction(), Back);
    /// assert_eq!(PosThreePosTwoPosOne.direction(), Right);
    /// assert_eq!(NegThreePosTwoPosOne.direction(), Left);
    /// # }
    /// ```
    pub const fn direction(self) -> Direction {
        use Direction::*;
        let x = self as u8;
        if x & 0b011_000 == 0 {
            [Down, Up][(x & 0b000_001 == 0) as usize]
        } else if x & 0b101_000 == 0b001_000 {
            [Left, Right][(x & 0b000_100 == 0) as usize]
        } else {
            [Back, Front][(x & 0b000_010 == 0) as usize]
        }
    }
}

/// Describes towards which direction the face of a cube is oriented.
///
/// ```
/// #  {
/// # use cube_rotations::CubeSurfacePoint::*;
/// # use cube_rotations::Direction::*;
/// assert_eq!(PosOnePosTwoPosThree.direction(), Up);
/// # }
/// ```
#[derive(PartialEq, Eq, Debug, Clone, Copy, PartialOrd, Ord)]
pub enum Direction {
    /// Towards the negative `x` half-axis.
    Left,
    /// Towards the positive `x` half-axis.
    Right,
    /// Towards the negative `y` half-axis.
    Back,
    /// Towards the positive `y` half-axis.
    Front,
    /// Towards the negative `z` half-axis.
    Down,
    /// Towards the positive `z` half-axis.
    Up,
}

/// A trait that groups the common behaviour of all possible data-types that
/// represent points on the surface of a cube.
///
/// Those can be [`CubeSurfacePoint`]s, [`ReferenceGroupPoint`]s,
///  [`OppositeGroupPoint`]s, or their LUT equivalents.
pub trait OctahedrallySymmetricPoint:
    Into<CubeSurfacePoint>
    + TryFrom<CubeSurfacePoint>
    + Div<Self>
    + Mul<ProperRotation, Output = Self>
    + Mul<ImproperRotation, Output = Self::OtherGroup>
    + MulAssign<ProperRotation>
{
    /// The geometric group mirror to the one to which `self` belongs.
    type OtherGroup;

    /// Returns the other point found in the same edge of the same face of the
    /// cube.
    fn beside(self) -> Self::OtherGroup;

    /// Returns the point in the corresponding position of the opposite face of
    /// the cube.
    fn opposite(self) -> Self::OtherGroup;

    /// Functionally identical to `self.opposite().beside()`. Also available
    /// with the words swapped, as the order doesn't matter.
    ///
    /// Corresponds to a rotation of 180° as seen from the face to which the
    /// point is _closest_ without being _on_.
    fn opposite_then_beside(self) -> Self;

    /// Returns the point which is positioned 90° clockwise
    /// from `self`, as viewed from the face on which both are located.
    fn one_right_angle_cw(self) -> Self;

    /// Returns the point which is positioned 90° anti-clockwise
    /// from `self`, as viewed from the face on which both are located.
    fn one_right_angle_acw(self) -> Self;

    /// Returns the point which is positioned (n * 90)° clockwise
    /// from `self`, as viewed from the face on which both are located.
    fn n_right_angles_cw(self, angle: u8) -> Self;

    /// Returns the point which is positioned (n * 90)° anti-clockwise
    /// from `self`, as viewed from the face on which both are located.
    fn n_right_angles_acw(self, angle: u8) -> Self;

    /// Flips the sign of the coördinate whose absolute value is 2.
    fn flip_sign_of_2(self) -> Self::OtherGroup;

    /// Flips the sign of the coördinates whose absolute values are 2 and 3.
    ///
    /// Corresponds to a rotation of 180° as seen from a face which the
    /// point is neither _on_ nor _close to_.
    fn flip_2_and_3(self) -> Self;

    //-------------------------Provided methods---------------------------------

    /// Returns the spatial direction towards which `self` is oriented.
    fn direction(self) -> Direction {
        Into::<CubeSurfacePoint>::into(self).direction()
    }

    /// Counts how many ones there are in the binary representation of a certain
    /// point, so it can be judged in which geometric group it belongs.
    fn odd_ones(self) -> bool {
        Into::<CubeSurfacePoint>::into(self).odd_ones()
    }

    /// Equal to the negation of [`Self::odd_ones`] by definition.
    fn even_ones(self) -> bool {
        !self.odd_ones()
    }

    /// Returns the point which is positioned (n * 90)°
    /// from `self`, as viewed from the face on which both are located.
    fn n_right_angles<const CLOCKWISE: bool>(self, angle: u8) -> Self {
        if CLOCKWISE {
            self.n_right_angles_cw(angle)
        } else {
            self.n_right_angles_acw(angle)
        }
    }

    /// Different name for [`CubeSurfacePoint::opposite_then_beside`].
    ///
    /// Corresponds to a rotation of 180° as seen from the face to which the
    /// point is _closest_ without being _on_.
    fn beside_then_opposite(self) -> Self {
        self.opposite_then_beside()
    }

    /// Different name for [`CubeSurfacePoint::opposite_then_beside`].
    ///
    /// Corresponds to a rotation of 180° as seen from the face to which the
    /// point is _closest_ without being _on_.
    fn flip_1_and_3(self) -> Self {
        self.opposite_then_beside()
    }

    /// Different name for [`CubeSurfacePoint::beside`]
    fn flip_sign_of_1(self) -> Self::OtherGroup {
        self.beside()
    }

    /// Different name for [`CubeSurfacePoint::opposite`]
    fn flip_sign_of_3(self) -> Self::OtherGroup {
        self.opposite()
    }

    /// Flips the sign of the coördinates whose absolute values are 1 and 2.
    ///
    /// Corresponds to a rotation of 180° as seen from the face on which the
    /// point is located.
    fn flip_1_and_2(self) -> Self {
        self.n_right_angles_cw(2)
    }
}

/// Rotates a copy of `self` in a way that maintains its Geometric Group.
impl Mul<ProperRotation> for CubeSurfacePoint {
    /// The Geometric Group doesn't change. Even if it did, this data-type is
    /// group-agnostic.
    type Output = CubeSurfacePoint;
    fn mul(self, rotation: ProperRotation) -> Self {
        self * Into::<Rotation>::into(rotation)
    }
}

/// Rotates `self` in a way that maintains its Geometric Group.
impl MulAssign<ProperRotation> for CubeSurfacePoint {
    /// Neither the Geometric Group nor the data-type change, so the result can
    /// be directly assigned.
    fn mul_assign(&mut self, x: ProperRotation) {
        *self = *self * x;
    }
}

/// Rotates a copy of `self` in a way that switches its Geometric Group.
impl Mul<ImproperRotation> for CubeSurfacePoint {
    /// Although the Geometric Group does change, `CubeSurfacePoint` does not
    /// change its data-type depending on Geometric Group. Thus the data-type
    /// remains the same.
    type Output = CubeSurfacePoint;
    fn mul(self, rotation: ImproperRotation) -> Self {
        self * Into::<Rotation>::into(rotation)
    }
}

/// Rotates `self` in a way that switches its Geometric Group.
impl MulAssign<ImproperRotation> for CubeSurfacePoint {
    /// The data-type remains the same, despite the Geometric Group changing.
    /// Thus, the result can be directly assigned.
    fn mul_assign(&mut self, x: ImproperRotation) {
        *self = *self * x;
    }
}

macro_rules! implement_trivial {
    ($fn: ident, $out: ty $(, $angle: ident)?) => {
        fn $fn(self $(, $angle:u8)?) -> $out {
            self.$fn($($angle)?)
        }
    };

    (everything; $us: ty, $others: ty) => {

        impl OctahedrallySymmetricPoint for $us {
            type OtherGroup = $others;

            implement_trivial!(beside, Self::OtherGroup);
            implement_trivial!(opposite, Self::OtherGroup);
            implement_trivial!(opposite_then_beside, Self);
            implement_trivial!(one_right_angle_cw, Self);
            implement_trivial!(one_right_angle_acw, Self);
            implement_trivial!(flip_sign_of_2, Self::OtherGroup);
            implement_trivial!(flip_2_and_3, Self);
            implement_trivial!(direction, Direction);
            implement_trivial!(n_right_angles_cw, Self, angle);
            implement_trivial!(n_right_angles_acw, Self, angle);
        }
    }
}

implement_trivial!(everything; CubeSurfacePoint, CubeSurfacePoint);
implement_trivial!(
    everything;
    luts::CubeSurfacePoint<true>,
    luts::CubeSurfacePoint<true>
);
implement_trivial!(
    everything;
    luts::CubeSurfacePoint<false>,
    luts::CubeSurfacePoint<false>
);
implement_trivial!(
    everything;
    luts::ReferenceGroupPoint,
    luts::OppositeGroupPoint
);
implement_trivial!(
    everything;
    luts::OppositeGroupPoint,
    luts::ReferenceGroupPoint
);

/// The set of `CubeSurfacePoint`s that can be made to coincide with the
/// Reference Point using just one *proper* rotation.
///
/// Not coincidentally, their
/// binary representations all have an even number of ones.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
#[repr(u8)]
pub enum ReferenceGroupPoint {
    /// The point `[1, 2, 3]`. Also the Point of Reference. Divided by itself,
    /// it yields –unsurprisingly enough– the identity operation.
    PosOnePosTwoPosThree = 00,
    /// The point `[1, -2, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `x` axis.
    PosOneNegTwoNegThree = 03,
    /// The point `[-1, 2, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `y` axis.
    NegOnePosTwoNegThree = 05,
    /// The point `[-1, -2, 3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `z` axis.
    NegOneNegTwoPosThree = 06,
    /// The point `[3, 2, -1]`. Divided by the Reference Point, it yields a
    /// rotation of 90° around the `y` axis.
    PosThreePosTwoNegOne = 09,
    /// The point `[3, -2, 1]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `y = 0, z = x` axis.
    PosThreeNegTwoPosOne = 10,
    /// The point `[-3, 2, 1]`. Divided by the Reference Point, it yields a
    /// rotation of -90° around the `y` axis.
    NegThreePosTwoPosOne = 12,
    /// The point `[-3, -2, -1]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `y = 0, z = -x` axis.
    NegThreeNegTwoNegOne = 15,
    /// The point `[1, 3, -2]`. Divided by the Reference Point, it yields a
    /// rotation of -90° around the `x` axis.
    PosOnePosThreeNegTwo = 17,
    /// The point `[1, -3, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 90° around the `x` axis.
    PosOneNegThreePosTwo = 18,
    /// The point `[-1, 3, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `x = 0, y = z` axis.
    NegOnePosThreePosTwo = 20,
    /// The point `[-1, -3, -2]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `x = 0, y = -z` axis.
    NegOneNegThreeNegTwo = 23,
    /// The point `[3, 1, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = y = z` axis.
    PosThreePosOnePosTwo = 24,
    /// The point `[3, -1, -2]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = -y = z` axis.
    PosThreeNegOneNegTwo = 27,
    /// The point `[-3, 1, -2]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = y = -z` axis.
    NegThreePosOneNegTwo = 29,
    /// The point `[-3, -1, 2]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = -y = -z` axis.
    NegThreeNegOnePosTwo = 30,
    /// The point `[2, 1, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `z = 0, x = y` axis.
    PosTwoPosOneNegThree = 33,
    /// The point `[2, -1, 3]`. Divided by the Reference Point, it yields a
    /// rotation of -90° around the `z` axis.
    PosTwoNegOnePosThree = 34,
    /// The point `[-2, 1, 3]`. Divided by the Reference Point, it yields a
    /// rotation of 90° around the `z` axis.
    NegTwoPosOnePosThree = 36,
    /// The point `[-2, -1, -3]`. Divided by the Reference Point, it yields a
    /// rotation of 180° around the `z = 0, x = -y` axis.
    NegTwoNegOneNegThree = 39,
    /// The point `[2, 3, 1]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = y = z` axis.
    PosTwoPosThreePosOne = 40,
    /// The point `[2, -3, -1]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = y = -z` axis.
    PosTwoNegThreeNegOne = 43,
    /// The point `[-2, 3, -1]`. Divided by the Reference Point, it yields a
    /// rotation of -120° around the `x = -y = -z` axis.
    NegTwoPosThreeNegOne = 45,
    /// The point `[-2, -3, 1]`. Divided by the Reference Point, it yields a
    /// rotation of 120° around the `x = -y = z` axis.
    NegTwoNegThreePosOne = 46,
}

/// The set of `CubeSurfacePoint`s that can be made to coincide with the
/// Reference Point using just one *improper* rotation.
///
/// Not coincidentally, their
/// binary representations all have an odd number of ones.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
#[repr(u8)]
pub enum OppositeGroupPoint {
    /// The point `[1, 2, -3]`. Divided by the Reference Point, it yields a
    /// reflection through the `z = 0` plane. Its arithmetic representation is
    /// equal to 2<sup>0</sup>, thus it is the zeroth Elementary Reflection.
    PosOnePosTwoNegThree = 01,
    /// The point `[1, -2, 3]`. Divided by the Reference Point, it yields a
    /// reflection through the `y = 0` plane. Its arithmetic representation is
    /// equal to 2<sup>1</sup>, thus it is the first Elementary Reflection.
    PosOneNegTwoPosThree = 02,
    /// The point `[-1, 2, 3]`. Divided by the Reference Point, it yields a
    /// reflection through the `x = 0` plane. Its arithmetic representation is
    /// equal to 2<sup>2</sup>, thus it is the second Elementary Reflection.
    NegOnePosTwoPosThree = 04,
    /// The point `[-1, -2, -3]`. Divided by the Reference Point, it yields a
    /// complete central inversion, ie a negation of all coördinates.
    NegOneNegTwoNegThree = 07,
    /// The point `[3, 2, 1]`. Divided by the Reference Point, it yields a
    /// reflection through the `x = z` plane. Its arithmetic representation is
    /// equal to 2<sup>3</sup>, thus it is the third Elementary Reflection.
    PosThreePosTwoPosOne = 08,
    /// The point `[3, -2, -1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 90° with respect to the `y` axis.
    PosThreeNegTwoNegOne = 11,
    /// The point `[-3, 2, -1]`. Divided by the Reference Point, it yields a
    /// reflection through the `z = -x` plane.
    NegThreePosTwoNegOne = 13,
    /// The point `[-3, -2, 1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -90° with respect to the `y` axis.
    NegThreeNegTwoPosOne = 14,
    /// The point `[1, 3, 2]`. Divided by the Reference Point, it yields a
    /// reflection through the `z = y` plane. Its arithmetic representation is
    /// equal to 2<sup>4</sup>, thus it is the fourth Elementary Reflection.
    PosOnePosThreePosTwo = 16,
    /// The point `[1, -3, -2]`. Divided by the Reference Point, it yields a
    /// reflection through the `y = -z` plane.
    PosOneNegThreeNegTwo = 19,
    /// The point `[-1, 3, -2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -90° with respect to the `x = y = z` axis.
    NegOnePosThreeNegTwo = 21,
    /// The point `[-1, -3, 2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 90° with respect to the `x` axis.
    NegOneNegThreePosTwo = 22,
    /// The point `[3, 1, -2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = -y = -z` axis.
    PosThreePosOneNegTwo = 25,
    /// The point `[3, -1, 2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = y = -z` axis.
    PosThreeNegOnePosTwo = 26,
    /// The point `[-3, 1, 2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = -y = z` axis.
    NegThreePosOnePosTwo = 28,
    /// The point `[-3, -1, -2]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = y = z` axis.
    NegThreeNegOneNegTwo = 31,
    /// The point `[2, 1, 3]`. Divided by the Reference Point, it yields a
    /// reflection through the `y = x` plane. Its arithmetic representation is
    /// equal to 2<sup>5</sup>, thus it is the fifth and final
    /// Elementary Reflection.
    PosTwoPosOnePosThree = 32,
    /// The point `[2, -1, -3]`. Divided by the Reference Point, it yields a
    /// reflection through the `x = -y` plane.
    PosTwoNegOneNegThree = 35,
    /// The point `[-2, 1, -3]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 90° with respect to the `z` axis.
    NegTwoPosOneNegThree = 37,
    /// The point `[-2, -1, 3]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -90° with respect to the `z` axis.
    NegTwoNegOnePosThree = 38,
    /// The point `[2, 3, -1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = -y = z` axis.
    PosTwoPosThreeNegOne = 41,
    /// The point `[2, -3, 1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = -y = -z` axis.
    PosTwoNegThreePosOne = 42,
    /// The point `[-2, 3, 1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of -60° with respect to the `x = y = -z` axis.
    NegTwoPosThreePosOne = 44,
    /// The point `[-2, -3, -1]`. Divided by the Reference Point, it yields a
    /// rotoreflection of 60° with respect to the `x = y = z` axis.
    NegTwoNegThreeNegOne = 47,
}

impl ReferenceGroupPoint {
    const fn to_u8(self) -> u8 {
        self as u8
    }
}

impl OppositeGroupPoint {
    const fn to_u8(self) -> u8 {
        self as u8
    }
}

macro_rules! implement {
    ($fnam: ident, $out: ty, $check: ident $(, $angle: ident)? $(; $const:tt)?) => {
        #[doc = concat!(
            "Please refer to the [function of the same name
            ](CubeSurfacePoint::",
            stringify!($fnam),
            ") in [`CubeSurfacePoint`]."
        )]
        $(pub $const)? fn $fnam(self $(, $angle:u8)?) -> $out {
            let Ok(result) = self.downcast().$fnam($($angle)?).$check() else {
                unreachable_semichecked()
            };
            result
        }
    };

    (mul; $rot: ty, $us: ty, $out: ty, $check: ident, $fnam:ident $(; $const:tt)?) => {


        #[doc = concat!(
            "Multiplies one [`",
                        stringify!($us),
                        "`] with one [`",
                        stringify!($rot),
                        "`], producing one [`",
                        stringify!($out),
                        "`] as a result. "
        )]
        /// Useful for static confirmation of Geometric Groups.
        $(pub $const)? fn $fnam(self, other: $rot) -> $out {
            let a: CubeSurfacePoint = self.downcast();
            let b: Rotation = Rotation {
                corresponding_point: other.corresponding_point.downcast()
            };
            let Ok(result) = a.mul(b).$check() else {
                unreachable_semichecked()
            };
            result
        }
    };

    (div; $us: ty, $other: ty, $out: ty, $check: ident, $fnam: ident $(; $const:tt)?) =>
    {

        #[doc = concat!(
            "Divides one [`",
            stringify!($us),
                        "`] by one [`",
                        stringify!($other),
                        "`], producing one [`",
                        stringify!($out),
                        "`] as a result. "
        )]
        /// Useful for static confirmation of Geometric Group.
        $(pub $const)? fn $fnam(self, other: $other) -> $out {
            let a: CubeSurfacePoint = self.downcast();
            let b: CubeSurfacePoint = other.downcast();
            let Ok(result) = a.div(b).$check() else {
                unreachable_semichecked()
            };
            result
        }

    };

    (everything; $us: ty, $others: ty, $check: ident, $anticheck: ident) => {
        impl $us {
            #[doc = concat!("Down-casts one [`", stringify!($us), "`] to a [`CubeSurfacePoint`].")]
            pub const fn downcast(self) -> CubeSurfacePoint {
                CubeSurfacePoint::probs_from_u8(self.to_u8())
            }

            implement!(beside, $others, $anticheck; const);
            implement!(opposite, $others, $anticheck; const);
            implement!(opposite_then_beside, Self, $check; const);
            implement!(one_right_angle_cw, Self, $check; const);
            implement!(one_right_angle_acw, Self, $check; const);
            implement!(flip_sign_of_2, $others, $anticheck; const);
            implement!(flip_2_and_3, Self, $check; const);
            implement!(n_right_angles_cw, Self, $check, angle; const);
            implement!(n_right_angles_acw, Self, $check, angle; const);
            implement!(mul; ProperRotation, $us, $us, $check, mul_prop; const);
            implement!(mul; ImproperRotation, $us, $others, $anticheck, mul_improp; const);
            implement!(div; $us, $us, ProperRotation, determine_group, div_prop; const);
            implement!(div; $us, $others, ImproperRotation, determine_antigroup, div_improp; const);

            /// Please refer to
            /// [the function of the same name](CubeSurfacePoint::direction)
            /// in [`CubeSurfacePoint`].
            pub const fn direction(self) -> Direction {
                self.downcast().direction()
            }
        }

        /// Rotates a copy of `self` in a way that maintains its Geometric Group.
        impl Mul<ProperRotation> for $us {
            /// Same Geometric Group, so same data-type.
            type Output = Self;
            implement!(mul; ProperRotation, $us, $us, $check, mul);
        }

        /// Rotates `self` in a way that maintains its Geometric Group.
        impl MulAssign<ProperRotation> for $us {
            /// The data-type doesn't change, so the result can be directly
            /// assigned.
            fn mul_assign (&mut self, x: ProperRotation) {
                *self = *self * x;
            }
        }

        /// Rotates a copy the argument in a way that maintains its Geometric
        /// Group.
        impl Mul<$us> for ProperRotation {
            /// Same Geometric Group, so same data-type.
            type Output = $us;
            fn mul(self, x: $us) -> Self::Output {
                x * self
            }
        }

        /// Rotates a copy of `self` in a way that switches its Geometric Group.
        impl Mul<ImproperRotation> for $us {
            /// Output belongs to the other Geometric Group.
            type Output = $others;
            implement!(mul; ImproperRotation, $us, $others, $anticheck, mul);
        }

        /// Rotates a copy the argument in a way that switches its Geometric
        /// Group.
        impl Mul<$us> for ImproperRotation {
            /// Output belongs to the other Geometric Group.
            type Output = $others;
            fn mul(self, x: $us) -> Self::Output {
                x * self
            }
        }

        /// Extracts the proper rotation that must occur so that the `divisor`
        /// point ends up coinciding with `self`, ie the dividend.
        impl Div for $us {
            /// A proper rotation is enough for this operation.
            type Output = ProperRotation;
            implement!(div; Self, Self, ProperRotation, determine_group, div);
        }

        /// Extracts the improper rotation that must occur so that the `divisor`
        /// point ends up coinciding with `self`, ie the dividend.
        impl Div<$others> for $us {
            /// This operation needs an improper rotation.
            type Output = ImproperRotation;

            implement!(div; Self, $others, ImproperRotation, determine_antigroup, div);
        }

        /// Discards any knowledge of Geometric Group, producing a general
        /// `CubeSurfacePoint`.
        impl From<$us> for CubeSurfacePoint {
            fn from(x: $us) -> Self {
                x.downcast()
            }
        }

        /// Please refer to [`CubeSurfacePoint::determine_group`].
        impl TryFrom<CubeSurfacePoint> for $us {
            #[doc = concat!(
            "If a certain [`CubeSurfacePoint`] does not belong to the [`",
                            stringify!($us),
                            "`]s, it must by necessity belong to the [`",
                            stringify!($others),
                            "`]s."
            )]
            type Error = $others;

            fn try_from(x: CubeSurfacePoint) -> Result<Self, Self::Error> {
                x.$check()
            }
        }

        impl OctahedrallySymmetricPoint for $us {
            type OtherGroup = $others;

            implement!(beside, Self::OtherGroup, $anticheck);
            implement!(opposite, Self::OtherGroup, $anticheck);
            implement!(flip_sign_of_2, Self::OtherGroup, $anticheck);
            implement!(opposite_then_beside, Self, $check);
            implement!(flip_2_and_3, Self, $check);
            implement!(one_right_angle_cw, Self, $check);
            implement!(one_right_angle_acw, Self, $check);
            implement!(n_right_angles_cw, Self, $check, angle);
            implement!(n_right_angles_acw, Self, $check, angle);
        }


    };
}

implement!(everything; ReferenceGroupPoint, OppositeGroupPoint, determine_group, determine_antigroup);
implement!(everything; OppositeGroupPoint, ReferenceGroupPoint, determine_antigroup, determine_group);

/// As per a [`Rotation`], but one that specifically needs no reflections.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct ProperRotation {
    corresponding_point: ReferenceGroupPoint,
}

/// As per a [`Rotation`], but one that specifically needs at least one
/// reflection.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct ImproperRotation {
    corresponding_point: OppositeGroupPoint,
}

impl Rotation {
    const fn determine_group(self) -> Result<ProperRotation, ImproperRotation> {
        let argh = self.corresponding_point.determine_group();
        match argh {
            Ok(corresponding_point) => Ok(ProperRotation {
                corresponding_point,
            }),
            Err(corresponding_point) => Err(ImproperRotation {
                corresponding_point,
            }),
        }
    }

    const fn determine_antigroup(self) -> Result<ImproperRotation, ProperRotation> {
        match self.determine_group() {
            Ok(a) => Err(a),
            Err(a) => Ok(a),
        }
    }
}

macro_rules! mul_div_rots {
    ($us: ty, $other: ty, $output: path) => {
        /// Implements multiplication between rotations, such that
        /// `rot_a * rot_b * point_x` can be computed as either
        /// `rot_a * (rot_b * point_x)` or `(rot_a * rot_b) * point_x`,
        /// with no change to the result computed.
        ///
        /// Important note: `Rotation`s are essentially highly simplified
        /// matrices. This means that, in the general case, commutativity _does
        /// not hold_, and therefore `rot_a * rot_b` and `rot_b * rot_a` are not
        /// necessarily equal.
        impl Mul<$other> for $us {
            /// The output type tells us all we know about the propriety of the
            /// result.
            type Output = $output;

            /// Implemented by multiplying the second rotation's corresponding
            /// point with the first one.
            fn mul(self, other: $other) -> Self::Output {
                let corresponding_point = self * other.corresponding_point;
                $output {
                    corresponding_point,
                }
            }
        }

        /// Implements division between rotations, such that
        /// `(rot_a * rot_b) / rot_b == rot_a`.
        ///
        /// Important note: `rot_a / rot_b` is not necessarily equal to
        /// `(1 / rot_b) * rot_a`. See also the relevant comment on `Mul`.
        impl Div<$other> for $us {
            /// The output type tells us all we know about the propriety of the
            /// result.
            type Output = $output;

            /// Implemented by dividing the two corresponding points
            /// with one another.
            fn div(self, other: $other) -> Self::Output {
                self.corresponding_point / other.corresponding_point
            }
        }
    };
}

mul_div_rots!(Rotation, Rotation, Rotation);
mul_div_rots!(ProperRotation, ProperRotation, ProperRotation);
mul_div_rots!(ImproperRotation, ImproperRotation, ProperRotation);
mul_div_rots!(ProperRotation, ImproperRotation, ImproperRotation);
mul_div_rots!(ImproperRotation, ProperRotation, ImproperRotation);

/// Discards any notion of propriety, producing a general `Rotation`.
impl From<ProperRotation> for Rotation {
    fn from(x: ProperRotation) -> Self {
        let c_p = x.corresponding_point as u8;
        Self {
            corresponding_point: CubeSurfacePoint::probs_from_u8(c_p),
        }
    }
}

/// Discards any notion of propriety, producing a general `Rotation`.
impl From<ImproperRotation> for Rotation {
    fn from(x: ImproperRotation) -> Self {
        let c_p = x.corresponding_point as u8;
        Self {
            corresponding_point: CubeSurfacePoint::probs_from_u8(c_p),
        }
    }
}

/// Discriminates a `Rotation` based on impropriety.
impl TryFrom<Rotation> for ImproperRotation {
    /// If a [`Rotation`] is not improper, it must by necessity be proper.
    type Error = ProperRotation;
    fn try_from(x: Rotation) -> Result<Self, Self::Error> {
        x.determine_antigroup()
    }
}

/// Discriminates a `Rotation` based on propriety.
impl TryFrom<Rotation> for ProperRotation {
    /// If a [`Rotation`] is not proper, it must by necessity be improper.
    type Error = ImproperRotation;
    fn try_from(x: Rotation) -> Result<Self, Self::Error> {
        x.determine_group()
    }
}

impl From<Rotation> for CubeSurfacePoint {
    fn from(x: Rotation) -> Self {
        x.corresponding_point
    }
}

impl From<ProperRotation> for ReferenceGroupPoint {
    fn from(x: ProperRotation) -> Self {
        x.corresponding_point
    }
}

impl From<ImproperRotation> for OppositeGroupPoint {
    fn from(x: ImproperRotation) -> Self {
        x.corresponding_point
    }
}

impl From<CubeSurfacePoint> for Rotation {
    fn from(corresponding_point: CubeSurfacePoint) -> Self {
        Self {
            corresponding_point,
        }
    }
}

impl From<ReferenceGroupPoint> for ProperRotation {
    fn from(corresponding_point: ReferenceGroupPoint) -> Self {
        Self {
            corresponding_point,
        }
    }
}

impl From<OppositeGroupPoint> for ImproperRotation {
    fn from(corresponding_point: OppositeGroupPoint) -> Self {
        Self {
            corresponding_point,
        }
    }
}

#[cfg(test)]
mod tests;
