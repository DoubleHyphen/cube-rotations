//! A module containing wrapper data-types whose calculations are performed
//! using **L**ook-**U**p **T**ables.
//!
//! # Usage
//!
//! ```
//! use cube_rotations::luts::CspLarge;
//! use cube_rotations::CubeSurfacePoint::*;
//! let x = CspLarge(NegThreePosOnePosTwo);
//! let y = CspLarge(PosTwoPosThreeNegOne);
//! let z = CspLarge(PosThreePosOneNegTwo);
//! let x_actual = CspLarge(PosThreePosTwoPosOne);
//! let rotation = x_actual / x;
//! let x_actual = x * rotation;
//! let y_actual = y * rotation;
//! let z_actual = z * rotation;
//! ```
//!
//! # Contents
//!
//! * [`CubeSurfacePoint::<false>`]: As per the basic
//!     [`CubeSurfacePoint`](super::CubeSurfacePoint). Each LUT used is at most 48 bytes in length,
//!     but some operations might need multiple look-ups.
//! * [`CubeSurfacePoint::<true>`]: As per the basic
//!     [`CubeSurfacePoint`](super::CubeSurfacePoint). Each LUT used is up to 2304 bytes in length,
//!     and each operation is guaranteed to consist of a single look-up.
//! * [`ReferenceGroupPoint`]: As per the basic
//!     [`ReferenceGroupPoint`](super::ReferenceGroupPoint). Each LUT used is up to 576 bytes in
//!     length, and each operation is guaranteed to consist of a single look-up.
//! * [`OppositeGroupPoint`]: As per the basic
//!     [`OppositeGroupPoint`](super::OppositeGroupPoint). Each LUT used is up to 576 bytes in
//!     length, and each operation is guaranteed to consist of a single look-up.
//!     
//!     
//! # Using LUTs for multiplication and division of rotation data-types
//!
//! The rotation data-types do not carry LUT information inside of them.
//! This means that, if they are multiplied/divided using the corresponding
//! operands, then their divisions and multiplications always occur iteratively.
//!
//! To alleviate this, the
//! [`multiply_rotations_luts`] and
//! [`divide_rotations_luts`] have been provided.
//! They need a boolean const parametre that denotes whether the operations will
//! take place using big LUTs or not. That said, they only operate on general
//! [`Rotation`]s; users needing more will need to implement their own functions
//! as per the source code available.
//!
//! With all that said, any down-stream user for whom this restriction is a
//! legitimate problem is cordially invited to inform the crate maintainer
//! posthaste.
//!
//! ### Small note
//!
//! The Geometric-Group-specific data-types contained herein only operate
//! using big LUTs. This is because the small-LUT versions of multiplication
//! and division operate, like the non-LUT versions, by conditionally performing
//! Elementary Reflections on the given point. However, a reflection immediately
//! switches the Geometric Group to which a point belongs, which means that the
//! LUTs used therein need to be 48 bytes long. Thus, multiplication and
//! division have no performance benefit compared to
//! [the Geometric-Group-agnostic data-type](CubeSurfacePoint);
//! since those operations are also the ones that need the most computational
//! resources, the implementation of small-point LUTs for
//! Geometric-Group-Specific point-types was not deemed worthwhile.
//!
//! # How to use one less big LUT
//! If one big LUT is necessary but two are too many, one can
//! get rid of the division's LUT. This can be achieved by only using `div_alt`
//! instead of the division operator; this substitutes division's big LUT for
//! a small LUT for the point's reciprocal, which is then multiplied by the
//! point in question using the remaining big LUT. The cost of this is that
//! `div_alt` takes two look-ups instead of just one.
//!
//! It is not yet known whether `div_alt` deserves to also exist for
//! `OppositeGroupPoint`s.
use crate::CubeSurfacePoint as CratePt;
use crate::Direction;
use crate::Rotation;
use core::convert::identity as id;
use core::ops::{Div, Mul, MulAssign};

/// A type synonym for convenience.
/// ```
/// use cube_rotations::CubeSurfacePoint::*;
/// use cube_rotations::luts::*;
/// let c: CspLarge = CspLarge(PosOnePosTwoPosThree);
/// // This line was not trivial to make functional, but we did it after all.
/// ```
pub type CspLarge = CubeSurfacePoint<true>;

#[doc(hidden)]
#[allow(non_snake_case)]
pub const fn CspLarge(x: CratePt) -> CspLarge {
    CubeSurfacePoint::<true>(x)
}

/// A type synonym for convenience.
/// ```
/// use cube_rotations::CubeSurfacePoint::*;
/// use cube_rotations::luts::*;
/// let c: CspSmall = CspSmall(PosThreePosTwoPosOne);
/// // This line was not trivial to make functional, but we did it after all.
/// ```
pub type CspSmall = CubeSurfacePoint<false>;

#[doc(hidden)]
#[allow(non_snake_case)]
pub const fn CspSmall(x: CratePt) -> CspSmall {
    CubeSurfacePoint::<false>(x)
}

/// A type synonym for convenience.
/// ```
/// use cube_rotations::ReferenceGroupPoint::*;
/// use cube_rotations::luts::*;
/// let c: RgpLarge = RgpLarge(PosOnePosTwoPosThree);
/// // This line was not trivial to make functional, but we did it after all.
/// ```
pub type RgpLarge = ReferenceGroupPoint;

#[doc(hidden)]
#[allow(non_snake_case)]
pub const fn RgpLarge(x: crate::ReferenceGroupPoint) -> RgpLarge {
    ReferenceGroupPoint(x)
}

/// A type synonym for convenience.
/// ```
/// use cube_rotations::OppositeGroupPoint::*;
/// use cube_rotations::luts::*;
/// let c: OgpLarge = OgpLarge(PosThreePosTwoPosOne);
/// // This line was not trivial to make functional, but we did it after all.
/// ```
pub type OgpLarge = OppositeGroupPoint;

#[doc(hidden)]
#[allow(non_snake_case)]
pub const fn OgpLarge(x: crate::OppositeGroupPoint) -> OgpLarge {
    OppositeGroupPoint(x)
}

macro_rules! with_lut {
    ($truefalse: expr; $fnam: ident $(; $vis:vis)?) => {

        #[doc = concat!(
        "Please refer to the [function of the same name](crate::",
                        stringify!(CubeSurfacePoint),
                        "::",
                        stringify!($fnam),
                        ") in the basic [`",
                        stringify!(CubeSurfacePoint),
                        "`](crate::",
                        stringify!(CubeSurfacePoint),
                        ")."
        )]
        $($vis)? const fn $fnam(self) -> Self {
            type Csp = CubeSurfacePoint::<{ $truefalse }>;

            const fn from_serial_number(x: u8) -> Csp {
                CubeSurfacePoint::<{ $truefalse }> (
                    CratePt::probs_from_u8(x).$fnam()
                )
            }

            const LUT: [Csp; 48] = {
                let mut result = [Csp::REFERENCE_POINT; 48];
                let mut i = 48;
                while i > 0 {
                    i -= 1;
                    result[i] = from_serial_number(i as u8);
                }
                result
            };

            LUT[self.0 as usize]
        }

    };

    (true; $name: ident, $($names:ident),+ $(; $vis:vis)?) => {
        with_lut!(true; $name $(; $vis)?);
        with_lut!(true; $($names),+ $(; $vis)?);
    };

    (false; $name: ident, $($names:ident),+ $(; $vis:vis)?) => {
        with_lut!(false; $name $(; $vis)?);
        with_lut!(false; $($names),+ $(; $vis)?);
    };
}

/// A helper wrapper-type around a [`CubeSurfacePoint`] that operates using
/// look-up tables.
///
/// A `luts::CubeSurfacePoint::<false>` only has LUTs 48
/// bytes long, but might need more than one look-up for each operation. A
/// `luts::CubeSurfacePoint::<true>`, in contrast, has LUTs up to 2304 bytes
/// long, but only ever needs one look-up per operation.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct CubeSurfacePoint<const BIG_LUTS: bool>(pub crate::CubeSurfacePoint);

impl<const BIG_LUTS: bool> CubeSurfacePoint<BIG_LUTS> {
    /// Please refer to the ordinary
    /// [`REFERENCE_POINT`](CubeSurfacePoint::REFERENCE_POINT).
    pub const REFERENCE_POINT: Self = Self(CratePt::REFERENCE_POINT);

    #[doc = concat!(
    "Please refer to the [function of the same name]",
    "(crate::CubeSurfacePoint::direction", ") in [`CubeSurfacePoint`]."
    )]
    pub const fn direction(self) -> Direction {
        const fn from_serial_number(x: u8) -> Direction {
            CratePt::probs_from_u8(x).direction()
        }

        const LUT: [Direction; 48] = {
            let ref_pt = CratePt::REFERENCE_POINT;
            let mut result = [ref_pt.direction(); 48];
            let mut i = 48;
            while i > 0 {
                i -= 1;
                result[i] = from_serial_number(i as u8);
            }
            result
        };
        LUT[self.0 as usize]
    }

    const fn probs_from_u8(x: u8) -> Self {
        Self(CratePt::probs_from_u8(x))
    }
}

impl<const BIG_LUTS: bool> From<CratePt> for CubeSurfacePoint<BIG_LUTS> {
    fn from(x: CratePt) -> Self {
        Self(x)
    }
}

impl<const BIG_LUTS: bool> From<CubeSurfacePoint<BIG_LUTS>> for CratePt {
    fn from(x: CubeSurfacePoint<BIG_LUTS>) -> Self {
        x.0
    }
}

impl From<Rotation> for CubeSurfacePoint<true> {
    fn from(x: Rotation) -> Self {
        Self(x.corresponding_point)
    }
}

impl From<Rotation> for CubeSurfacePoint<false> {
    fn from(x: Rotation) -> Self {
        Self(x.corresponding_point)
    }
}

impl CubeSurfacePoint<false> {
    with_lut!(false; one_right_angle_cw, one_right_angle_acw, beside,
                opposite, opposite_then_beside, flip_sign_of_2, flip_2_and_3; pub);
    with_lut!(false; swap_x_y, swap_y_z, swap_z_x, reciprocal);

    #[doc = concat!(
    "Please refer to the [function of the same name
    ](crate::CubeSurfacePoint::",
                    stringify!(n_right_angles),
                    ") in [`CubeSurfacePoint`](crate::CubeSurfacePoint)."
    )]
    pub const fn n_right_angles<const CLOCKWISE: bool>(self, angle: u8) -> Self {
        let mut angle = angle & 0b11;
        let mut result = self;
        while angle > 0 {
            result = if CLOCKWISE {
                result.one_right_angle_cw()
            } else {
                result.one_right_angle_acw()
            };
            angle -= 1;
        }
        result
    }

    #[doc = concat!(
    "Please refer to the [function of the same name
    ](crate::CubeSurfacePoint::",
                    stringify!(n_right_angles_acw),
                    ") in [`CubeSurfacePoint`](crate::CubeSurfacePoint)."
    )]
    pub const fn n_right_angles_acw(self, angle: u8) -> Self {
        self.n_right_angles::<false>(angle)
    }

    #[doc = concat!(
    "Please refer to the [function of the same name
    ](crate::CubeSurfacePoint::",
                    stringify!(n_right_angles_cw),
                    ") in [`CubeSurfacePoint`](crate::CubeSurfacePoint)."
    )]
    pub const fn n_right_angles_cw(self, angle: u8) -> Self {
        self.n_right_angles::<true>(angle)
    }
}

impl CubeSurfacePoint<true> {
    with_lut!(true; one_right_angle_cw, one_right_angle_acw, beside,
                opposite, opposite_then_beside, flip_sign_of_2, flip_2_and_3; pub);

    #[doc = concat!(
    "Please refer to the [function of the same name
    ](crate::CubeSurfacePoint::",
                    stringify!(n_right_angles_cw),
                    ") in [`CubeSurfacePoint`](crate::CubeSurfacePoint)."
    )]
    pub const fn n_right_angles_cw(self, angle: u8) -> Self {
        let angle = angle & 0b11;
        type Csp = CubeSurfacePoint<true>;
        const LUT: [[Csp; 48]; 4] = {
            let mut result = [[Csp::REFERENCE_POINT; 48]; 4];
            let mut i: usize = 48 * 4;
            while i > 0 {
                i -= 1;
                result[i % 4][i / 4] = CubeSurfacePoint::<true>(
                    CratePt::probs_from_u8(i as u8 / 4).n_right_angles_cw(i as u8 % 4),
                );
            }
            result
        };

        LUT[angle as usize][self.0 as usize]
    }

    #[doc = concat!(
    "Please refer to the [function of the same name
    ](crate::CubeSurfacePoint::",
                    stringify!(n_right_angles_acw),
                    ") in [`CubeSurfacePoint`](crate::CubeSurfacePoint)."
    )]
    pub const fn n_right_angles_acw(self, angle: u8) -> Self {
        let angle = angle & 0b11;
        type Csp = CubeSurfacePoint<true>;
        const LUT: [[Csp; 48]; 4] = {
            let mut result = [[Csp::REFERENCE_POINT; 48]; 4];
            let mut i: usize = 48 * 4;
            while i > 0 {
                i -= 1;
                result[i % 4][i / 4] = CubeSurfacePoint::<true>(
                    CratePt::probs_from_u8(i as u8 / 4).n_right_angles_acw(i as u8 % 4),
                );
            }
            result
        };

        LUT[angle as usize][self.0 as usize]
    }

    /// An alternative implementation of division, that uses the same big
    /// LUT as multiplication does, but performs two look-ups instead of
    /// one.
    pub fn div_alt(self, divisor: Self) -> Rotation {
        const RECIPROCALS: [CubeSurfacePoint<true>; 48] = {
            let mut i = 48;
            let ref_pt = CubeSurfacePoint::<true>::REFERENCE_POINT;
            let mut result = [ref_pt; 48];
            while i > 0 {
                i -= 1;
                let pt = CratePt::probs_from_u8(i as u8);
                let siiiigh = ref_pt.0.mul(ref_pt.0.div(pt));
                result[i] = CubeSurfacePoint::<true>(siiiigh);
            }
            result
        };

        Rotation {
            corresponding_point: (Rotation {
                corresponding_point: self.0,
            } * RECIPROCALS[divisor.0 as usize])
                .0,
        }
    }
}

/// Rotates a copy of `self` according to a `Rotation`.
impl Mul<Rotation> for CubeSurfacePoint<false> {
    /// We use the most general data-type possible, so the output does not
    /// change.
    type Output = Self;

    /// The rotation happens Elementary-Reflection-by-Elementary-Reflection
    /// as usual, but each Elementary Reflection is performed with a LUT.
    fn mul(self, rot: Rotation) -> Self::Output {
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

        let rot = rot & 7;

        Self::probs_from_u8(result.0 as u8 ^ rot)
    }
}

/// Rotates a copy of `self` according to a `Rotation`.
impl Mul<Rotation> for CubeSurfacePoint<true> {
    /// We use the most general data-type possible, so the output does not
    /// change.
    type Output = Self;

    /// The `Rotation` is not examined bit-by-bit. Instead, a
    /// look-up on a 2-D LUT produces the result directly.
    fn mul(self, rot: Rotation) -> Self::Output {
        type Csp = CubeSurfacePoint<true>;
        const fn from_serial_number(x: u16) -> Csp {
            let rot = crate::Rotation {
                corresponding_point: CratePt::probs_from_u8((x / 48) as u8),
            };
            let surface_point = CratePt::probs_from_u8((x % 48) as u8);
            CubeSurfacePoint::<true>(rot.mul(surface_point))
        }
        const LUT: [[Csp; 48]; 48] = {
            let mut result = [[Csp::REFERENCE_POINT; 48]; 48];
            let mut i = 48 * 48;
            while i > 0 {
                i -= 1;
                result[i / 48][i % 48] = from_serial_number(i as u16);
            }
            result
        };

        LUT[rot.corresponding_point as usize][self.0 as usize]
    }
}

/// Rotates `self` according to a `Rotation`.
impl MulAssign<Rotation> for CubeSurfacePoint<true> {
    /// We use the most general data-type possible, so the output does not
    /// change. Thus, it can be directly assigned.
    ///
    /// The `Rotation` is not examined bit-by-bit. Instead, a look-up on a
    /// 2-D LUT produces the result directly.
    fn mul_assign(&mut self, rot: Rotation) {
        *self = *self * rot;
    }
}

/// Rotates `self` according to a `Rotation`.
impl MulAssign<Rotation> for CubeSurfacePoint<false> {
    /// We use the most general data-type possible, so the output does not
    /// change. Thus, it can be directly assigned.
    ///
    /// The rotation happens Elementary-Reflection-by-Elementary-Reflection
    /// as usual, but each Elementary Reflection is performed with a LUT.
    fn mul_assign(&mut self, rot: Rotation) {
        *self = *self * rot;
    }
}

/// Rotates a copy of a given `CubeSurfacePoint` according to `self`.
impl Mul<CubeSurfacePoint<true>> for Rotation {
    /// Output same as input.
    type Output = CubeSurfacePoint<true>;

    /// `self` is not examined bit-by-bit. Instead, a look-up on a 2-D
    /// LUT produces the result directly.
    fn mul(self, cub_sur_pt: Self::Output) -> Self::Output {
        cub_sur_pt * self
    }
}

/// Rotates a copy of a given `CubeSurfacePoint` according to `self`.
impl Mul<CubeSurfacePoint<false>> for Rotation {
    /// Output same as input.
    type Output = CubeSurfacePoint<false>;

    /// The rotation happens Elementary-Reflection-by-Elementary-Reflection
    /// as usual, but each Elementary Reflection is performed with a LUT.
    fn mul(self, cub_sur_pt: Self::Output) -> Self::Output {
        cub_sur_pt * self
    }
}

/// Extracts the rotation that must occur so that the `divisor` point ends
/// up coinciding with `self`, ie the dividend.
impl Div for CubeSurfacePoint<false> {
    /// This rotation can be either proper or improper.
    type Output = Rotation;

    /// The operation occurs by finding the reciprocal of the `divisor`
    /// using a LUT, then rotating it according to `self`.
    /// The rotation happens Elementary-Reflection-by-Elementary-Reflection
    /// as usual, but each Elementary Reflection is performed with a LUT.
    fn div(self, divisor: Self) -> Self::Output {
        let recip = divisor.reciprocal();
        let rot = Rotation {
            corresponding_point: self.0,
        };
        Rotation {
            corresponding_point: (recip.mul(rot)).0,
        }
    }
}

/// Extracts the rotation that must occur so that the `divisor` point ends
/// up coinciding with `self`, ie the dividend.
impl Div for CubeSurfacePoint<true> {
    /// This rotation can be either proper or improper.
    type Output = Rotation;

    /// A look-up on a 2-D LUT produces the result directly.
    fn div(self, divisor: Self) -> Self::Output {
        const fn from_serial_number(x: u16) -> Rotation {
            let divisor = CratePt::probs_from_u8((x / 48) as u8);

            let point = CratePt::probs_from_u8((x % 48) as u8);
            point.div(divisor)
        }

        const LUT: [[Rotation; 48]; 48] = {
            let temp_rot = Rotation {
                corresponding_point: CratePt::REFERENCE_POINT,
            };
            let mut result = [[temp_rot; 48]; 48];
            let mut i = 48 * 48;
            while i > 0 {
                i -= 1;
                result[i % 48][i / 48] = from_serial_number(i as u16);
            }
            result
        };

        let divisor = divisor.0 as usize;
        let dividend = self.0 as usize;
        LUT[dividend][divisor]
    }
}

use crate::{ImproperRotation, ProperRotation};

/// Rotates a copy of `self` according to a `ProperRotation`.
/// Maintains Geometric Group.
impl Mul<ProperRotation> for CubeSurfacePoint<false> {
    /// The Geometric Group doesn't change. Even if it did, this data-type
    /// is group-agnostic.
    type Output = Self;

    /// The rotation happens Elementary-Reflection-by-Elementary-Reflection
    /// as usual, but each Elementary Reflection is performed with a LUT.
    fn mul(self, x: ProperRotation) -> Self {
        let rot: Rotation = x.into();
        rot * self
    }
}

/// Rotates a copy of `self` according to a `ProperRotation`.
/// Maintains Geometric Group.
impl Mul<ProperRotation> for CubeSurfacePoint<true> {
    /// The Geometric Group doesn't change. Even if it did, this data-type
    /// is group-agnostic.
    type Output = Self;

    /// The `ProperRotation` is not examined bit-by-bit. Instead, a look-up
    /// on a 2-D LUT produces the result directly.
    ///
    /// While this could have been implemented using smaller LUTs than the
    /// ones used for `Mul<Rotation>`, it was deemed a useless middle
    /// solution.
    fn mul(self, x: ProperRotation) -> Self {
        let rot: Rotation = x.into();
        rot * self
    }
}

/// Rotates a copy of `self` according to an `ImproperRotation`.
/// Switches Geometric Group.
impl Mul<ImproperRotation> for CubeSurfacePoint<false> {
    /// Although the Geometric Group does change, `CubeSurfacePoint` does
    /// not change its data-type depending on Geometric Group. Thus the
    /// data-type remains the same.
    type Output = Self;

    /// The rotation happens Elementary-Reflection-by-Elementary-Reflection
    /// as usual, but each Elementary Reflection is performed with a LUT.
    fn mul(self, x: ImproperRotation) -> Self {
        let rot: Rotation = x.into();
        rot * self
    }
}

/// Rotates a copy of `self` according to an `ImproperRotation`. Switches
/// Geometric Group.
impl Mul<ImproperRotation> for CubeSurfacePoint<true> {
    /// Although the Geometric Group does change, `CubeSurfacePoint` does not
    /// change its data-type depending on Geometric Group. Thus the data-type
    /// remains the same.
    type Output = Self;

    /// The `ImproperRotation` is not examined bit-by-bit. Instead, a
    /// look-up on a 2-D LUT produces the result directly.
    ///
    /// While this could have been implemented using smaller LUTs than the
    /// ones
    /// used for `Mul<Rotation>`, it was deemed a useless middle solution.
    fn mul(self, x: ImproperRotation) -> Self {
        let rot: Rotation = x.into();
        rot * self
    }
}

/// Rotates `self` according to a `ProperRotation`. Maintains Geometric
/// Group.
impl MulAssign<ProperRotation> for CubeSurfacePoint<true> {
    /// Neither the Geometric Group nor the data-type change, so the result
    /// can be directly assigned.
    ///
    /// The `ProperRotation` is not examined bit-by-bit. Instead, a
    /// look-up on a 2-D LUT produces the result directly.
    ///
    /// While this could have been implemented using smaller LUTs than the
    /// ones
    /// used for `Mul<Rotation>`, it was deemed a useless middle solution.
    fn mul_assign(&mut self, x: ProperRotation) {
        *self = *self * x;
    }
}

/// Rotates `self` according to an `ImproperRotation`. Switches Geometric
/// Group.
impl MulAssign<ImproperRotation> for CubeSurfacePoint<true> {
    /// The data-type remains the same, despite the Geometric Group
    /// changing. Thus, the result can be directly assigned.
    ///
    /// The `ImproperRotation` is not examined bit-by-bit. Instead, a
    /// look-up on a 2-D LUT produces the result directly.
    ///
    /// While this could have been implemented using smaller LUTs than the
    /// ones
    /// used for `Mul<Rotation>`, it was deemed a useless middle solution.
    fn mul_assign(&mut self, x: ImproperRotation) {
        *self = *self * x;
    }
}

fn mul_rots_big_luts(rot_1: Rotation, rot_2: Rotation) -> Rotation {
    let corr_point: CubeSurfacePoint<true> = rot_2.into();
    (rot_1 * corr_point).into()
}

fn div_rots_big_luts(rot_1: Rotation, rot_2: Rotation) -> Rotation {
    let corr_point_1: CubeSurfacePoint<true> = rot_1.into();
    let corr_point_2: CubeSurfacePoint<true> = rot_2.into();
    corr_point_1 / corr_point_2
}

fn mul_rots_small_luts(rot_1: Rotation, rot_2: Rotation) -> Rotation {
    let corr_point: CubeSurfacePoint<false> = rot_2.into();
    (rot_1 * corr_point).into()
}

fn div_rots_small_luts(rot_1: Rotation, rot_2: Rotation) -> Rotation {
    let corr_point_1: CubeSurfacePoint<false> = rot_1.into();
    let corr_point_2: CubeSurfacePoint<false> = rot_2.into();
    corr_point_1 / corr_point_2
}

/// Multiplies two `Rotation`s together, using LUTs.
pub fn multiply_rotations_luts<const BIG: bool>(rot_1: Rotation, rot_2: Rotation) -> Rotation {
    [mul_rots_small_luts, mul_rots_big_luts][BIG as usize](rot_1, rot_2)
}

/// Divides two `Rotation`s together, using LUTs.
pub fn divide_rotations_luts<const BIG: bool>(rot_1: Rotation, rot_2: Rotation) -> Rotation {
    [div_rots_small_luts, div_rots_big_luts][BIG as usize](rot_1, rot_2)
}

/// Rotates `self` according to a `ProperRotation`. Maintains Geometric
/// Group.
impl MulAssign<ProperRotation> for CubeSurfacePoint<false> {
    /// Neither the Geometric Group nor the data-type change, so the result
    /// can be directly assigned.
    ///
    /// The `ProperRotation` is not examined bit-by-bit. Instead, a
    /// look-up on a 2-D LUT produces the result directly.
    ///
    /// While this could have been implemented using smaller LUTs than the
    /// ones
    /// used for `Mul<Rotation>`, it was deemed a useless middle solution.
    fn mul_assign(&mut self, x: ProperRotation) {
        *self = *self * x;
    }
}

/// Rotates `self` according to a `ImproperRotation`. Switches Geometric
/// Group.
impl MulAssign<ImproperRotation> for CubeSurfacePoint<false> {
    /// The data-type remains the same, despite the Geometric Group
    /// changing. Thus, the result can be directly assigned.
    ///
    /// The `ImproperRotation` is not examined bit-by-bit. Instead, a
    /// look-up on a 2-D LUT produces the result directly.
    ///
    /// While this could have been implemented using smaller LUTs than the
    /// ones
    /// used for `Mul<Rotation>`, it was deemed a useless middle solution.
    fn mul_assign(&mut self, x: ImproperRotation) {
        *self = *self * x;
    }
}

macro_rules! with_lut {
    ($us: ty, $out: path, $fnam: ident, $convert: ident) => {

        #[doc = concat!(
        "Please refer to the [function of the same name
        ](crate::CubeSurfacePoint::",
                        stringify!($fnam),
                        ") in 
                        [`CubeSurfacePoint`](crate::CubeSurfacePoint)."
        )]
        pub const fn $fnam(self) -> $out {

            const fn from_serial_number (i: usize) -> $out {
                $convert(<$us>::index_to_self(i).0.$fnam())
            }

            const LUT: [$out; 24] = {
                let mut result = [from_serial_number(0); 24];
                let mut i = 24;
                while i > 0 {
                    i -= 1;
                    result[i] = from_serial_number(i);
                }
                result
            };

            LUT[self.0 as usize >> 1]
        }
    };

    (angles; $us: path, $fnam: ident) => {
        #[doc = concat!(
        "Please refer to the [function of the same name
        ](crate::CubeSurfacePoint::",
                        stringify!($fnam),
                        ") in 
                        [`CubeSurfacePoint`](crate::CubeSurfacePoint)."
        )]
        pub const fn $fnam(self, angle:u8) -> Self {
            let angle = angle & 0b11;

            const fn from_serial_number (i: usize) -> $us {
                $us(<$us>::index_to_self(i / 4).0.$fnam((i as u8) % 4))
            }

            const LUT: [[$us; 24]; 4] = {
                let mut result = [[from_serial_number(0); 24]; 4];
                let mut i: usize = 24 * 4;
                while i > 0 {
                    i -= 1;
                    result[i % 4][i / 4] = from_serial_number(i);
                }
                result
            };

            LUT[angle as usize][self.0 as usize >> 1]
        }
    };

    (muldiv; $us: ty, $other: ty, $out: ty, $fnam: ident, $out_to_out: ident $(; $const:tt)? $(.$siiiigh: tt)?) => {
        #[doc = concat!(
        "Please refer to the [function of the same name](crate::",
                        stringify!($us),
                        "::",
                        stringify!($fnam),
                        ") in the basic [`",
                        stringify!($us),
                        "`](crate::",
                        stringify!($us),
                        ")."
        )]
        $(pub $const)? fn $fnam(self, other: $other) -> $out {
            const fn from_serial_number(x: usize) -> $out {
                let us = <$us>::index_to_self(x / 24).0;
                let other = <$other>::index_to_self(x % 24) $(. $siiiigh)?;
                $out_to_out(us.$fnam(other))
            }

            const LUT: [[$out; 24]; 24] = {
                let mut result = [[from_serial_number(0); 24]; 24];
                let mut i = 24 * 24;
                while i > 0 {
                    i -= 1;
                    result[i / 24][i % 24] = from_serial_number(i);
                }
                result
            };

            LUT[self.0 as usize >> 1][other.self_to_index() as usize >> 1]
        }
    };

    (everything; $us: ty, $others:ty, $check: ident,
        $self_to_self:ident, $other_to_other: ident $(,)?) => {

            const fn index_to_self(x: usize) -> $us {
                let x = (x as u8) * 2;
                let pt_1 = CratePt::probs_from_u8(x);
                let pt_2 = CratePt::probs_from_u8(x ^ 1);
                let result = match (pt_1.$check(), pt_2.$check()) {
                    (Ok(point), Err(_)) => point,
                    (Err(_), Ok(point)) => point,
                    _ => crate::unreachable_semichecked(),
                };
                $self_to_self(result)
            }

            const fn self_to_index(self) -> usize {
                self.0 as usize
            }

            with_lut!($us, Direction, direction, id);
            with_lut!($us, $others, beside, $other_to_other);
            with_lut!($us, $others, opposite, $other_to_other);
            with_lut!($us, $others, flip_sign_of_2, $other_to_other);
            with_lut!($us, $us, opposite_then_beside, $self_to_self);
            with_lut!($us, $us, flip_2_and_3, $self_to_self);
            with_lut!($us, $us, one_right_angle_cw, $self_to_self);
            with_lut!($us, $us, one_right_angle_acw, $self_to_self);
            with_lut!(angles; $us, n_right_angles_cw);
            with_lut!(angles; $us, n_right_angles_acw);
            with_lut!(muldiv; $us, $us, ProperRotation, div_prop, id; const .0);
            with_lut!(muldiv; $us, $others, ImproperRotation, div_improp, id; const .0);
            with_lut!(muldiv; $us, ProperRotation, $us, mul_prop, $self_to_self; const);
            with_lut!(muldiv; $us, ImproperRotation, $others, mul_improp, $other_to_other; const);
        };

        (impls; $us: ty, $others: ty) => {

            /// Rotates a copy of `self` according to a `ProperRotation`.
            /// Maintains Geometric Group.
            impl Mul<ProperRotation> for $us {
                /// Output belongs to the same Geometric Group.
                type Output = Self;
                /// The `ProperRotation` is not examined bit-by-bit. Instead, a
                /// look-up on a 2-D LUT produces the result directly.
                fn mul (self, x: ProperRotation) -> Self::Output {
                    self.mul_prop(x)
                }
            }

            /// Rotates `self` according to a `ProperRotation`. Maintains
            /// Geometric Group.
            impl MulAssign<ProperRotation> for $us {
                /// The data-type doesn't change, so the result can be directly
                /// assigned.
                ///
                /// The `ProperRotation` is not examined bit-by-bit. Instead, a
                /// look-up on a 2-D LUT produces the result directly.
                fn mul_assign(&mut self, x: ProperRotation) {
                    *self = *self * x;
                }
            }

            /// Rotates a copy of `self` according to an `ImproperRotation`.
            /// Switches Geometric Group.
            impl Mul<ImproperRotation> for $us {
                /// Output belongs to the other Geometric Group.
                type Output = $others;
                /// The `ImproperRotation` is not examined bit-by-bit. Instead,
                /// a look-up on a 2-D LUT produces the result directly.
                fn mul (self, x: ImproperRotation) -> Self::Output {
                    self.mul_improp(x)
                }
            }

            impl Mul<$us> for ProperRotation {
                /// Output belongs to the same Geometric Group.
                type Output = $us;
                /// The `ProperRotation` is not examined bit-by-bit. Instead,
                /// a look-up on a 2-D LUT produces the result directly.
                fn mul (self, x: $us) -> Self::Output {
                    x * self
                }
            }

            impl Mul<$us> for ImproperRotation {
                /// Output belongs to the other Geometric Group.
                type Output = $others;
                /// The `ImproperRotation` is not examined bit-by-bit. Instead,
                /// a look-up on a 2-D LUT produces the result directly.
                fn mul (self, x: $us) -> Self::Output {
                    x * self
                }
            }

            /// Extracts the proper rotation that must occur so that the
            /// divisor` point ends up coinciding with `self`, ie the dividend.
            impl Div for $us {
                /// A proper rotation is enough for this operation.
                type Output = ProperRotation;
                /// A look-up on a 2-D LUT produces the result directly.
                fn div (self, x: Self) -> Self::Output {
                    self.div_prop(x)
                }
            }

            /// Extracts the improper rotation that must occur so that the
            /// divisor` point ends up coinciding with `self`, ie the dividend.
            impl Div<$others> for $us {
                /// This operation needs an improper rotation.
                type Output = ImproperRotation;
                /// A look-up on a 2-D LUT produces the result directly.
                fn div (self, x: $others) -> Self::Output {
                    self.div_improp(x)
                }
            }
        };

        (impls; $us: ty, $others: ty, $check: ident, $self_to_self: ident, $other_to_other: ident) => {
            /// Discards any knowledge of Geometric Group, producing a general
            /// `crate::CubeSurfacePoint`.
            impl From<$us> for CratePt {
                fn from(x: $us) -> CratePt {
                    x.0.downcast()
                }
            }

            /// Please refer to [`crate::CubeSurfacePoint::determine_group`].
            impl TryFrom<CratePt> for $us {
                #[doc = concat!(
                "If a certain [`CubeSurfacePoint`] does not belong to the [`",
                stringify!($us),
                                "`]s, it must by necessity belong to the [`",
                                stringify!($others),
                                "`]s."
                )]
                type Error = $others;

                fn try_from(x: CratePt) -> Result<Self, Self::Error> {
                    match x.$check() {
                        Ok(point) => {
                            Ok($self_to_self(point))
                        },
                        Err(point) => {
                            Err($other_to_other(point))
                        },
                    }
                }
            }
        };

}

/// As per the basic
/// [`ReferenceGroupPoint`]. Each LUT used is up to 576 bytes in
/// length, and each operation is guaranteed to consist of a single look-up.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct ReferenceGroupPoint(pub crate::ReferenceGroupPoint);

/// As per the basic
/// [`OppositeGroupPoint`]. Each LUT used is up to 576 bytes in
/// length, and each operation is guaranteed to consist of a single look-up.
#[derive(Debug, Copy, Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct OppositeGroupPoint(pub crate::OppositeGroupPoint);

const fn rgp_to_rgp(x: crate::ReferenceGroupPoint) -> ReferenceGroupPoint {
    ReferenceGroupPoint(x)
}

const fn ogp_to_ogp(x: crate::OppositeGroupPoint) -> OppositeGroupPoint {
    OppositeGroupPoint(x)
}

impl From<ProperRotation> for ReferenceGroupPoint {
    fn from(x: ProperRotation) -> Self {
        Self(x.corresponding_point)
    }
}

impl From<ImproperRotation> for OppositeGroupPoint {
    fn from(x: ImproperRotation) -> Self {
        Self(x.corresponding_point)
    }
}

impl From<CubeSurfacePoint<true>> for Rotation {
    fn from(corresponding_point: CubeSurfacePoint<true>) -> Self {
        Self {
            corresponding_point: corresponding_point.0,
        }
    }
}

impl From<CubeSurfacePoint<false>> for Rotation {
    fn from(corresponding_point: CubeSurfacePoint<false>) -> Self {
        Self {
            corresponding_point: corresponding_point.0,
        }
    }
}

impl From<ReferenceGroupPoint> for ProperRotation {
    fn from(corresponding_point: ReferenceGroupPoint) -> Self {
        Self {
            corresponding_point: corresponding_point.0,
        }
    }
}

impl From<OppositeGroupPoint> for ImproperRotation {
    fn from(corresponding_point: OppositeGroupPoint) -> Self {
        Self {
            corresponding_point: corresponding_point.0,
        }
    }
}

impl ProperRotation {
    const fn index_to_self(x: usize) -> Self {
        Self {
            corresponding_point: ReferenceGroupPoint::index_to_self(x).0,
        }
    }

    const fn self_to_index(self) -> usize {
        self.corresponding_point as usize
    }
}

impl ImproperRotation {
    const fn index_to_self(x: usize) -> Self {
        Self {
            corresponding_point: OppositeGroupPoint::index_to_self(x).0,
        }
    }

    const fn self_to_index(self) -> usize {
        self.corresponding_point as usize
    }
}

impl ReferenceGroupPoint {
    with_lut!(
        everything;
        ReferenceGroupPoint,
        OppositeGroupPoint,
        determine_group,
        rgp_to_rgp,
        ogp_to_ogp,
    );

    /// An alternative implementation of division, that uses the same big
    /// LUT as multiplication does, but performs two look-ups instead of
    /// one.
    pub fn div_alt(self, x: Self) -> ProperRotation {
        const LUT: [ReferenceGroupPoint; 24] = {
            let mut result = [ReferenceGroupPoint::index_to_self(0); 24];
            let mut i = 24;
            while i > 0 {
                i -= 1;
                result[i] = ReferenceGroupPoint(
                    ReferenceGroupPoint::index_to_self(0)
                        .div_prop(ReferenceGroupPoint::index_to_self(i))
                        .corresponding_point,
                );
            }
            result
        };

        let corresponding_point = ProperRotation {
            corresponding_point: self.0,
        } * LUT[x.0 as usize >> 1];
        ProperRotation {
            corresponding_point: corresponding_point.0,
        }
    }
}

impl OppositeGroupPoint {
    with_lut!(
        everything;
        OppositeGroupPoint,
        ReferenceGroupPoint,
        determine_antigroup,
        ogp_to_ogp,
        rgp_to_rgp,
    );

    /// An alternative implementation of division, that uses the same big
    /// LUT as multiplication does, but performs two look-ups instead of
    /// one.
    #[deprecated(note = "It is not yet clear whether this function should \
also exist for `OppositeGroupPoint`s.")]
    pub fn div_alt(self, x: Self) -> ProperRotation {
        const LUT: [OppositeGroupPoint; 24] = {
            let mut result = [OppositeGroupPoint::index_to_self(0); 24];
            let mut i = 24;
            while i > 0 {
                i -= 1;
                result[i] = OppositeGroupPoint(
                    ReferenceGroupPoint::index_to_self(0)
                        .div_improp(OppositeGroupPoint::index_to_self(i))
                        .corresponding_point,
                );
            }
            result
        };

        let corresponding_point = ImproperRotation {
            corresponding_point: self.0,
        } * LUT[x.0 as usize >> 1];
        ProperRotation {
            corresponding_point: corresponding_point.0,
        }
    }
}

with_lut!(impls; ReferenceGroupPoint, OppositeGroupPoint);
with_lut!(impls; OppositeGroupPoint, ReferenceGroupPoint);
with_lut!(impls;
ReferenceGroupPoint,
OppositeGroupPoint,
determine_group,
rgp_to_rgp,
ogp_to_ogp
);
with_lut!(impls;
OppositeGroupPoint,
ReferenceGroupPoint,
determine_antigroup,
ogp_to_ogp,
rgp_to_rgp
);
