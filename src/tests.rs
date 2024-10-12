use crate::*;

type _Coordinates = [i8; 3];

const REF_PT: CubeSurfacePoint = CubeSurfacePoint::REFERENCE_POINT;

impl CubeSurfacePoint {
    fn one_by_one() -> impl Iterator<Item = Self> + Clone {
        (0u8..48).flat_map(CubeSurfacePoint::try_from_u8)
    }

    fn opps() -> impl Iterator<Item = Self> + Clone {
        Self::one_by_one().filter(|x| x.odd_ones())
    }

    fn refs() -> impl Iterator<Item = Self> + Clone {
        Self::one_by_one().filter(|x| x.even_ones())
    }

    #[rustfmt::skip]
    const fn coordinates(self) -> _Coordinates {
        [
            [ 1,  2,  3], [ 1,  2, -3], [ 1, -2,  3], [ 1, -2, -3],
            [-1,  2,  3], [-1,  2, -3], [-1, -2,  3], [-1, -2, -3],

            [ 3,  2,  1], [ 3,  2, -1], [ 3, -2,  1], [ 3, -2, -1],
            [-3,  2,  1], [-3,  2, -1], [-3, -2,  1], [-3, -2, -1],

            [ 1,  3,  2], [ 1,  3, -2], [ 1, -3,  2], [ 1, -3, -2],
            [-1,  3,  2], [-1,  3, -2], [-1, -3,  2], [-1, -3, -2],

            [ 3,  1,  2], [ 3,  1, -2], [ 3, -1,  2], [ 3, -1, -2],
            [-3,  1,  2], [-3,  1, -2], [-3, -1,  2], [-3, -1, -2],

            [ 2,  1,  3], [ 2,  1, -3], [ 2, -1,  3], [ 2, -1, -3],
            [-2,  1,  3], [-2,  1, -3], [-2, -1,  3], [-2, -1, -3],

            [ 2,  3,  1], [ 2,  3, -1], [ 2, -3,  1], [ 2, -3, -1],
            [-2,  3,  1], [-2,  3, -1], [-2, -3,  1], [-2, -3, -1],
        ][self as usize]
    }

    fn try_from_coordinates(point: _Coordinates) -> Option<Self> {
        let abs_coords = [point[0].abs(), point[1].abs(), point[2].abs()];
        let orientation = match abs_coords {
            [1, 2, 3] => Some(0b000),
            [3, 2, 1] => Some(0b001),
            [1, 3, 2] => Some(0b010),
            [3, 1, 2] => Some(0b011),
            [2, 1, 3] => Some(0b100),
            [2, 3, 1] => Some(0b101),
            _ => None,
        }?;
        let result: u8 = orientation << 3
            | ((point[0] < 0) as u8) << 2
            | ((point[1] < 0) as u8) << 1
            | (point[2] < 0) as u8;

        Self::try_from_u8(result)
    }
}

type Tee = i8;
type Vector = nalgebra::Vector3<Tee>;

fn get_all_rotations() -> [nalgebra::Rotation3<f32>; 24] {
    type Tee = f32;
    use std::f32::consts::TAU;
    type Unit = nalgebra::Unit<nalgebra::Vector3<Tee>>;
    type Vector = nalgebra::Vector3<Tee>;
    type Rotation = nalgebra::Rotation3<Tee>;
    fn cleave(n: u8) -> Tee {
        TAU / (n as Tee)
    }

    fn add2(a: Unit, b: Unit) -> Unit {
        Unit::new_normalize(a.into_inner() + b.into_inner())
    }

    fn add3(a: Unit, b: Unit, c: Unit) -> Unit {
        Unit::new_normalize(a.into_inner() + b.into_inner() + c.into_inner())
    }
    let (x, y, z) = (Vector::x_axis(), Vector::y_axis(), Vector::z_axis());
    let axes_and_rotations = [
        (x, 4),
        (y, 4),
        (z, 4),
        (add2(x, y), 2),
        (add2(y, z), 2),
        (add2(z, x), 2),
        (add2(x, -y), 2),
        (add2(y, -z), 2),
        (add2(z, -x), 2),
        (add3(x, y, z), 3),
        (add3(x, y, -z), 3),
        (add3(x, -y, z), 3),
        (add3(x, -y, -z), 3),
    ];
    let mut all_rotations = [Rotation::identity(); 24];

    for (axis, amount) in axes_and_rotations {
        let angle = cleave(amount);
        for n in 1..amount {
            let angle = angle * (n as Tee);
            let rotation = Rotation::from_axis_angle(&axis, angle);
            let corresponding_point = rotation * Vector::new(1.0, 2.0, 3.0);
            let a = corresponding_point.map(|x| x.round() as i8);
            let to_csp = |x| CubeSurfacePoint::try_from_coordinates(x);
            let index_two = to_csp([a[0], a[1], a[2]]).unwrap() as usize;
            all_rotations[index_two / 2] = rotation;
        }
    }
    all_rotations
}

#[test]
#[cfg_attr(miri, ignore)]
fn check_all_rotations() {
    for (&xs, &ys) in get_all_rotations().iter().zip(PROPER_ROTATIONS.iter()) {
        for (x, &y) in xs.into_inner().into_iter().zip(ys.into_iter()) {
            assert_eq!(x.round() as Tee, y);
        }
    }

    for x in CubeSurfacePoint::refs() {
        let a = Vector::from_row_slice(&x.coordinates());
        let b = PROPER_ROTATIONS[x as usize >> 1] * Vector::new(1, 2, 3);

        assert_eq!(a, b);
    }

    for x in CubeSurfacePoint::opps() {
        let a = Vector::from_row_slice(&x.coordinates());
        let b = IMPROPER_ROTATIONS[x as usize >> 1] * Vector::new(1, 2, 3);

        assert_eq!(a, b);
    }
}

type Matrix = nalgebra::Matrix3<Tee>;

const ROTS: ([Matrix; 24], [Matrix; 24], [Matrix; 48]) = {
    #[rustfmt::skip]
    const ALL_ARRAYS: [[Tee; 9]; 24] = [
        [ 1,  0,  0,  0,  1,  0,  0,  0,  1],
        [ 1,  0,  0,  0, -1,  0,  0,  0, -1],
        [-1,  0,  0,  0,  1,  0,  0,  0, -1],
        [-1,  0,  0,  0, -1,  0,  0,  0,  1],
        [ 0,  0,  1,  0,  1,  0, -1,  0,  0],
        [ 0,  0,  1,  0, -1,  0,  1,  0,  0],
        [ 0,  0, -1,  0,  1,  0,  1,  0,  0],
        [ 0,  0, -1,  0, -1,  0, -1,  0,  0],
        [ 1,  0,  0,  0,  0,  1,  0, -1,  0],
        [ 1,  0,  0,  0,  0, -1,  0,  1,  0],
        [-1,  0,  0,  0,  0,  1,  0,  1,  0],
        [-1,  0,  0,  0,  0, -1,  0, -1,  0],
        [ 0,  0,  1,  1,  0,  0,  0,  1,  0],
        [ 0,  0,  1, -1,  0,  0,  0, -1,  0],
        [ 0,  0, -1,  1,  0,  0,  0, -1,  0],
        [ 0,  0, -1, -1,  0,  0,  0,  1,  0],
        [ 0,  1,  0,  1,  0,  0,  0,  0, -1],
        [ 0,  1,  0, -1,  0,  0,  0,  0,  1],
        [ 0, -1,  0,  1,  0,  0,  0,  0,  1],
        [ 0, -1,  0, -1,  0,  0,  0,  0, -1],
        [ 0,  1,  0,  0,  0,  1,  1,  0,  0],
        [ 0,  1,  0,  0,  0, -1, -1,  0,  0],
        [ 0, -1,  0,  0,  0,  1, -1,  0,  0],
        [ 0, -1,  0,  0,  0, -1,  1,  0,  0],
    ];

    const fn arr_to_rot(x: [Tee; 9]) -> Matrix {
        Matrix::new(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8])
    }

    const fn arr_to_irot(x: [Tee; 9]) -> Matrix {
        Matrix::new(x[0], x[1], x[2], x[3], x[4], x[5], -x[6], -x[7], -x[8])
    }

    let mut rotations = [arr_to_rot(ALL_ARRAYS[0]); 24];
    let mut matrices = [arr_to_irot(ALL_ARRAYS[0]); 24];
    let mut all = [arr_to_irot(ALL_ARRAYS[0]); 48];
    let mut i: usize = 24;

    while i > 0 {
        i -= 1;
        rotations[i] = arr_to_rot(ALL_ARRAYS[i]);
        matrices[i] = arr_to_irot(ALL_ARRAYS[i]);
        if i.count_ones() & 1 == 0 {
            all[2 * i] = rotations[i];
            all[2 * i + 1] = matrices[i];
        } else {
            all[2 * i] = matrices[i];
            all[2 * i + 1] = rotations[i];
        }
    }
    (rotations, matrices, all)
};

const PROPER_ROTATIONS: [Matrix; 24] = ROTS.0;
const IMPROPER_ROTATIONS: [Matrix; 24] = ROTS.1;
const ALL_MATRICES: [Matrix; 48] = ROTS.2;

impl From<CubeSurfacePoint> for Vector {
    fn from(point: CubeSurfacePoint) -> Self {
        Vector::from_row_slice(&point.coordinates())
    }
}

#[test]
#[cfg_attr(miri, ignore)]
fn exhaustive_search() {
    search(CubeSurfacePoint::refs());
    search(CubeSurfacePoint::opps());

    fn search(iterator: impl Iterator<Item = CubeSurfacePoint> + Clone) {
        assert_eq!(iterator.clone().count(), 24);

        for original_point in iterator.clone() {
            let original_vector: Vector = original_point.into();

            for target_point in iterator.clone() {
                let rotation = target_point / original_point;

                let final_vector: Vector = target_point.into();
                let rot = PROPER_ROTATIONS.get(rotation.corresponding_point as usize >> 1);

                if rot.is_none() {
                    dbg!((original_point, target_point, original_vector, final_vector));
                }

                let rot = rot.expect("No rotation found.");

                for random_point in iterator.clone() {
                    let rotated_point: Vector = (rotation * random_point).into();

                    let random_vector: Vector = random_point.into();
                    let rotated_vector = rot * random_vector;

                    let success = rotated_vector == rotated_point;

                    if !success {
                        dbg!((
                            original_point,
                            original_vector,
                            target_point,
                            final_vector,
                            rotation,
                            rot,
                            random_point,
                            random_vector,
                            rotated_point,
                            rotated_vector,
                            "\n\n\n\n\n",
                        ));
                        panic!("Rotations not identical.");
                    }
                }
            }
        }
    }
}

#[test]
fn multiplications() {
    for point in CubeSurfacePoint::one_by_one() {
        for rot in CubeSurfacePoint::refs() {
            let rot = rot / REF_PT;
            let rot_2 = PROPER_ROTATIONS[rot.corresponding_point as usize >> 1];
            assert_eq!(rot_2 * Vector::from(point), Vector::from(rot * point));
        }
    }
}

#[test]
fn divisions() {
    inner_function(CubeSurfacePoint::refs());
    inner_function(CubeSurfacePoint::opps());

    fn inner_function(iter: impl Iterator<Item = CubeSurfacePoint> + Clone) {
        for point_1 in iter.clone() {
            for point_2 in iter.clone() {
                let rotation = point_1 / point_2;
                let rotation_2 = PROPER_ROTATIONS[rotation.corresponding_point as usize >> 1];
                assert_eq!(rotation_2 * Vector::from(point_2), Vector::from(point_1));
            }
        }
    }
}

#[test]
fn improper_multiplications() {
    for point in CubeSurfacePoint::one_by_one() {
        for impr_rot in CubeSurfacePoint::opps() {
            let impr_rot = impr_rot / REF_PT;
            let impr_rot_2 = IMPROPER_ROTATIONS[impr_rot.corresponding_point as usize >> 1];
            assert_eq!(
                impr_rot_2 * Vector::from(point),
                Vector::from(impr_rot * point)
            );
        }
    }
}

#[test]
fn improper_divisions() {
    for point_1 in CubeSurfacePoint::refs() {
        for point_2 in CubeSurfacePoint::opps() {
            let impr_rot = point_1 / point_2;
            let impr_rot_2 = IMPROPER_ROTATIONS[impr_rot.corresponding_point as usize >> 1];
            assert_eq!(impr_rot_2 * Vector::from(point_2), Vector::from(point_1));

            let impr_rot = point_2 / point_1;
            let impr_rot_2 = IMPROPER_ROTATIONS[impr_rot.corresponding_point as usize >> 1];
            assert_eq!(impr_rot_2 * Vector::from(point_1), Vector::from(point_2));
        }
    }
}

#[test]
fn reciprocability() {
    for x in CubeSurfacePoint::one_by_one() {
        let recip_x = x.reciprocal();
        let rot = Rotation {
            corresponding_point: x,
        };
        let recip_rot = Rotation {
            corresponding_point: recip_x,
        };
        assert_eq!(rot * recip_x, REF_PT);
        assert_eq!(recip_rot * x, REF_PT);
    }
}

#[test]
fn opposite() {
    for x in CubeSurfacePoint::one_by_one() {
        let opposite = x.opposite();
        let [x, y, z] = x.coordinates();
        let [a, b, c] = opposite.coordinates();
        let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
        sums.sort();
        assert_eq!(sums, [0, 2, 4]);
    }
}

#[test]
fn beside() {
    for x in CubeSurfacePoint::one_by_one() {
        let beside = x.beside();
        let [x, y, z] = x.coordinates();
        let [a, b, c] = beside.coordinates();
        let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
        sums.sort();
        assert_eq!(sums, [0, 4, 6]);
    }
}

#[test]
fn rights() {
    use crate::CubeSurfacePoint::*;

    let at_least_one_per_axis_is_cw = PosOnePosTwoPosThree.one_right_angle_cw()
        == PosTwoNegOnePosThree
        && PosThreePosTwoPosOne.one_right_angle_cw() == PosThreePosOneNegTwo
        && PosOnePosThreePosTwo.one_right_angle_cw() == NegTwoPosThreePosOne;

    for x in CubeSurfacePoint::one_by_one() {
        let coord = x.coordinates();
        let coord2 = x.one_right_angle_cw().coordinates();
        let (face, that, this) = if coord[0].abs() == 3 {
            (0, 1, 2)
        } else if coord[1].abs() == 3 {
            (1, 2, 0)
        } else if coord[2].abs() == 3 {
            (2, 0, 1)
        } else {
            panic!("Not on surface of cube!")
        };

        let plane_remains_unchanged = coord2[face] == coord[face];

        let the_absolute_values_of_one_and_two_swapped =
            coord[that].abs() == coord2[this].abs() && coord2[that].abs() == coord[this].abs();

        let sign_sum = coord[that].is_positive() as u8
            + coord2[that].is_positive() as u8
            + coord[this].is_positive() as u8
            + coord2[this].is_positive() as u8;

        let exactly_one_coordinate_changed_sign = sign_sum & 1 == 1;

        let turning_four_times_changes_nothing = {
            x == x
                .one_right_angle_cw()
                .one_right_angle_cw()
                .one_right_angle_cw()
                .one_right_angle_cw()
        };

        let opposite_points_turn_opposite = {
            let opposite = x.opposite();
            let counter_rotation = opposite.one_right_angle_cw() / opposite;
            x == x.one_right_angle_cw() * counter_rotation
        };

        assert_ne!(x, x.one_right_angle_cw());
        assert_ne!(x, x.one_right_angle_cw().one_right_angle_cw());
        assert_ne!(
            x,
            x.one_right_angle_cw()
                .one_right_angle_cw()
                .one_right_angle_cw()
        );
        assert!(at_least_one_per_axis_is_cw);
        assert!(plane_remains_unchanged);
        assert!(the_absolute_values_of_one_and_two_swapped);
        assert!(exactly_one_coordinate_changed_sign);
        assert!(turning_four_times_changes_nothing);
        assert!(opposite_points_turn_opposite);
    }
}

#[test]
fn antirights() {
    for x in CubeSurfacePoint::one_by_one() {
        assert_eq!(x, x.one_right_angle_cw().one_right_angle_acw());
    }
}

#[test]
fn n_rights() {
    for x in CubeSurfacePoint::one_by_one() {
        for angle in 0..16 {
            let mut y = x;
            let mut z = x;
            for _ in 0..angle {
                y = y.one_right_angle_cw();
                z = z.one_right_angle_acw();
            }
            assert_eq!(y, x.n_right_angles_cw(angle));
            assert_eq!(z, x.n_right_angles_acw(angle));
        }
    }
}

#[test]
fn direction() {
    for x in CubeSurfacePoint::one_by_one() {
        use Direction::*;
        let coord = x.coordinates();
        let result = x.direction();
        let x = match coord {
            [_, _, -3] => Down,
            [_, _, 3] => Up,
            [-3, _, _] => Left,
            [3, _, _] => Right,
            [_, -3, _] => Back,
            [_, 3, _] => Front,
            _ => panic!(),
        };
        assert_eq!(x, result);
    }
}

#[test]
fn swap_xy() {
    for point in CubeSurfacePoint::one_by_one() {
        let try_1 = point.swap_x_y();
        let try_2 = {
            let mut coord = point.coordinates();
            coord.swap(0, 1);
            CubeSurfacePoint::try_from_coordinates(coord).unwrap()
        };
        assert_eq!(try_1, try_2);
    }
}

#[test]
fn swap_xz() {
    for point in CubeSurfacePoint::one_by_one() {
        let try_1 = point.swap_z_x();
        let try_2 = {
            let mut coord = point.coordinates();
            coord.swap(0, 2);
            CubeSurfacePoint::try_from_coordinates(coord).unwrap()
        };
        assert_eq!(try_1, try_2);
    }
}

#[test]
fn swap_zy() {
    for point in CubeSurfacePoint::one_by_one() {
        let try_1 = point.swap_y_z();
        let try_2 = {
            let mut coord = point.coordinates();
            coord.swap(2, 1);
            CubeSurfacePoint::try_from_coordinates(coord).unwrap()
        };
        assert_eq!(try_1, try_2);
    }
}

#[test]
#[cfg(all(miri, not(debug_assertions)))]
const fn test_transmute() {
    let mut x = 48;
    while x > 0 {
        x -= 1;
        let _ = CubeSurfacePoint::_try_from_u8_readable(x);
    }
}

#[test]
fn test_from_u8() {
    use crate::CubeSurfacePoint as CSP;
    let mut x = 255;

    while x >= 48 {
        let rslt: Result<CSP, _> = x.try_into();
        assert!(rslt.is_err());
        let rslt: Option<CSP> = CSP::try_from_u8(x);
        assert!(rslt.is_none());
        x -= 1;
    }

    while x < 48 {
        let _rslt: CSP = x.try_into().unwrap();
        let _rslt: CSP = CSP::try_from_u8(x).unwrap();
        x = x.wrapping_sub(1);
    }
}

#[should_panic(expected = "Dead code was called!")]
#[test]
#[cfg(debug_assertions)]
const fn unreachable() {
    unreachable_semichecked::<CubeSurfacePoint>();
}

#[should_panic(expected = "This value does not correspond to any cube surface point!")]
#[test]
#[cfg(debug_assertions)]
const fn illegal_unwrap() {
    CubeSurfacePoint::const_unwrap_semichecked(None);
}

#[should_panic(expected = "This value does not correspond to any cube surface point!")]
#[test]
#[cfg(debug_assertions)]
fn illegal_transmute() {
    let integer = loop {
        let integer = rand::random::<u8>();
        if integer >= 48 {
            break integer;
        }
    };
    let _ = CubeSurfacePoint::probs_from_u8(integer);
}

#[test]
fn luts() {
    for point in CubeSurfacePoint::one_by_one() {
        let small_lut_pt = luts::CubeSurfacePoint::<false>::from(point);
        let big_lut_pt = luts::CubeSurfacePoint::<true>::from(point);

        macro_rules! assert_three {
            ($fn: ident) => {{
                let a = point.$fn();
                let b = small_lut_pt.$fn().into();
                let c = big_lut_pt.$fn().into();
                assert_eq!(a, b);
                assert_eq!(c, b);
                assert_eq!(a, c);
            }};

            ($fn: ident, $($fns:ident),+) => {
                assert_three!($fn);
                assert_three!($($fns),+);
            };
        }

        dbg!(point);
        assert_three!(
            beside,
            opposite,
            direction,
            one_right_angle_cw,
            one_right_angle_acw
        );

        for angle in 0..16 {
            let a = point.n_right_angles_cw(angle);
            let b = small_lut_pt.n_right_angles_cw(angle).into();
            let c = big_lut_pt.n_right_angles_cw(angle).into();
            assert_eq!(a, b);
            assert_eq!(c, b);
            assert_eq!(a, c);

            let a = point.n_right_angles_acw(angle);
            let b = small_lut_pt.n_right_angles_acw(angle).into();
            let c = big_lut_pt.n_right_angles_acw(angle).into();
            assert_eq!(a, b);
            assert_eq!(c, b);
            assert_eq!(a, c);
        }

        for point_2 in CubeSurfacePoint::one_by_one() {
            let small_2 = luts::CubeSurfacePoint::<false>::from(point_2);
            let big_2 = luts::CubeSurfacePoint::<true>::from(point_2);
            let ratio_1: Rotation = point / point_2;
            let ratio_2: Rotation = small_lut_pt / small_2;
            let ratio_3: Rotation = big_lut_pt / big_2;
            assert_eq!(ratio_1, ratio_2);
            assert_eq!(ratio_3, ratio_2);
            assert_eq!(ratio_3, ratio_1);
            assert_eq!(ratio_3, big_lut_pt.div_alt(big_2));

            let rot = Rotation {
                corresponding_point: point_2,
            };
            let prod_1 = rot * point;
            let prod_2 = (rot * small_lut_pt).into();
            let prod_3 = (rot * big_lut_pt).into();
            assert_eq!(prod_1, prod_2);
            assert_eq!(prod_2, prod_3);
            assert_eq!(prod_3, prod_1);
        }
    }
}

#[test]
fn geom_group() {
    macro_rules! check {
        ($q: ident, $($a: ident, )*) => {
            $(assert_eq!($q.downcast().$a(), $q.$a().downcast());)*
        };
        ($q: ident) => {
            check!(
                $q,
                beside,
                opposite,
                one_right_angle_cw,
                one_right_angle_acw,
            );

            for n in 16..32 {
                assert_eq!($q.downcast().n_right_angles_cw(n),
                            $q.n_right_angles_cw(n).downcast());
                assert_eq!($q.downcast().n_right_angles_acw(n),
                            $q.n_right_angles_acw(n).downcast());
            }
            for point in CubeSurfacePoint::one_by_one() {
                let aa = point.determine_group();
                match aa {
                    Ok(qq) => {
                        let rot: Rotation = ($q / qq).into();
                        assert_eq!(rot, $q.downcast() / qq.downcast());
                    },
                    Err(qq) => {
                        let rot: Rotation = ($q / qq).into();
                        assert_eq!(rot, $q.downcast() / qq.downcast());

                    }
                }
            }
        };
    }

    for point in CubeSurfacePoint::one_by_one() {
        let a = point.determine_group();
        let b = point.determine_antigroup();
        match (a, b) {
            (Ok(q), Err(w)) => {
                assert_eq!(q, w);
                check!(q);
                let a = luts::ReferenceGroupPoint(q);
                assert_eq!(a.beside().0, q.beside());
                assert_eq!(a.opposite().0, q.opposite());
                assert_eq!(a.one_right_angle_cw().0, q.one_right_angle_cw());
                assert_eq!(a.one_right_angle_acw().0, q.one_right_angle_acw());
                for n in 16..32 {
                    assert_eq!(a.n_right_angles_acw(n).0, q.n_right_angles_acw(n));
                    assert_eq!(a.n_right_angles_cw(n).0, q.n_right_angles_cw(n));
                }
            }
            (Err(q), Ok(w)) => {
                assert_eq!(q, w);
                check!(q);
                let a = luts::OppositeGroupPoint(q);
                assert_eq!(a.beside().0, q.beside());
                assert_eq!(a.opposite().0, q.opposite());
                assert_eq!(a.one_right_angle_cw().0, q.one_right_angle_cw());
                assert_eq!(a.one_right_angle_acw().0, q.one_right_angle_acw());
                for n in 16..32 {
                    assert_eq!(a.n_right_angles_acw(n).0, q.n_right_angles_acw(n));
                    assert_eq!(a.n_right_angles_cw(n).0, q.n_right_angles_cw(n));
                }
            }
            _ => panic!(),
        }
    }
}

#[test]
fn lut_geom_group() {
    for point in CubeSurfacePoint::one_by_one() {
        for point2 in CubeSurfacePoint::one_by_one() {
            let a = point.determine_group();
            let b = point2.determine_group();

            match (a, b) {
                (Ok(q), Ok(w)) => {
                    let q = luts::ReferenceGroupPoint(q);
                    let w = luts::ReferenceGroupPoint(w);
                    assert_eq!(q / w, q.0 / w.0);
                    let q = ProperRotation {
                        corresponding_point: q.0,
                    };
                    assert_eq!((q * w).0, q * w.0);
                }
                (Ok(q), Err(w)) => {
                    let q = luts::ReferenceGroupPoint(q);
                    let w = luts::OppositeGroupPoint(w);
                    assert_eq!(q / w, q.0 / w.0);
                    let q = ProperRotation {
                        corresponding_point: q.0,
                    };
                    assert_eq!((q * w).0, q * w.0);
                }
                (Err(q), Ok(w)) => {
                    let q = luts::OppositeGroupPoint(q);
                    let w = luts::ReferenceGroupPoint(w);
                    assert_eq!(q / w, q.0 / w.0);
                    let q = ImproperRotation {
                        corresponding_point: q.0,
                    };
                    assert_eq!((q * w).0, q * w.0);
                }
                (Err(q), Err(w)) => {
                    let q = luts::OppositeGroupPoint(q);
                    let w = luts::OppositeGroupPoint(w);
                    assert_eq!(q / w, q.0 / w.0);
                    let q = ImproperRotation {
                        corresponding_point: q.0,
                    };
                    assert_eq!((q * w).0, q * w.0);
                }
            }
        }
    }
}

#[test]
fn test_rotation_multiplication_and_division() {
    let to_matrix = |x: Rotation| ALL_MATRICES[x.corresponding_point as usize];

    for rot_a in CubeSurfacePoint::one_by_one().map(Rotation::from) {
        for rot_b in CubeSurfacePoint::one_by_one().map(Rotation::from) {
            assert_eq!(
                to_matrix(rot_a) * to_matrix(rot_b),
                to_matrix(rot_a * rot_b)
            );

            assert_eq!((rot_a * rot_b) / rot_b, rot_a);

            assert_eq!(
                rot_a * rot_b,
                luts::multiply_rotations_luts::<true>(rot_a, rot_b)
            );
            assert_eq!(
                rot_a * rot_b,
                luts::multiply_rotations_luts::<false>(rot_a, rot_b)
            );
            assert_eq!(
                rot_a / rot_b,
                luts::divide_rotations_luts::<true>(rot_a, rot_b)
            );
            assert_eq!(
                rot_a / rot_b,
                luts::divide_rotations_luts::<false>(rot_a, rot_b)
            );
        }
    }
}

#[test]
fn test_opposite_then_beside() {
    macro_rules! check {
        ($x: expr) => {
            let a = $x.opposite_then_beside();
            let b = $x.beside_then_opposite();
            let c = $x.opposite().beside();
            let d = $x.beside().opposite();
            assert!(a == b && b == c && c == d);
        };
    }

    for x in CubeSurfacePoint::one_by_one() {
        check!(x);
        check!(luts::CubeSurfacePoint::<true>(x));
        check!(luts::CubeSurfacePoint::<false>(x));

        match x.determine_group() {
            Ok(x) => {
                check!(x);
                check!(luts::ReferenceGroupPoint(x));
            }
            Err(x) => {
                check!(x);
                check!(luts::OppositeGroupPoint(x));
            }
        }
    }
}

#[test]
fn test_flips() {
    fn coordinates(x: impl OctahedrallySymmetricPoint) -> [i8; 3] {
        let x: CubeSurfacePoint = x.into();
        x.coordinates()
    }

    for x in CubeSurfacePoint::one_by_one() {
        macro_rules! check {
            ($point: expr) => {
                let [x, y, z] = coordinates($point);
                let [a, b, c] = coordinates($point.flip_sign_of_1());
                let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
                sums.sort();
                assert_eq!(sums, [0, 4, 6]);

                let [a, b, c] = coordinates($point.flip_sign_of_2());
                let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
                sums.sort();
                assert_eq!(sums, [0, 2, 6]);

                let [a, b, c] = coordinates($point.flip_sign_of_3());
                let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
                sums.sort();
                assert_eq!(sums, [0, 2, 4]);

                let [a, b, c] = coordinates($point.flip_1_and_2());
                let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
                sums.sort();
                assert_eq!(sums, [0, 0, 6]);

                let [a, b, c] = coordinates($point.flip_1_and_3());
                let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
                sums.sort();
                assert_eq!(sums, [0, 0, 4]);

                let [a, b, c] = coordinates($point.flip_2_and_3());
                let mut sums = [(x + a).abs(), (y + b).abs(), (z + c).abs()];
                sums.sort();
                assert_eq!(sums, [0, 0, 2]);
            };
        }
        check!(x);
        check!(luts::CubeSurfacePoint::<true>(x));
        check!(luts::CubeSurfacePoint::<false>(x));

        match x.determine_group() {
            Ok(x) => {
                check!(x);
                check!(luts::ReferenceGroupPoint(x));
            }
            Err(x) => {
                check!(x);
                check!(luts::OppositeGroupPoint(x));
            }
        }
    }
}
