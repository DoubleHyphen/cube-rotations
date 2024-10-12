# Cube Rotations

## Introduction
The `cube-rotations` crate offers the functionality needed to model the 24 different rotations that can be performed on a cube while keeping its faces parallel to the same set of planes. It's been built to help program smart building blocks, and is meant to run on the cheapest microcontrollers currently available on the market.

## Including the project
Add the following line under the `[dependencies]` section of your `Cargo.toml` file:

```toml
cube-rotations = "1.0"
```

## Usage

Say you have three points, `x`, `y`, and `z`. Each of them has a default orientation:
```rust
let x = NegThreePosOnePosTwo;
let y = PosTwoPosThreeNegOne;
let z = PosThreePosOneNegTwo;
```

But now you realise that `x`'s actual orientation is different:
```rust
let x_actual = PosThreePosTwoPosOne;
```
How do we find which rotation needs to be performed, such that `x` ends up 
coïnciding with `x_actual`?

All it takes is one division:
```rust
let rotation = x_actual / x;
```
And then, the rotation can simply be applied to each point:
```rust
let x_actual = x * rotation;
let y_actual = y * rotation;
let z_actual = z * rotation;
```
Please note that `(x_actual / x) * x` yields just `x_actual`, exactly as one would suppose from the notation.

## Look-up Tables
There's also a `luts` module that performs all operations using Look-up Tables:

```rust
let x = luts::CubeSurfacePoint(NegThreePosOnePosTwo);
let y = luts::CubeSurfacePoint(PosTwoPosThreeNegOne);
let z = luts::CubeSurfacePoint(PosThreePosOneNegTwo);
let x_actual = luts::CubeSurfacePoint(PosThreePosTwoPosOne);
let rotation = x_actual / x;
let x_actual = x * rotation;
let y_actual = y * rotation;
let z_actual = z * rotation;
```
There are more things to mention, but this is enough for a first glance. 
