# SOA_1: Manipulator Jacobian with Spatial Operator Algebra

Homework SOA_1 for **MKM504E Advanced Topics in Robotics** (Istanbul Technical University).

A general MATLAB script that builds the manipulator Jacobian of an n-DOF serial arm with revolute joints using spatial operator algebra:

```
J = Phi_t * Phi * H        (6 x n)
```

Rows 1-3 of `J` give the tip angular velocity, rows 4-6 the tip linear velocity, so `[w_t; v_t] = J * thetadot`.

## Files

| File | Description |
|------|-------------|
| `SOA_1.m` | MATLAB script: body-fixed frames, `Phi_t`, `H`, `Phi`, `J`, a check against the geometric Jacobian, and the sketch |
| `SOA_1_sketch.pdf` | Sketch of the example arm in its initial configuration, with the fixed reference frame |

## Usage

Run `SOA_1.m` in MATLAB (R2020a or later, because of `exportgraphics`). It prints `J`, prints `max |J - Jc|` (about 1e-16), and writes `SOA_1_sketch.pdf`.

For another manipulator only these inputs change:

- `n`: number of joints
- `Lb`: link vectors in their own body frames (3 x n, last column is the vector to the tip)
- `hb`: joint axes in their own body frames (3 x n, unit vectors)
- `dtheta`: joint rotations from the pre-initial to the initial configuration [rad]

## Method

1. All body frames start parallel to the fixed frame {0} (pre-initial configuration).
2. Frames i..n are rotated about joint axis `h_i` with Rodrigues' formula by `dtheta(i)`.
3. Link vectors and joint axes are expressed in {0} and used to fill:
   - `phit` (6 x 6n): `[0 ... 0 phi_tn]`
   - `H` (6n x n): block diagonal, `H_i = [h_i; 0]`
   - `phi` (6n x 6n): lower block-triangular, `I` on the diagonal and `phi_ij = [I 0; -l~_ji I]` below it
4. `J = phit * phi * H`, then compared with the geometric Jacobian `[h_i; h_i x (p_t - p_i)]`.

## Example arm

5-DOF arm: `l12 = 0.2 x1`, `l23 = 0.5 z2`, `l34 = 0.4 z3`, `l45 = 0.3 x4`, `l5t = 0.2 x5`;
`h1 = z1`, `h2 = y2`, `h3 = z3`, `h4 = y4`, `h5 = x5`;
initial configuration `dtheta = [0 45 0 -60 0]` degrees.
