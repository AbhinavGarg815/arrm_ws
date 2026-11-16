# Transformation Matrices - arm_final

Homogeneous transformation matrices between consecutive frames.
Convention: URDF RPY (XYZ extrinsic / ZYX intrinsic).

## Notation

### Frames

| Index | Link |
|-------|------|
| $L_{0}$ | base_link |
| $L_{1}$ | Link1_Shoulder |
| $L_{2}$ | Link2_UpperArm |
| $L_{3}$ | Link3_ForeArm |
| $L_{4}$ | Link4_Wrist1 |
| $L_{5}$ | Link5_Wrist2 |
| $L_{6}$ | Link6_Gripper |
| $L_{7}$ | Finger_Moving |

### Joint Variables

| Variable | Joint | Type | From | To |
|----------|-------|------|------|----|
| $q_{1}$ | joint1 | continuous (rad) | $L_{0}$ | $L_{1}$ |
| $q_{2}$ | joint2 | continuous (rad) | $L_{1}$ | $L_{2}$ |
| $q_{3}$ | joint3 | continuous (rad) | $L_{2}$ | $L_{3}$ |
| $q_{4}$ | joint4 | continuous (rad) | $L_{3}$ | $L_{4}$ |
| $q_{5}$ | joint5 | continuous (rad) | $L_{4}$ | $L_{5}$ |
| $q_{6}$ | joint6 | continuous (rad) | $L_{5}$ | $L_{6}$ |
| $q_{7}$ | Slider_39 | prismatic (m) | $L_{6}$ | $L_{7}$ |

Shorthand: $c_i = \cos(q_i)$, $s_i = \sin(q_i)$

### Kinematic Tree

```
L0: base_link
  +-- [continuous] joint1 (q1)
      L1: Link1_Shoulder
        +-- [continuous] joint2 (q2)
            L2: Link2_UpperArm
              +-- [continuous] joint3 (q3)
                  L3: Link3_ForeArm
                    +-- [continuous] joint4 (q4)
                        L4: Link4_Wrist1
                          +-- [continuous] joint5 (q5)
                              L5: Link5_Wrist2
                                +-- [continuous] joint6 (q6)
                                    L6: Link6_Gripper
                                      +-- [prismatic] Slider_39 (q7)
                                          L7: Finger_Moving
```

## Transforms

## joint1

$L_{0}$ **base_link** -> $L_{1}$ **Link1_Shoulder** (continuous)
  Variable: $q_{1}$

- **origin xyz**: (0.074556, 0.083272, 0.20227) m
- **origin rpy**: (-1.579827, 0.00016, 0.017765) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{0}_{1}(q_{1}) = T_{fixed} \cdot R_{axis}(q_{1})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.999842 & 0 & -0.017765 & 0.074556 \\
0.017765 & -0.009032 & 0.999801 & 0.083272 \\
-0.00016 & -0.999959 & -0.009031 & 0.20227 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{1}) = \begin{bmatrix}
c_{1} & -s_{1} & 0 & 0 \\
s_{1} & c_{1} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## joint2

$L_{1}$ **Link1_Shoulder** -> $L_{2}$ **Link2_UpperArm** (continuous)
  Variable: $q_{2}$

- **origin xyz**: (0.055397, -0.006109, 0.168494) m
- **origin rpy**: (1.570796, 0.231806, 1.495452) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{1}_{2}(q_{2}) = T_{fixed} \cdot R_{axis}(q_{2})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.073259 & 0.017293 & 0.997163 & 0.055397 \\
0.970492 & 0.229084 & -0.075273 & -0.006109 \\
-0.229736 & 0.973253 & 0 & 0.168494 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{2}) = \begin{bmatrix}
c_{2} & -s_{2} & 0 & 0 \\
s_{2} & c_{2} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## joint3

$L_{2}$ **Link2_UpperArm** -> $L_{3}$ **Link3_ForeArm** (continuous)
  Variable: $q_{3}$

- **origin xyz**: (0.132173, 0.260352, -0.089924) m
- **origin rpy**: (3.141593, 0, 0) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{2}_{3}(q_{3}) = T_{fixed} \cdot R_{axis}(q_{3})$ where:

$$
T_{fixed} = \begin{bmatrix}
1 & 0 & 0 & 0.132173 \\
0 & -1 & 0 & 0.260352 \\
0 & 0 & -1 & -0.089924 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{3}) = \begin{bmatrix}
c_{3} & -s_{3} & 0 & 0 \\
s_{3} & c_{3} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## joint4

$L_{3}$ **Link3_ForeArm** -> $L_{4}$ **Link4_Wrist1** (continuous)
  Variable: $q_{4}$

- **origin xyz**: (-0.075564, -0.027775, -0.036337) m
- **origin rpy**: (1.570796, 1.549855, -1.297363) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{3}_{4}(q_{4}) = T_{fixed} \cdot R_{axis}(q_{4})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.005655 & 0.26998 & -0.962849 & -0.075564 \\
-0.020162 & -0.962638 & -0.270039 & -0.027775 \\
-0.999781 & 0.02094 & 0 & -0.036337 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{4}) = \begin{bmatrix}
c_{4} & -s_{4} & 0 & 0 \\
s_{4} & c_{4} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## joint5

$L_{4}$ **Link4_Wrist1** -> $L_{5}$ **Link5_Wrist2** (continuous)
  Variable: $q_{5}$

- **origin xyz**: (0.016426, 0.001425, 0.082512) m
- **origin rpy**: (-1.567187, 0.478036, -1.418347) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{4}_{5}(q_{5}) = T_{fixed} \cdot R_{axis}(q_{5})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.134837 & -0.066293 & 0.988648 & 0.016426 \\
-0.877602 & 0.455245 & 0.150218 & 0.001425 \\
-0.460036 & -0.887895 & 0.003204 & 0.082512 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{5}) = \begin{bmatrix}
c_{5} & -s_{5} & 0 & 0 \\
s_{5} & c_{5} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## joint6

$L_{5}$ **Link5_Wrist2** -> $L_{6}$ **Link6_Gripper** (continuous)
  Variable: $q_{6}$

- **origin xyz**: (-0.054997, 0.021646, -0.028534) m
- **origin rpy**: (-1.678871, -1.540667, 1.502969) rad
- **axis**: (0, 0, 1)

### Local Transform

$T^{5}_{6}(q_{6}) = T_{fixed} \cdot R_{axis}(q_{6})$ where:

$$
T_{fixed} = \begin{bmatrix}
0.002042 & 0.174966 & -0.984572 & -0.054997 \\
0.030055 & 0.984119 & 0.174948 & 0.021646 \\
0.999546 & -0.029949 & -0.003249 & -0.028534 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

$$
R_{axis}(q_{6}) = \begin{bmatrix}
c_{6} & -s_{6} & 0 & 0 \\
s_{6} & c_{6} & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Slider_39

$L_{6}$ **Link6_Gripper** -> $L_{7}$ **Finger_Moving** (prismatic)
  Variable: $q_{7}$

- **origin xyz**: (0.002255, -0.068272, 0.025905) m
- **origin rpy**: (-1.570796, -1.294169, -3.092055) rad
- **axis**: (0, 0, 1)
- **limits**: [-0.065, -0.01] m

### Local Transform

$$
T^{6}_{7}(q_{7}) = \begin{bmatrix}
-0.272778 & -0.960802 & 0.049518 & 0.002255 \\
-0.013524 & -0.047635 & -0.998773 & -0.068272 \\
0.961982 & -0.273113 & 0 & 0.025905 + q_{7} \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

---

## Global Transform Chains

Transform from root $L_0$ to any link, as product of local transforms along the kinematic chain.

$$T^{0}_{2} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2})\quad (L_0 \to L_{2}: \text{Link2_UpperArm})$$

$$T^{0}_{3} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3})\quad (L_0 \to L_{3}: \text{Link3_ForeArm})$$

$$T^{0}_{4} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4})\quad (L_0 \to L_{4}: \text{Link4_Wrist1})$$

$$T^{0}_{5} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5})\quad (L_0 \to L_{5}: \text{Link5_Wrist2})$$

$$T^{0}_{6} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6})\quad (L_0 \to L_{6}: \text{Link6_Gripper})$$

$$T^{0}_{7} = T^{0}_{1}(q_{1}) \cdot T^{1}_{2}(q_{2}) \cdot T^{2}_{3}(q_{3}) \cdot T^{3}_{4}(q_{4}) \cdot T^{4}_{5}(q_{5}) \cdot T^{5}_{6}(q_{6}) \cdot T^{6}_{7}(q_{7})\quad (L_0 \to L_{7}: \text{Finger_Moving})$$

