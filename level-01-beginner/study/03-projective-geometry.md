# Module 3: Projective Geometry and Multi-View Geometry

> Covers all content of "Projective Geometry" from visual-slam-roadmap Level 1.
> This is the **geometric core** of visual SLAM — understand this, and half of SLAM makes sense.
> Each formula is accompanied by a **complete numerical computation example**.

---

## 3.1 Pinhole Camera Model

### 3.1.1 Complete Pipeline from 3D World to 2D Image

The first fundamental question of visual SLAM: **How does a 3D point in the world become a pixel on an image?**

This process involves three steps:

```text
World coordinates P_w ──[Extrinsic R,t]──→ Camera coordinates P_c ──[Intrinsic K]──→ Pixel coordinates (u,v)
```

**Step 1: World Coordinate System → Camera Coordinate System (Rigid Body Transformation)**

$$P_c = R P_w + t$$

where
$$R \in SO(3)$$
is the rotation matrix,
$$t \in \mathbb{R}^3$$
is the translation vector.

**Step 2: Camera Coordinate System → Normalized Image Plane (Perspective Projection)**

A point in camera coordinates
$$[X_c, Y_c, Z_c]^\top$$
projects to the normalized plane
$$Z=1$$

$$x_n = \begin{bmatrix} X_c/Z_c \\\\ Y_c/Z_c \\\\ 1 \end{bmatrix}$$

This step embodies the perspective effect: distant objects project closer to the image center (divided by a larger Z).

**Step 3: Normalized Plane → Pixel Coordinates (Intrinsic Matrix)**

$$\begin{bmatrix} u \\\\ v \\\\ 1 \end{bmatrix} = K \begin{bmatrix} X_c/Z_c \\\\ Y_c/Z_c \\\\ 1 \end{bmatrix} = \begin{bmatrix} f_x & 0 & c_x \\\\ 0 & f_y & c_y \\\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_c/Z_c \\\\ Y_c/Z_c \\\\ 1 \end{bmatrix}$$

where:
- **Focal length**:
  $$f_x$$, $$f_y$$
  (in pixels, determines the field of view)
- **Principal point**:
  $$c_x$$, $$c_y$$
  (the intersection of the optical axis with the image plane, usually at the image center)
- $$f_x$$
  and
  $$f_y$$
  are usually close (square pixels), but CCD manufacturing tolerances can cause slight differences

**Combined into a projection matrix P**:

$$P = K [R \mid t]_{3\times4}$$

$$x_{pixel} \sim P \cdot P_{world}$$

where
$$\sim$$
denotes equivalence up to a scale factor (homogeneous coordinates).

> **Example 1** — Complete Projection Calculation
>
> Camera parameters:
> $$f_x = 800$$,
> $$f_y = 800$$,
> $$c_x = 640$$,
> $$c_y = 480$$
> (1280×960 image)
>
> Camera pose: at origin, facing +Z (i.e.
> $$R=I$$,
> $$t=[0,0,0]^\top$$
> )
>
> World point
> $$P_w = [2, 1.5, 10]^\top$$
> (10m ahead, 2m to the right, 1.5m above)
>
> **Step 1**:
> $$P_c = I \cdot [2,1.5,10]^\top + [0,0,0]^\top = [2, 1.5, 10]^\top$$
>
> **Step 2**: normalized coordinates
> $$x_n = \begin{bmatrix} 2/10 \\\\ 1.5/10 \\\\ 1 \end{bmatrix} = \begin{bmatrix} 0.2 \\\\ 0.15 \\\\ 1 \end{bmatrix}$$
>
> **Step 3**: pixel coordinates
> $$\begin{bmatrix} u \\\\ v \end{bmatrix} = \begin{bmatrix} 800\times0.2 + 640 \\\\ 800\times0.15 + 480 \end{bmatrix} = \begin{bmatrix} 160 + 640 \\\\ 120 + 480 \end{bmatrix} = \begin{bmatrix} 800 \\\\ 600 \end{bmatrix}$$
>
> That is: an object 10 meters away appears at (800, 600) in the image.
>
> ---
>
> **Example 1b** — Same point, camera moves back 5 meters
>
> Camera moves back:
> $$t = [0, 0, -5]^\top$$
> (the world point is farther from the camera)
>
> $$P_c = [2, 1.5, 10]^\top - [0, 0, -5]^\top = [2, 1.5, 15]^\top$$
>
> Normalized:
> $$x_n = [2/15, 1.5/15, 1]^\top = [0.1333, 0.1, 1]^\top$$
>
> Pixels:
> $$u = 800\times0.1333 + 640 \approx 747$$,
> $$v = 800\times0.1 + 480 = 560$$
>
> After moving back, the pixel is closer to the principal point (640, 480) — perspective shrinking effect!

---

### 3.1.2 Practical Projection Example

Camera at origin, facing +Z direction, intrinsic
$$f_x = f_y = 500$$
, principal point
$$(320, 240)$$

World point
$$[1, 0.5, 5]^\top$$
(5m ahead, 1m to the right, 0.5m above):

```text
P_c = [1, 0.5, 5]ᵀ        (camera coordinates)
x_n = [0.2, 0.1, 1]ᵀ      (normalized plane)
u = 500 * 0.2 + 320 = 420  (pixel X)
v = 500 * 0.1 + 240 = 290  (pixel Y)
```

Key observations:
- A point directly ahead
  $$[0, 0, Z]^\top$$
  always projects to the principal point
  $$(c_x, c_y)$$
- As distance increases → the same-sized object appears smaller in the image (perspective effect)

> **Example 2** — Understanding Each Parameter of the K Matrix
>
> $$K = \begin{bmatrix} 500 & 0 & 320 \\\\ 0 & 500 & 240 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> Point
> $$P_1=[1, 0, 5]^\top$$
> (1m to the right) →
> $$x_n=[0.2, 0, 1]^\top$$
> →
> $$(u,v)=(420, 240)$$
>
> Point
> $$P_2=[0, 0, 5]^\top$$
> (directly ahead) →
> $$x_n=[0, 0, 1]^\top$$
> →
> $$(u,v)=(320, 240)$$
> = principal point ✓
>
> Point
> $$P_3=[0, 1, 5]^\top$$
> (1m above) →
> $$x_n=[0, 0.2, 1]^\top$$
> →
> $$(u,v)=(320, 340)$$
>
> Point
> $$P_4=[1, 0, 10]^\top$$
> (1m to the right, but farther) →
> $$x_n=[0.1, 0, 1]^\top$$
> →
> $$(u,v)=(370, 240)$$
>
> Compare
> $$P_1$$
> and
> $$P_4$$
> : same 1m to the right, distance changes from 5m to 10m → u changes from 420 to 370, closer to the principal point.

---

## 3.2 Camera Calibration

### 3.2.1 Calibration of Intrinsic Matrix K

Intrinsic parameters are **properties of the camera itself** and do not change with motion:

$$K = \begin{bmatrix} f_x & s & c_x \\\\ 0 & f_y & c_y \\\\ 0 & 0 & 1 \end{bmatrix}$$

- $$f_x, f_y$$
  : focal length (pixels)
- $$c_x, c_y$$
  : principal point
- $$s$$
  : **skew factor**, usually 0 in modern cameras

> **Example 3** — Estimating K from Camera Specifications
>
> Camera specs: sensor 6.4mm × 4.8mm, resolution 1280×960, focal length 8mm
>
> $$f_x = \frac{8\text{mm}}{6.4\text{mm}} \times 1280 = 1.25 \times 1280 = 1600 \text{ px}$$
> $$f_y = \frac{8\text{mm}}{4.8\text{mm}} \times 960 \approx 1.667 \times 960 = 1600 \text{ px}$$
>
> Principal point (assumed at center):
> $$c_x = 1280/2 = 640$$,
> $$c_y = 960/2 = 480$$
>
> $$K = \begin{bmatrix} 1600 & 0 & 640 \\\\ 0 & 1600 & 480 \\\\ 0 & 0 & 1 \end{bmatrix}$$

**Zhang's Calibration Method (Zhang's Method, 2000)**

Core idea: use a planar checkerboard with known physical square dimensions → capture from multiple viewpoints → solve for K.

Steps:
1. Print a checkerboard and measure the physical square dimensions
2. Capture ~20 images from different angles
3. Detect corners in each image (known world coordinates: checkerboard lies on Z=0 plane)
4. Estimate homography H for each image (3×3 matrix mapping checkerboard points to pixels)
5. Recover K (intrinsics) from multiple H matrices, and R,t (extrinsics) for each image

**Why is camera calibration needed?**
- Undistortion: wide-angle lenses have significant barrel distortion
- Accurate projection:
  $$f_x, f_y$$
  directly affect depth estimation accuracy
- SLAM practice: use the ROS camera_calibration package or Kalibr toolbox

---

### 3.2.2 Lens Distortion Model

Real lenses are imperfect; pixel positions deviate from ideal projection:

**Radial Distortion**:

$$x_{distorted} = x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$$

where
$$r^2 = x^2 + y^2$$
(distance to the principal point).

- $$k_1 > 0$$
  : pincushion distortion
- $$k_1 < 0$$
  : barrel distortion (common in wide-angle lenses)

> **Example 4** — Radial Distortion Calculation
>
> Normalized coordinates
> $$(x, y) = (0.2, 0.1)$$
> , distortion parameters
> $$k_1 = -0.3$$,
> $$k_2 = 0.1$$
>
> $$r^2 = 0.2^2 + 0.1^2 = 0.04 + 0.01 = 0.05$$
>
> Radial distortion coefficient:
> $$1 + k_1 r^2 + k_2 r^4 = 1 - 0.3\times0.05 + 0.1\times0.0025$$
> $$= 1 - 0.015 + 0.00025 = 0.98525$$
>
> $$x_{distorted} = 0.2 \times 0.98525 = 0.19705$$
> $$y_{distorted} = 0.1 \times 0.98525 = 0.09853$$
>
> Barrel distortion (
> $$k_1<0$$
> ) causes points to "shrink" inward (0.2→0.197).
>
> If
> $$k_1=0.3$$
> (pincushion):
> $$1 + 0.3\times0.05 = 1.015$$
> → points "bulge" outward.

**Tangential Distortion**:

$$x_{distorted} = x + [2p_1 xy + p_2(r^2 + 2x^2)]$$

Caused by the lens not being parallel to the sensor.

> **Example 5** — Tangential Distortion Calculation
>
> Normalized coordinates
> $$(x, y) = (0.2, 0.1)$$,
> $$p_1 = 0.001$$,
> $$p_2 = -0.0005$$
>
> $$r^2 = 0.05$$
> $$x_{distorted} = 0.2 + 2\times0.001\times0.2\times0.1 + (-0.0005)\times(0.05 + 2\times0.04)$$
> $$= 0.2 + 0.00004 + (-0.0005)\times(0.05 + 0.08) = 0.2 + 0.00004 - 0.000065$$
> $$= 0.199975$$
>
> Tangential distortion is usually very small (micron level), but remains important for high-precision calibration.

In SLAM practice: by default use the **plumb_bob** model or **equidistant** model (fisheye).

---

## 3.3 Rigid Body Motion Representation

### 3.3.1 Three Rotation Representations

| Representation | Variables | Constraints | Use in SLAM |
|------|------|------|-----------|
| **Rotation matrix R** | 3×3 matrix | $R^\top R=I$, $\det=1$ | Theoretical analysis |
| **Euler angles** | (roll, pitch, yaw) | Gimbal lock | Debugging/visualization |
| **Quaternion q** | $(w, x, y, z)$ | $\|q\|=1$ | Actual storage/interpolation |
| **Rotation vector (axis-angle)** | $\theta \mathbf{n}$ (3-dim) | None | Optimization/Lie algebra |

> **Example 6** — Euler Angles → Rotation Matrix (ZYX order)
>
> Given
> $$(\text{yaw}, \text{pitch}, \text{roll}) = (30^\circ, 15^\circ, 0^\circ)$$
>
> $$R_z(30^\circ) = \begin{bmatrix} 0.866 & -0.5 & 0 \\\\ 0.5 & 0.866 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$R_y(15^\circ) = \begin{bmatrix} 0.966 & 0 & 0.259 \\\\ 0 & 1 & 0 \\\\ -0.259 & 0 & 0.966 \end{bmatrix}$$
>
> $$R = R_z \cdot R_y \cdot R_x = \begin{bmatrix} 0.866 & -0.5 & 0 \\\\ 0.5 & 0.866 & 0 \\\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 0.966 & 0 & 0.259 \\\\ 0 & 1 & 0 \\\\ -0.259 & 0 & 0.966 \end{bmatrix}$$
>
> $$= \begin{bmatrix} 0.866\times0.966 + (-0.5)\times0 & 0.866\times0 + (-0.5)\times1 & 0.866\times0.259 + (-0.5)\times0.966 \\\\ 0.5\times0.966 + 0.866\times0 & 0.5\times0 + 0.866\times1 & 0.5\times0.259 + 0.866\times0.966 \\\\ -0.259 & 0 & 0.966 \end{bmatrix}$$
>
> $$= \begin{bmatrix} 0.837 & -0.5 & -0.259 \\\\ 0.483 & 0.866 & 0.130 \\\\ -0.259 & 0 & 0.966 \end{bmatrix}$$

### 3.3.2 Homogeneous Transformation

$$\begin{bmatrix} P' \\\\ 1 \end{bmatrix} = \begin{bmatrix} R & t \\\\ 0 & 1 \end{bmatrix} \begin{bmatrix} P \\\\ 1 \end{bmatrix}$$

**Composition and Inverse of Transformations**:

$$T_{ac} = T_{ab} \cdot T_{bc}$$

$$T^{-1} = \begin{bmatrix} R^\top & -R^\top t \\\\ 0 & 1 \end{bmatrix}$$

Practical tip in actual usage:
- `T_cw` = world → camera transformation (also called camera extrinsics)
- `T_wc` = camera → world transformation (also called camera pose)
  $$T_{cw} = T_{wc}^{-1}$$

> **Example 7** — Inverting a Homogeneous Transformation
>
> Camera pose
> $$T_{wc}$$
> : rotation about Z axis by
> $$90^\circ$$
> , translation
> $$[1, 2, 0]^\top$$
>
> $$T_{wc} = \begin{bmatrix} 0 & -1 & 0 & 1 \\\\ 1 & 0 & 0 & 2 \\\\ 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> Find
> $$T_{cw} = T_{wc}^{-1}$$
> (world→camera extrinsics):
>
> $$R^\top = \begin{bmatrix} 0 & 1 & 0 \\\\ -1 & 0 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$-R^\top t = -\begin{bmatrix} 0 & 1 & 0 \\\\ -1 & 0 & 0 \\\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 \\\\ 2 \\\\ 0 \end{bmatrix} = -\begin{bmatrix} 2 \\\\ -1 \\\\ 0 \end{bmatrix} = \begin{bmatrix} -2 \\\\ 1 \\\\ 0 \end{bmatrix}$$
>
> $$T_{cw} = \begin{bmatrix} 0 & 1 & 0 & -2 \\\\ -1 & 0 & 0 & 1 \\\\ 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> Verification:
> $$T_{wc} \cdot T_{cw} = I_{4\times4}$$
> ✓
>
> This means: the camera is at
> $$T_{wc}$$
> , and a world point
> $$P_w$$
> in camera coordinates is
> $$P_c = T_{cw} \cdot P_w$$

---

## 3.4 Epipolar Geometry — The Core of the Core

### 3.4.1 Problem Setup

You have two cameras that observe the same 3D point. There is relative motion
$$(R, t)$$
between the two cameras.

**The core question**: given a pixel
$$p_1$$
in image 1, its corresponding point
$$p_2$$
in image 2 must lie on a **single line**.

This line is called the **epipolar line**. This is the epipolar constraint.

### 3.4.2 Essential Matrix E

Defined in **normalized camera coordinates** (
$$x_n = K^{-1}p$$
):

$$E = t^\wedge R$$

where
$$t^\wedge$$
is the skew-symmetric matrix of
$$t$$
.

**Epipolar constraint** (normalized coordinates):
$$x_2^\top E x_1 = 0$$

$$E$$
is a
$$3\times3$$
matrix with 5 degrees of freedom (3 rotation + 3 translation - 1 scale ambiguity = 5), and has rank 2.

> **Example 8** — Constructing the Essential Matrix E
>
> Camera 2 relative to Camera 1: pure horizontal translation
> $$t = [1, 0, 0]^\top$$
> (1 unit to the right), no rotation
> $$R = I$$
>
> $$t^\wedge = \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix}$$
>
> $$E = t^\wedge R = \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix}$$
>
> Verify
> $$\det(E) = 0$$
> (characteristic of rank 2). Singular values:
> $$\sigma_1=1, \sigma_2=1, \sigma_3=0$$
> ✓
>
> ---
>
> **Example 8b** — Verifying the Epipolar Constraint
>
> 3D point
> $$P = [2, 1, 5]^\top$$
> , Camera 1 at origin, Camera 2 at
> $$[1, 0, 0]^\top$$
>
> $$x_1 = [2/5, 1/5, 1]^\top = [0.4, 0.2, 1]^\top$$,
> $$x_2 = [(2-1)/5, 1/5, 1]^\top = [0.2, 0.2, 1]^\top$$
>
> $$x_2^\top E x_1 = [0.2, 0.2, 1] \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} 0.4 \\\\ 0.2 \\\\ 1 \end{bmatrix}$$
> $$= [0.2, 0.2, 1] \begin{bmatrix} 0 \\\\ -1 \\\\ 0.2 \end{bmatrix} = 0.2\times0 + 0.2\times(-1) + 1\times0.2 = -0.2 + 0.2 = 0$$
>
> The epipolar constraint holds! ✓

### 3.4.3 Fundamental Matrix F

Defined in **pixel coordinates**:

$$F = K_2^{-\top} E K_1^{-1}$$

**Epipolar constraint** (pixel coordinates): This is the form used in actual computations.
$$p_2^\top F p_1 = 0$$

$$F$$
is a
$$3\times3$$
matrix, rank 2, with 7 degrees of freedom.

> **Example 9** — Computing F from E and K
>
> Assume both cameras use the same intrinsics:
> $$f_x=f_y=800$$,
> $$c_x=c_y=320$$
> (640×640 image)
>
> $$K = \begin{bmatrix} 800 & 0 & 320 \\\\ 0 & 800 & 320 \\\\ 0 & 0 & 1 \end{bmatrix}, \quad K^{-1} = \begin{bmatrix} 1/800 & 0 & -320/800 \\\\ 0 & 1/800 & -320/800 \\\\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0.00125 & 0 & -0.4 \\\\ 0 & 0.00125 & -0.4 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> Assume
> $$E$$
> as in Example 8:
> $$E = \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix}$$
>
> $$F = K^{-\top} E K^{-1} = \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -0.00125 \\\\ 0 & 0.00125 & 0 \end{bmatrix}$$
>
> (simplified computation: special case for pure horizontal translation)
>
> F maps a pixel in image 1 to an epipolar line in image 2:
> $$l_2 = F p_1$$

### 3.4.4 Computing F from Matched Points: The 8-Point Algorithm

**This is the most important algorithm in all of epipolar geometry.**

Given 8 or more pairs of matched points
$$\{(p_1^i, p_2^i)\}$$
, solve for
$$F$$

Each pair of points contributes one equation:
$$[u_1u_2,\ v_1u_2,\ u_2,\ u_1v_2,\ v_1v_2,\ v_2,\ u_1,\ v_1,\ 1] \cdot \text{vec}(F) = 0$$

Construct matrix
$$A$$
(one row per point pair), find the eigenvector corresponding to the smallest eigenvalue of
$$A^\top A$$
→
$$\text{vec}(F)$$

> **Example 10** — 8-Point Algorithm by Hand (simplified to 4 point pairs)
>
> Four matched point pairs (pixel coordinates):
> $$p_1^1 = [400, 300], p_2^1 = [380, 300]$$
> $$p_1^2 = [500, 300], p_2^2 = [480, 300]$$
> $$p_1^3 = [300, 400], p_2^3 = [280, 400]$$
> $$p_1^4 = [500, 500], p_2^4 = [480, 500]$$
>
> Construct matrix A (one row per point pair):
>
> For the 1st point pair
> $$[u_1=400, v_1=300, u_2=380, v_2=300]$$:
> $$[400\times380,\ 300\times380,\ 380,\ 400\times300,\ 300\times300,\ 300,\ 400,\ 300,\ 1]$$
> $$= [152000,\ 114000,\ 380,\ 120000,\ 90000,\ 300,\ 400,\ 300,\ 1]$$
>
> These values are too large; in practice, **normalization** is needed (see Hartley normalization below).
>
> After constructing A, perform SVD on
> $$A$$
> , take the last column of
> $$V$$
> → reshape to 3×3 → obtain F.
>
> Practical solution (simplified):
> ```python
> import numpy as np
> A = np.zeros((4, 9))
> for i, (p1, p2) in enumerate(matches):
>     u1, v1 = p1
>     u2, v2 = p2
>     A[i] = [u1*u2, v1*u2, u2, u1*v2, v1*v2, v2, u1, v1, 1]
> _, _, Vt = np.linalg.svd(A)
> F = Vt[-1].reshape(3, 3)
> # Enforce rank 2
> U, S, Vt = np.linalg.svd(F)
> S[2] = 0
> F = U @ np.diag(S) @ Vt
> ```

**Key post-processing**: SVD decomposition of
$$F$$
, set the smallest singular value to 0, reconstruct → enforce
$$\text{rank}(F)=2$$
.

**A more robust approach**: first normalize the matched points (shift to origin, scale so average distance =
$$\sqrt{2}$$
), then apply the 8-point algorithm, finally denormalize → Hartley's normalized 8-point algorithm.

> **Example 11** — Hartley Normalization
>
> X coordinates of 4 points: [400, 500, 300, 500], mean = 425
> Y coordinates: [300, 300, 400, 500], mean = 375
>
> Demean:
> $$x' = x - 425$$,
> $$y' = y - 375$$
> After demeaning: [(-25,-75), (75,-75), (-125,25), (75,125)]
>
> Average distance:
> $$(25^2+75^2)^{0.5} = 79.1$$,
> $$(75^2+75^2)^{0.5}=106.1$$
> , ... average ≈ 100
>
> Scaling factor:
> $$s = \sqrt{2} / 100 \approx 0.01414$$
>
> Normalization transform:
> $$T = \begin{bmatrix} 0.01414 & 0 & -425\times0.01414 \\\\ 0 & 0.01414 & -375\times0.01414 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> Compute
> $$T_1, T_2$$
> separately for the two images, compute
> $$\tilde{F}$$
> on normalized coordinates, then denormalize:
> $$F = T_2^\top \tilde{F} T_1$$

### 3.4.5 Recovering Camera Motion (R, t) from F/E

From
$$E = U \Sigma V^\top$$
(
$$\Sigma = \text{diag}(1,1,0)$$
):

$$R_1 = UWV^\top,\quad R_2 = UW^\top V^\top$$
$$t_1 = U_3,\quad t_2 = -U_3$$

where
$$W=\begin{bmatrix}0&-1&0\\\\1&0&0\\\\0&0&1\end{bmatrix}$$

There are 4 possible
$$(R,t)$$
combinations. **Selection criterion**: triangulate a point and check whether it lies in front of both cameras (positive depth).

> **Example 12** — Recovering R, t from E (numerical)
>
> Assume pure horizontal translation
> $$t=[1,0,0]^\top$$,
> $$R=I$$:
>
> $$E = t^\wedge R = \begin{bmatrix} 0 & 0 & 0 \\\\ 0 & 0 & -1 \\\\ 0 & 1 & 0 \end{bmatrix}$$
>
> SVD decomposition of E:
> $$U = \begin{bmatrix} 0 & 0 & 1 \\\\ 0 & -1 & 0 \\\\ 1 & 0 & 0 \end{bmatrix}, \quad \Sigma = \begin{bmatrix} 1 & 0 & 0 \\\\ 0 & 1 & 0 \\\\ 0 & 0 & 0 \end{bmatrix}, \quad V^\top = \begin{bmatrix} 0 & 1 & 0 \\\\ 1 & 0 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> Verify:
> $$U\Sigma V^\top = E$$
> ✓ (
> $$\Sigma$$
> is
> $$[1,1,0]$$
> — rank 2)
>
> Recover R and t:
>
> $$W = \begin{bmatrix} 0 & -1 & 0 \\\\ 1 & 0 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$R_1 = UWV^\top$$
> :
> $$UWV^\top = \begin{bmatrix} 0 & 0 & 1 \\\\ 0 & -1 & 0 \\\\ 1 & 0 & 0 \end{bmatrix} \begin{bmatrix} 0 & -1 & 0 \\\\ 1 & 0 & 0 \\\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 0 & 1 & 0 \\\\ 1 & 0 & 0 \\\\ 0 & 0 & 1 \end{bmatrix} = I$$
>
> $$t_{\pm 1} = U[:,2] = [1, 0, 0]^\top$$
> $$t_{\pm 2} = -U[:,2] = [-1, 0, 0]^\top$$
>
> 4 combinations:
> $$(R_1, t_{+1})$$,
> $$(R_1, t_{-1})$$,
> $$(R_2, t_{+1})$$,
> $$(R_2, t_{-1})$$
>
> > The **correct combination** is
> > $$(I, [1,0,0]^\top)$$
> > — the triangulated point must be in front of both cameras (positive depth).

---

## 3.5 Homography H

When all 3D points are **coplanar** (e.g., all lie on a ground plane), the 2D-2D correspondence is described by a homography matrix.

$$p_2 \sim H p_1$$

$$H$$
is a
$$3\times3$$
matrix with 8 degrees of freedom.

> **Example 13** — Constructing a Ground Plane Homography
>
> Ground plane:
> $$Z=0$$
> (i.e., the world XY plane)
>
> Camera 1 at origin looking down at the ground:
> $$R_1=I$$,
> $$t_1=[0, 0, -h]^\top$$
> (camera at height h above)
> Camera 2 at same height but displaced:
> $$R_2=I$$,
> $$t_2=[d, 0, -h]^\top$$
>
> Homography matrix:
> $$H = K_2(R - t n^\top/d)K_1^{-1}$$
>
> where
> $$n=[0,0,1]^\top$$
> (ground plane normal vector),
> $$d=h$$
> (distance from camera to plane)
>
> Assuming
> $$h=1.5$$
> , focal length=800, principal point=(320,240):
> $$H = K \begin{bmatrix} 1 & 0 & 0 \\\\ 0 & 1 & 0 \\\\ 0 & 0 & 1-h/h \end{bmatrix} K^{-1} = K \begin{bmatrix} 1 & 0 & 0 \\\\ 0 & 1 & 0 \\\\ 0 & 0 & 0 \end{bmatrix} K^{-1}$$
>
> This is a degenerate case (points on the
> $$Z=0$$
> plane). In practice, H is a full-rank 3×3 matrix.

**Key usage in ORB-SLAM**: during initialization, compute both
$$F$$
and
$$H$$
simultaneously, and select whichever fits the data better (because the scene might be planar).

---

## 3.6 Triangulation

**Problem**: given the projection matrices
$$P_1, P_2$$
of two cameras and matched points
$$(p_1, p_2)$$
, find the 3D point coordinates.

**DLT Method (Linear Triangulation)**:

For each camera:
$$p \times (PX) = 0$$
(cross product form)

Each camera contributes two independent equations:
$$[u P_3 - P_1] X = 0$$
$$[v P_3 - P_2] X = 0$$

Two cameras give 4 equations total; use SVD to find the least-squares solution →
$$X$$

> **Example 14** — DLT Triangulation by Hand
>
> Camera 1 at origin:
> $$P_1 = K[I \mid 0]$$
> , Camera 2 pure translation:
> $$P_2 = K[I \mid t]$$
> where
> $$t=[1,0,0]^\top$$
>
> To simplify, use normalized coordinates (K=I):
> $$P_1 = \begin{bmatrix}1&0&0&0\\\\0&1&0&0\\\\0&0&1&0\end{bmatrix}$$,
> $$P_2 = \begin{bmatrix}1&0&0&1\\\\0&1&0&0\\\\0&0&1&0\end{bmatrix}$$
>
> Matched points (normalized coordinates):
> $$x_1 = [0.4, 0.2, 1]^\top$$,
> $$x_2 = [0.2, 0.2, 1]^\top$$
>
> For Camera 1:
> $$[u_1 P_{1,3} - P_{1,1}]X = 0$$
> $$P_{1,1} = [1,0,0,0], \quad P_{1,3} = [0,0,1,0]$$
> $$0.4 \times [0,0,1,0] - [1,0,0,0] = [-1, 0, 0.4, 0]$$
>
> Similarly
> $$[v_1 P_{1,3} - P_{1,2}]X = [0, -1, 0.2, 0]$$
>
> For Camera 2:
> $$[u_2 P_{2,3} - P_{2,1}]X = [-1, 0, 0.2, -1]$$,
> $$[v_2 P_{2,3} - P_{2,2}]X = [0, -1, 0.2, 0]$$
>
> Form
> $$A X = 0$$:
> $$A = \begin{bmatrix} -1 & 0 & 0.4 & 0 \\\\ 0 & -1 & 0.2 & 0 \\\\ -1 & 0 & 0.2 & -1 \\\\ 0 & -1 & 0.2 & 0 \end{bmatrix}$$
>
> SVD solution →
> $$X = [2, 1, 5, 1]^\top$$
> →
> $$P = [2, 1, 5]^\top$$
> ✓ (ground truth!)

**Triangulation accuracy**:
- Larger baseline (wider camera separation) → more accurate depth estimation
- Farther point from camera → greater depth uncertainty
- Point near epipolar direction → increased uncertainty

---

## 3.7 Projective Space and Vanishing Points

### 3.7.1 Homogeneous Coordinates

The homogeneous representation of a 3D point
$$[X,Y,Z]^\top$$
:
$$[X,Y,Z,1]^\top$$

$$[X,Y,Z,W]^\top$$
is equivalent to
$$[X/W, Y/W, Z/W, 1]^\top$$
(
$$W\neq0$$
).

$$[X,Y,Z,0]^\top$$
represents a **point at infinity** (direction vector).

> **Example 15** — Homogeneous Coordinate Operations
>
> The homogeneous representation of point
> $$[3, 6, 9]^\top$$
> :
> $$[3, 6, 9, 1]^\top \sim [1, 2, 3, 1/3]^\top \sim [6, 12, 18, 2]^\top$$
>
> Recover from homogeneous:
> $$[6, 12, 18, 2]^\top$$
> → divide by
> $$W$$
> →
> $$[3, 6, 9]^\top$$
> ✓
>
> Direction vector (point at infinity):
> $$[1, 0, 0, 0]^\top$$
> represents the X-axis direction.
>
> Projecting a point at infinity:
> $$P \cdot [1, 0, 0, 0]^\top = K[R|t] \cdot [1,0,0,0]^\top = K \cdot R \cdot [1,0,0]^\top$$
> → **vanishing point**!

### 3.7.2 Vanishing Point

Parallel lines converge to a point in the image → **vanishing point**.

$$\text{vanishing point} = K \cdot d$$

where
$$d$$
is the direction vector of the parallel lines.

> **Example 16** — Vanishing Point Calculation
>
> Indoor corridor: direction of parallel lines in Y direction (vertical)
> $$d = [0, 1, 0]^\top$$
>
> Camera intrinsic
> $$K$$:
>
> $$\text{VP}_Y = K \cdot [0, 1, 0]^\top = \begin{bmatrix} 800 & 0 & 320 \\\\ 0 & 800 & 240 \\\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 0 \\\\ 1 \\\\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\\\ 800 \\\\ 0 \end{bmatrix}$$
>
> Normalize:
> $$\text{VP}_Y = (0/0, 800/0)$$
> → at infinity! (Vertical parallel lines remain parallel in the image, because the vanishing point for the Y-axis is at infinity — when the camera is level)
>
> For the Z direction (depth forward):
> $$d = [0, 0, 1]^\top$$
> $$\text{VP}_Z = K \cdot [0, 0, 1]^\top = [320, 240, 1]^\top$$
> → pixel (320, 240) = principal point!
>
> Parallel lines along the depth direction converge at the principal point — this is the principle of **one-point perspective**.

Uses in SLAM:
- Extracting directional information from structured scenes like buildings
- Manhattan World assumption: lines in indoor scenes are primarily along three orthogonal directions

---

## 📝 Self-Check Checklist

- [ ] Can you draw the projection pipeline of the pinhole camera model by hand?
- [ ] What is the physical meaning of each parameter in the K matrix?
- [ ] Difference between E and F? In which coordinate system is each defined?
- [ ] Steps of the 8-point algorithm? (normalization → construct A matrix → SVD → rank-2 constraint → denormalization)
- [ ] How many combinations when recovering (R,t) from E? How to select the correct one?
- [ ] Why does DLT triangulation use the cross product? What does cross product = 0 mean?
- [ ] Why is det(R) = 1 for rotation matrices?

---

> **Next step**: complete `exercises/exercise_03_projective_geometry.py` to implement these algorithms hands-on.
