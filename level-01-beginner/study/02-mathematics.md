# Module 2: Mathematical Foundations Required for SLAM

> This module covers all knowledge points of "Mathematics" in visual-slam-roadmap Level 1.
> The goal is to enable you to understand all mathematical formulas in SLAM papers.
> Each formula is accompanied by a **complete numerical calculation example**.

---

## 2.1 Linear Algebra — The Backbone of SLAM

In SLAM, everything takes place in **vector space**: 3D point coordinates, camera velocity, and translation vectors are all vectors; rotations and transformations are matrices.

### 2.1.1 Vector

Physical meaning: a point or direction in space.

```text
a = [3, 0, 0]ᵀ    # direction along X-axis (magnitude = 3)
b = [0, 4, 0]ᵀ    # direction along Y-axis
```

#### Vector Magnitude (L2 Norm)

$$\|a\| = \sqrt{a_1^2 + a_2^2 + a_3^2}$$

> **Example 1** — Computing vector magnitude
>
> Given
>
> $$a = [3, 0, 0]^\top$$
>
> $$\|a\| = \sqrt{3^2 + 0^2 + 0^2} = \sqrt{9} = 3$$
>
> Given
>
> $$v = [1, 2, 2]^\top$$
>
> $$\|v\| = \sqrt{1^2 + 2^2 + 2^2} = \sqrt{1 + 4 + 4} = \sqrt{9} = 3$$
>
> In SLAM, the Euclidean distance between two 3D points
>
> $$P_1=[1,2,3]^\top$$
>
> $$P_2=[4,6,3]^\top$$
>
> is:
>
> $$\|P_2 - P_1\| = \|[3, 4, 0]^\top\| = \sqrt{9 + 16 + 0} = \sqrt{25} = 5$$

#### Dot Product

$$a \cdot b = a^\top b = a_1b_1 + a_2b_2 + a_3b_3$$

Geometric meaning:

$$a \cdot b = \| a \| \cdot \| b \| \cdot \cos\theta$$

> **Example 2** — Dot product and angle calculation
>
> $$a = [1, 2, 3]^\top$$
>
> $$b = [4, -5, 6]^\top$$
>
> $$a \cdot b = 1\times4 + 2\times(-5) + 3\times6 = 4 - 10 + 18 = 12$$
>
> Magnitudes:
>
> $$\|a\| = \sqrt{1+4+9} = \sqrt{14} \approx 3.742$$
>
> $$\|b\| = \sqrt{16+25+36} = \sqrt{77} \approx 8.775$$
>
> Angle:
>
> $$\cos\theta = \frac{a \cdot b}{\|a\|\|b\|} = \frac{12}{3.742 \times 8.775} \approx 0.3653$$
>
> $$\theta \approx 68.6^\circ$$
>
> ---
>
> **Example 2b** — Orthogonality check (commonly used in SLAM)
>
> $$a = [1, 0, 0]^\top$$
>
> $$b = [0, 1, 0]^\top$$
>
> $$a \cdot b = 1\times0 + 0\times1 + 0\times0 = 0$$
>
> Dot product is 0 → the two vectors are **orthogonal** (perpendicular). The X-axis and Y-axis are perpendicular to each other.

- If

  $$a \cdot b = 0$$

  , the two vectors are **orthogonal** (commonly used in SLAM to check whether directions are perpendicular)
  If

  $$a \cdot b > 0$$

  , the angle < 90° — used to compute the sum of squared pixel errors in **reprojection error**
- 

#### Cross Product

$$a \times b = \begin{bmatrix} a_2b_3 - a_3b_2 \\\\ a_3b_1 - a_1b_3 \\\\ a_1b_2 - a_2b_1 \end{bmatrix}$$

Geometric meaning: the resulting vector is perpendicular to both a and b, with magnitude =

$$\|a\| \cdot \|b\| \cdot \sin\theta$$

> **Example 3** — Computing the cross product
>
> $$a = [1, 2, 3]^\top$$
>
> $$b = [4, 5, 6]^\top$$
>
> $$a \times b = \begin{bmatrix} 2\times6 - 3\times5 \\\\ 3\times4 - 1\times6 \\\\ 1\times5 - 2\times4 \end{bmatrix} = \begin{bmatrix} 12 - 15 \\\\ 12 - 6 \\\\ 5 - 8 \end{bmatrix} = \begin{bmatrix} -3 \\\\ 6 \\\\ -3 \end{bmatrix}$$
>
> Verify perpendicularity:
>
> $$(a \times b) \cdot a = -3\times1 + 6\times2 + (-3)\times3 = -3+12-9 = 0$$
>
> ✓
>
> Uses in SLAM:
> - Computing **plane normal vectors**: the cross product of two non-parallel vectors in space yields the normal vector. For example,
> -
>
> $$a=[1,0,0]^\top$$
>
>   $$b=[0,1,0]^\top$$
>
>   , then
>
>   $$a\times b=[0,0,1]^\top$$
>
>   (the Z-axis normal vector, i.e., the normal of the XY plane)
> - Constructing the **skew-symmetric matrix** (used for the essential matrix in epipolar geometry)

#### Skew-Symmetric Matrix

Any vector

$$v = [x, y, z]^\top$$

can be used to construct:

$$v^\wedge = \begin{bmatrix} 0 & -z & y \\\\ z & 0 & -x \\\\ -y & x & 0 \end{bmatrix}$$

Property:

$$a^\wedge b = a \times b$$

(skew-symmetric matrix times vector = cross product)

> **Example 4** — Constructing a skew-symmetric matrix and verifying
>
> $$v = [1, 2, 3]^\top$$
>
> $$v^\wedge = \begin{bmatrix} 0 & -3 & 2 \\\\ 3 & 0 & -1 \\\\ -2 & 1 & 0 \end{bmatrix}$$
>
> Verify:
>
> $$v^\wedge b = v \times b$$
>
> (using
>
> $$b=[4,5,6]^\top$$
>
> )
>
> $$v^\wedge b = \begin{bmatrix} 0 & -3 & 2 \\\\ 3 & 0 & -1 \\\\ -2 & 1 & 0 \end{bmatrix} \begin{bmatrix} 4 \\\\ 5 \\\\ 6 \end{bmatrix} = \begin{bmatrix} 0\times4 + (-3)\times5 + 2\times6 \\\\ 3\times4 + 0\times5 + (-1)\times6 \\\\ (-2)\times4 + 1\times5 + 0\times6 \end{bmatrix} = \begin{bmatrix} -15+12 \\\\ 12-6 \\\\ -8+5 \end{bmatrix} = \begin{bmatrix} -3 \\\\ 6 \\\\ -3 \end{bmatrix}$$
>
> This matches the result of
>
> $$v \times b$$
>
> from Example 3 ✓
>
> This is exactly the
>
> $$t^\wedge$$
>
> in the **essential matrix**
>
> $$E = t^\wedge R$$

---

### 2.1.2 Matrix

**The most important matrix types in SLAM:**

| Matrix Type | Symbol | Use |
|-------------|--------|-----|
| Rotation Matrix | $R$ | $R^\top R = I$, $\det(R)=1$: 3D rotation transformation |
| Camera Intrinsics | $K$ | Maps 3D camera coordinates to 2D pixels |
| Projection Matrix | $P = K[R \mid t]$ | World 3D → Image 2D |
| Hessian Matrix | $H = J^\top J$ | Second-order information in optimization |
| Covariance Matrix | $\Sigma$ | Describes state uncertainty |

#### Matrix Rank

The rank of a matrix = the number of linearly independent rows/columns.

> **Example 5** — Computing the rank of a matrix
>
> $$A = \begin{bmatrix} 1 & 2 & 3 \\\\ 2 & 4 & 6 \\\\ 3 & 6 & 9 \end{bmatrix}$$
>
> Observe: row 2 = row 1 × 2, row 3 = row 1 × 3 → only 1 independent row →
>
> $$\text{rank}(A) = 1$$
>
> $$B = \begin{bmatrix} 1 & 2 & 3 \\\\ 4 & 5 & 6 \\\\ 7 & 8 & 9 \end{bmatrix}$$
>
> Row 3 = 2×Row 2 − Row 1 (
>
> $$[14,16,18] - [1,2,3] = [13,14,15]$$
>
> ≠ [7,8,9])... Actually, the rank of this matrix = 2 (determinant is 0, but the first two rows are linearly independent), because
>
> $$\det(B) = 1(45-48) - 2(36-42) + 3(32-35) = -3 + 12 - 9 = 0$$

- In SLAM:
  The

  $$E$$

  matrix must have rank 2 (after recovery via the eight-point algorithm, SVD must be used to force the smallest singular value to zero)
  The rank of the

  $$H$$

  matrix determines the uniqueness of the solution (rank deficiency → prior constraints are needed)

#### Determinant

> **Example 6** — Computing determinants
>
> 2×2 matrix:
>
> $$\det\begin{bmatrix} a & b \\\\ c & d \end{bmatrix} = ad - bc$$
>
> $$A = \begin{bmatrix} 2 & 1 \\\\ 3 & 4 \end{bmatrix} \quad \Rightarrow \quad \det(A) = 2\times4 - 1\times3 = 8 - 3 = 5$$
>
> 3×3 matrix:
>
> $$R_z(30^\circ) = \begin{bmatrix} \cos30^\circ & -\sin30^\circ & 0 \\\\ \sin30^\circ & \cos30^\circ & 0 \\\\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0.866 & -0.5 & 0 \\\\ 0.5 & 0.866 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$\det(R_z) = 0.866 \times (0.866\times1 - 0\times0) - (-0.5) \times (0.5\times1 - 0\times0) + 0 \times (...)$$
> $$= 0.866 \times 0.866 + 0.5 \times 0.5 = 0.75 + 0.25 = 1$$
>
> Rotation matrix
>
> $$\det(R) = 1$$
>
> (preserves volume and handedness) ✓

-

  $$\det(R) = 1$$

  : rotation matrices preserve volume

  $$\det(H)$$

  : indicator of the solvability of an optimization problem

#### Inverse Matrix

- Special property of rotation matrices:

  $$R^{-1} = R^\top$$

  (orthogonal matrix). General matrix inversion uses LU decomposition or SVD.
  In SLAM,

  $$K^{-1}$$

  is used to recover normalized camera coordinates from pixel coordinates

> **Example 7** — Computing the inverse matrix
>
> $$A = \begin{bmatrix} 2 & 1 \\\\ 3 & 4 \end{bmatrix}$$
>
> 2×2 inverse formula:
>
> $$A^{-1} = \frac{1}{\det(A)}\begin{bmatrix} d & -b \\\\ -c & a \end{bmatrix} = \frac{1}{ad-bc}\begin{bmatrix} d & -b \\\\ -c & a \end{bmatrix}$$
>
> $$A^{-1} = \frac{1}{5}\begin{bmatrix} 4 & -1 \\\\ -3 & 2 \end{bmatrix} = \begin{bmatrix} 0.8 & -0.2 \\\\ -0.6 & 0.4 \end{bmatrix}$$
>
> Verification:
>
> $$A A^{-1} = \begin{bmatrix} 2 & 1 \\\\ 3 & 4 \end{bmatrix} \begin{bmatrix} 0.8 & -0.2 \\\\ -0.6 & 0.4 \end{bmatrix} = \begin{bmatrix} 1.6-0.6 & -0.4+0.4 \\\\ 2.4-2.4 & -0.6+1.6 \end{bmatrix} = \begin{bmatrix} 1 & 0 \\\\ 0 & 1 \end{bmatrix}$$
>
> ✓
>
> ---
>
> **Example 7b** — Special property of rotation matrices
>
> $$R = R_z(30^\circ) = \begin{bmatrix} 0.866 & -0.5 & 0 \\\\ 0.5 & 0.866 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$R^{-1} = R^\top = \begin{bmatrix} 0.866 & 0.5 & 0 \\\\ -0.5 & 0.866 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> (i.e., a rotation of
>
> $$-30^\circ$$
>
> , because
>
> $$\cos(-30^\circ)=\cos30^\circ$$
>
> $$\sin(-30^\circ)=-\sin30^\circ$$
>
> )

#### QR Decomposition

$$A = QR$$

Q is an orthogonal matrix, R is an upper triangular matrix. Used in SLAM for stable solutions to least-squares problems.

> **Example 8** — QR decomposition (Gram-Schmidt)
>
> $$A = \begin{bmatrix} 1 & 2 \\\\ 3 & 2 \end{bmatrix}$$
>
> Column vectors:
>
> $$a_1 = [1,3]^\top$$
>
> $$a_2 = [2,2]^\top$$
>
> **Step 1**:
>
> $$u_1 = a_1 = [1,3]^\top$$
> $$q_1 = \frac{u_1}{\|u_1\|} = \frac{[1,3]^\top}{\sqrt{10}} \approx [0.3162, 0.9487]^\top$$
>
> **Step 2** projection:
>
> $$r_{12} = q_1^\top a_2 = 0.3162\times2 + 0.9487\times2 = 2.5298$$
> $$u_2 = a_2 - r_{12}q_1 = \begin{bmatrix} 2 \\\\ 2 \end{bmatrix} - 2.5298\begin{bmatrix} 0.3162 \\\\ 0.9487 \end{bmatrix} = \begin{bmatrix} 2 - 0.8 \\\\ 2 - 2.4 \end{bmatrix} = \begin{bmatrix} 1.2 \\\\ -0.4 \end{bmatrix}$$
> $$q_2 = \frac{u_2}{\|u_2\|} = \frac{[1.2,-0.4]^\top}{\sqrt{1.44+0.16}} = \frac{[1.2,-0.4]^\top}{\sqrt{1.6}} \approx [0.9487, -0.3162]^\top$$
>
> **Result**:
>
> $$Q = \begin{bmatrix} 0.3162 & 0.9487 \\\\ 0.9487 & -0.3162 \end{bmatrix}, \quad R = \begin{bmatrix} \|u_1\| & r_{12} \\\\ 0 & \|u_2\| \end{bmatrix} = \begin{bmatrix} 3.1623 & 2.5298 \\\\ 0 & 1.2649 \end{bmatrix}$$
>
> Verification:
>
> $$QR = A$$
>
> ...
>
> $$\begin{bmatrix}0.3162\times3.1623 & 0.3162\times2.5298+0.9487\times1.2649 \\\\ 0.9487\times3.1623 & 0.9487\times2.5298+(-0.3162)\times1.2649\end{bmatrix} = \begin{bmatrix}1 & 2 \\\\ 3 & 2\end{bmatrix} = A$$
>
> ✓

---

### 2.1.3 Singular Value Decomposition (SVD) — Most Important!

$$A = U \Sigma V^\top$$

- U: left singular vectors (orthonormal basis for the column space of A)
  Σ: diagonal matrix with non-negative singular values in descending order
  V: right singular vectors (orthonormal basis for the row space of A)

> **Example 9** — SVD worked out by hand (2×2 matrix)
>
> $$A = \begin{bmatrix} 3 & 1 \\\\ 1 & 3 \end{bmatrix}$$
>
> **Step 1** Compute
>
> $$A^\top A$$
> $$A^\top A = \begin{bmatrix} 3 & 1 \\\\ 1 & 3 \end{bmatrix} \begin{bmatrix} 3 & 1 \\\\ 1 & 3 \end{bmatrix} = \begin{bmatrix} 10 & 6 \\\\ 6 & 10 \end{bmatrix}$$
>
> **Step 2** Find the eigenvalues and eigenvectors of
>
> $$A^\top A$$
> $$\det(A^\top A - \lambda I) = \det\begin{bmatrix} 10-\lambda & 6 \\\\ 6 & 10-\lambda \end{bmatrix} = (10-\lambda)^2 - 36 = 0$$
> $$\lambda^2 - 20\lambda + 64 = 0 \quad \Rightarrow \quad \lambda_1 = 16,\ \lambda_2 = 4$$
>
> Singular values:
>
> $$\sigma_1 = \sqrt{16} = 4$$
>
> $$\sigma_2 = \sqrt{4} = 2$$
>
> **Step 3** Find the eigenvectors (i.e., the columns of
>
> $$V$$
>
> )
>
> For
>
> $$\lambda_1=16$$
>
> $$(A^\top A - 16I)v_1 = \begin{bmatrix} -6 & 6 \\\\ 6 & -6 \end{bmatrix} v_1 = 0 \Rightarrow v_1 = \frac{1}{\sqrt{2}}[1, 1]^\top$$
>
> For
>
> $$\lambda_2=4$$
>
> $$(A^\top A - 4I)v_2 = \begin{bmatrix} 6 & 6 \\\\ 6 & 6 \end{bmatrix} v_2 = 0 \Rightarrow v_2 = \frac{1}{\sqrt{2}}[1, -1]^\top$$
>
> $$V = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 & 1 \\\\ 1 & -1 \end{bmatrix} \approx \begin{bmatrix} 0.7071 & 0.7071 \\\\ 0.7071 & -0.7071 \end{bmatrix}$$
>
> **Step 4**: Find the columns of U
>
> $$U$$
>
> $$u_i = Av_i / \sigma_i$$
>
> $$u_1 = \frac{1}{4}\begin{bmatrix} 3 & 1 \\\\ 1 & 3 \end{bmatrix} \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\\\ 1 \end{bmatrix} = \frac{1}{4\sqrt{2}}\begin{bmatrix} 4 \\\\ 4 \end{bmatrix} = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\\\ 1 \end{bmatrix}$$
> $$u_2 = \frac{1}{2}\begin{bmatrix} 3 & 1 \\\\ 1 & 3 \end{bmatrix} \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\\\ -1 \end{bmatrix} = \frac{1}{2\sqrt{2}}\begin{bmatrix} 2 \\\\ -2 \end{bmatrix} = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\\\ -1 \end{bmatrix}$$
>
> **Final SVD**:
>
> $$A = \begin{bmatrix} 0.7071 & 0.7071 \\\\ 0.7071 & -0.7071 \end{bmatrix} \begin{bmatrix} 4 & 0 \\\\ 0 & 2 \end{bmatrix} \begin{bmatrix} 0.7071 & 0.7071 \\\\ 0.7071 & -0.7071 \end{bmatrix}$$

**4 key uses of SVD in SLAM:**

1. **Solving homogeneous linear systems (eight-point algorithm for F/E matrices)**

   $$Ax = 0$$

   Solution = the column of V corresponding to the smallest singular value (the last column)

2. **Solving the least-squares solution of a non-homogeneous system**

   $$Ax = b$$

   $$x = V \Sigma^{-1} U^\top b$$

3. **Finding the optimal rigid-body transformation from point cloud correspondences in ICP**

   $$H = \sum (p_i - \bar{p})(q_i - \bar{q})^\top$$

   SVD decomposition of H yields the optimal rotation

4. **Enforcing matrix rank constraints**
   For example, the

   $$E$$

   matrix must have rank 2 → set the smallest singular value to zero and reconstruct

> **Example 10** — SVD solving a least-squares problem
>
> $$A = \begin{bmatrix} 1 & 2 \\\\ 3 & 4 \\\\ 5 & 6 \end{bmatrix}, \quad b = \begin{bmatrix} 1 \\\\ 1 \\\\ 1 \end{bmatrix}$$
>
> Find
>
> $$x$$
>
> that minimizes
>
> $$\|Ax - b\|^2$$
>
> SVD solution:
>
> $$x = V \Sigma^{-1} U^\top b$$
>
> Using numpy (conceptual demonstration):
> ```python
> import numpy as np
> A = np.array([[1,2],[3,4],[5,6]])
> b = np.array([1,1,1])
> U, S, Vt = np.linalg.svd(A, full_matrices=False)
> x = Vt.T @ np.diag(1/S) @ U.T @ b
> # x ≈ [-1.0, 1.0]
> # Verify: Ax = [1,1,1]ᵀ = b (happens to have an exact solution!)
> ```

**Practical example (computing the F matrix via the eight-point algorithm):**
```python
A = ...  # 8×9 matrix, each row comes from the epipolar constraint of a pair of matched points
_, _, Vt = np.linalg.svd(A)
F = Vt[-1].reshape(3, 3)      # solution corresponding to the smallest singular value

# Enforce rank 2
U, S, Vt = np.linalg.svd(F)
S[2] = 0                       # set the smallest singular value to zero
F_rank2 = U @ np.diag(S) @ Vt  # valid essential matrix
```

---

### 2.1.4 Eigenvalues and Eigenvectors

$$A v = \lambda v$$

-

  $$v$$

  : eigenvector (direction unchanged after transformation)

  $$\lambda$$

  : eigenvalue (scaling factor after transformation)

> **Example 11** — Computing eigenvalues and eigenvectors by hand
>
> $$A = \begin{bmatrix} 2 & 1 \\\\ 1 & 2 \end{bmatrix}$$
>
> **Step 1** Characteristic equation
>
> $$\det(A - \lambda I) = 0$$
> $$\det\begin{bmatrix} 2-\lambda & 1 \\\\ 1 & 2-\lambda \end{bmatrix} = (2-\lambda)^2 - 1 = 0$$
> $$\lambda^2 - 4\lambda + 3 = 0 \quad \Rightarrow \quad (\lambda-1)(\lambda-3) = 0$$
> $$\lambda_1 = 3,\ \lambda_2 = 1$$
>
> **Step 2**: Find the eigenvectors
>
> For
>
> $$\lambda_1=3$$
>
> $$(A-3I)v = \begin{bmatrix} -1 & 1 \\\\ 1 & -1 \end{bmatrix}v = 0 \Rightarrow v_1 = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\\\ 1 \end{bmatrix}$$
>
> For
>
> $$\lambda_2=1$$
>
> $$(A-I)v = \begin{bmatrix} 1 & 1 \\\\ 1 & 1 \end{bmatrix}v = 0 \Rightarrow v_2 = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 \\\\ -1 \end{bmatrix}$$
>
> **Verification**:
>
> $$Av_1 = \begin{bmatrix}2&1\\\\1&2\end{bmatrix}\frac{1}{\sqrt{2}}\begin{bmatrix}1\\\\1\end{bmatrix} = \frac{1}{\sqrt{2}}\begin{bmatrix}3\\\\3\end{bmatrix} = 3v_1$$
>
> ✓

**Key applications in SLAM:**

1. **Harris corner detection**
   The two eigenvalues of the image gradient covariance matrix

   $$M$$

   : -

  $$\lambda_1, \lambda_2$$

  both large → **corner** (large gradients in both directions)

  $$\lambda_1 \gg \lambda_2$$

  or vice versa → **edge** (large gradient in only one direction)
  both small → **flat region**

   > **Example 12** — Computing the Harris response
   >
   > Suppose the structure tensor at a certain pixel is:
   >
   $$M = \begin{bmatrix} 100 & 50 \\\\ 50 & 100 \end{bmatrix}$$
   >
   > Eigenvalues:
   >
   $$\det(M-\lambda I) = (100-\lambda)^2 - 2500 = 0$$
   >
   $$\lambda^2 - 200\lambda + 7500 = 0 \Rightarrow \lambda_1 = 150,\ \lambda_2 = 50$$
   >
   > Both eigenvalues are large → **corner**!
   >
   > Harris response (
   >
   $$k=0.04$$

   > ):
   >
   $$R = \det(M) - k\cdot\text{trace}(M)^2 = (100\times100 - 50\times50) - 0.04 \times (200)^2$$
   >
   $$= 7500 - 0.04 \times 40000 = 7500 - 1600 = 5900$$
   >
   > Compare with a flat region:
   >
   $$M = \begin{bmatrix} 1 & 0.1 \\\\ 0.1 & 1 \end{bmatrix}$$

   >
   $$\det=0.99$$

   >
   $$\text{trace}=2$$
   >
   $$R = 0.99 - 0.04 \times 4 = 0.83 \quad\text{(small → flat)}$$

   Harris response:

   $$R = \lambda_1\lambda_2 - k(\lambda_1+\lambda_2)^2$$

   is equivalent to

   $$R = \det(M) - k \cdot \text{trace}(M)^2$$

2. **PCA (Principal Component Analysis)**
   The eigenvectors of the covariance matrix = the principal directions of the data
   Used in SLAM for point cloud normal estimation

3. **Observability Analysis**
   The eigenvalues of the FIM (Fisher Information Matrix) in VIO systems:
   - Zero eigenvalues → correspond to unobservable states (e.g., global yaw angle in VIO)

---

### 2.1.5 Rigid Body Transformations and Homogeneous Coordinates

#### Rotation Matrix R ∈ SO(3)

$$SO(3) = \{R \in \mathbb{R}^{3\times3} \mid R^\top R = I,\ \det(R) = 1\}$$

3 degrees of freedom (one rotation angle about each of the X, Y, Z axes).

Rotation about the Z axis by

$$\theta$$

$$R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\\\ \sin\theta & \cos\theta & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$

Rotation about the X axis by

$$\theta$$

$$R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\\\ 0 & \cos\theta & -\sin\theta \\\\ 0 & \sin\theta & \cos\theta \end{bmatrix}$$

Rotation about the Y axis by

$$\theta$$

$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\\\ 0 & 1 & 0 \\\\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

> **Example 13** — Rotating a 3D point
>
> Rotate about the Z axis by
>
> $$45^\circ$$
>
> (
>
> $$\cos45^\circ = \sin45^\circ = \frac{\sqrt{2}}{2} \approx 0.7071$$
>
> )
>
> $$R_z(45^\circ) = \begin{bmatrix} 0.7071 & -0.7071 & 0 \\\\ 0.7071 & 0.7071 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> Point
>
> $$P = [2, 0, 3]^\top$$
>
> after rotation:
>
> $$P' = R_z P = \begin{bmatrix} 0.7071 & -0.7071 & 0 \\\\ 0.7071 & 0.7071 & 0 \\\\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 2 \\\\ 0 \\\\ 3 \end{bmatrix} = \begin{bmatrix} 0.7071\times2 + (-0.7071)\times0 + 0\times3 \\\\ 0.7071\times2 + 0.7071\times0 + 0\times3 \\\\ 0\times2 + 0\times0 + 1\times3 \end{bmatrix} = \begin{bmatrix} 1.414 \\\\ 1.414 \\\\ 3 \end{bmatrix}$$
>
> The point originally on the X-axis
>
> $$(2,0,3)$$
>
> , after a
>
> $$45^\circ$$
>
> rotation, ends up at
>
> $$(1.414, 1.414, 3)$$
>
> , with Z unchanged.

#### Homogeneous Transformation Matrix T ∈ SE(3)

$$T = \begin{bmatrix} R & t \\\\ 0 & 1 \end{bmatrix} \in \mathbb{R}^{4\times4}$$

6 degrees of freedom (3 rotation + 3 translation).

Transforming a point:

$$p' = T p$$

, where

$$p$$

is the homogeneous coordinate

$$[x, y, z, 1]^\top$$

> **Example 14** — Complete rigid-body transformation
>
> First rotate about the Z axis by
>
> $$90^\circ$$
>
> (
>
> $$\cos90^\circ=0$$
>
> $$\sin90^\circ=1$$
>
> ), then translate by
>
> $$[1, 2, 0]^\top$$
>
> $$R = R_z(90^\circ) = \begin{bmatrix} 0 & -1 & 0 \\\\ 1 & 0 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}, \quad t = \begin{bmatrix} 1 \\\\ 2 \\\\ 0 \end{bmatrix}$$
>
> $$T = \begin{bmatrix} 0 & -1 & 0 & 1 \\\\ 1 & 0 & 0 & 2 \\\\ 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> Point
>
> $$P_{world} = [3, 0, 5]^\top$$
>
> transformed to camera coordinates:
>
> $$P_{cam} = T \begin{bmatrix} 3 \\\\ 0 \\\\ 5 \\\\ 1 \end{bmatrix} = \begin{bmatrix} 0\times3 + (-1)\times0 + 0\times5 + 1\times1 \\\\ 1\times3 + 0\times0 + 0\times5 + 1\times2 \\\\ 0\times3 + 0\times0 + 1\times5 + 0\times1 \\\\ 0+0+0+1 \end{bmatrix} = \begin{bmatrix} 1 \\\\ 5 \\\\ 5 \\\\ 1 \end{bmatrix}$$
>
> i.e.,
>
> $$P_{cam} = [1, 5, 5]^\top$$

**Transformation composition**:

$$T_{ac} = T_{ab} \cdot T_{bc}$$

World point → camera coordinates:

$$P_{cam} = T_{cw} \cdot P_{world} = T_{wc}^{-1} \cdot P_{world}$$

> **Example 15** — Transformation composition
>
> $$T_{ab}$$
>
> : from A to B (rotation
>
> $$90^\circ$$
>
> about Z, translation
>
> $$[1,0,0]$$
>
> )
>
> $$T_{bc}$$
>
> : from B to C (rotation
>
> $$90^\circ$$
>
> about Z, translation
>
> $$[0,1,0]$$
>
> )
>
> $$T_{ab} = \begin{bmatrix} 0 & -1 & 0 & 1 \\\\ 1 & 0 & 0 & 0 \\\\ 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad T_{bc} = \begin{bmatrix} 0 & -1 & 0 & 0 \\\\ 1 & 0 & 0 & 1 \\\\ 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> $$T_{ac} = T_{ab} \cdot T_{bc} = \begin{bmatrix} -1 & 0 & 0 & 1 \\\\ 0 & -1 & 0 & 1 \\\\ 0 & 0 & 1 & 0 \\\\ 0 & 0 & 0 & 1 \end{bmatrix}$$
>
> The composite transformation: rotation about Z by
>
> $$180^\circ$$
>
> (
>
> $$90^\circ+90^\circ$$
>
> ), translation by
>
> $$[1,1,0]^\top$$

#### Three Representations of Rotation

| Representation | Parameters | Pros | Cons |
|----------------|------------|------|------|
| Euler Angles | 3 | Intuitive | Gimbal lock, discontinuities |
| Rotation Vector (Axis-Angle) | 3 | Compact | Singularity (near θ=0) |
| Quaternion | 4 | No singularities, smooth interpolation | Not intuitive |

> **Example 16** — Conversion between the three representations
>
> Rotation about the Z axis by 60°:
>
> $$R_z(60^\circ) = \begin{bmatrix} 0.5 & -0.866 & 0 \\\\ 0.866 & 0.5 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> **Euler Angles** (ZYX):
>
> $$(60^\circ, 0^\circ, 0^\circ)$$
>
> **Rotation Vector** (Axis-Angle):
>
> $$\theta = 60^\circ = \frac{\pi}{3}$$
>
> Axis:
>
> $$\mathbf{n} = [0, 0, 1]^\top$$
>
> Rotation vector:
>
> $$\theta\mathbf{n} = [0, 0, \frac{\pi}{3}]^\top$$
>
> **Quaternion**:
>
> $$q = (\cos\frac{\theta}{2}, \mathbf{n}\sin\frac{\theta}{2}) = (\cos30^\circ, 0, 0, \sin30^\circ) = (0.866, 0, 0, 0.5)$$

**SLAM practice**: ORB-SLAM uses quaternion + translation vector to represent pose; optimization is performed on the se(3) Lie algebra (a 6-dimensional vector).

---

## 2.2 Probability and Statistics — The Language of Uncertainty in SLAM

The essence of SLAM is a probabilistic inference problem:

$$P(\text{map}, \text{pose} \mid \text{observation})$$

> Given sensor observations, what is the maximum **posterior probability** of the map and trajectory?

### 2.2.1 Gaussian Distribution (Normal Distribution)

#### Univariate Gaussian

$$p(x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$$

-

  $$\mu$$

  : mean (the most probable value)

  $$\sigma$$

  : standard deviation (measure of uncertainty)

> **Example 17** — Computing probability density for a univariate Gaussian
>
> A depth sensor measures depth, with noise following
>
> $$\mathcal{N}(\mu=5.0\text{m},\ \sigma=0.1\text{m})$$
>
> Probability density at
>
> $$x=5.05\text{m}$$
>
> $$p(5.05) = \frac{1}{0.1\sqrt{2\pi}} \exp\left(-\frac{(5.05-5.0)^2}{2 \times 0.01}\right) = \frac{1}{0.1\times2.5066} \exp\left(-\frac{0.0025}{0.02}\right)$$
> $$= \frac{1}{0.25066} \times e^{-0.125} \approx 3.989 \times 0.8825 \approx 3.52$$
>
> At
>
> $$x=5.3\text{m}$$
>
> (3σ away):
>
> $$p(5.3) = \frac{1}{0.1\sqrt{2\pi}} \exp\left(-\frac{0.09}{0.02}\right) = 3.989 \times e^{-4.5} \approx 3.989 \times 0.0111 \approx 0.044$$
>
> As we can see, the probability density drops rapidly the further we deviate from the mean.

**Why does SLAM almost exclusively use the Gaussian distribution?**
1. Central Limit Theorem: the sum of a large number of independent errors ≈ Gaussian
2. The Gaussian distribution is closed under marginalization and conditioning (the foundation of the Kalman filter)
3. Taking the negative log turns products into sums of squares → least-squares optimization

**The 68-95-99.7 rule**:
-

  $$[\mu-\sigma, \mu+\sigma]$$

  contains ~68% of the probability
-

  $$[\mu-2\sigma, \mu+2\sigma]$$

  contains ~95% of the probability
-

  $$[\mu-3\sigma, \mu+3\sigma]$$

  contains ~99.7% of the probability

> **Example 18** — The 68-95-99.7 rule applied to SLAM
>
> The X coordinate of a camera pose estimate
>
> $$\sim \mathcal{N}(\mu=2.0\text{m},\ \sigma=0.05\text{m})$$
>
> - 68% probability the true value is within
>
>   $$[1.95,\ 2.05]$$
>
>   m
> - 95% probability the true value is within
>
>   $$[1.90,\ 2.10]$$
>
>   m
> - 99.7% probability the true value is within
>
>   $$[1.85,\ 2.15]$$
>
>   m
>
> This is the 1D version of the **uncertainty ellipsoid** in SLAM. Smaller σ = more precise estimate.

#### Multivariate Gaussian

$$p(\mathbf{x}) = \frac{1}{(2\pi)^{n/2}|\Sigma|^{1/2}} \exp\left(-\frac{1}{2}(\mathbf{x}-\mu)^\top\Sigma^{-1}(\mathbf{x}-\mu)\right)$$

-

  $$\mu$$

  : n-dimensional mean vector

  $$\Sigma$$

  : n×n covariance matrix (describes correlations among dimensions)

In SLAM: the covariance matrix of the camera pose describes your **uncertainty ellipsoid**.

> **Example 19** — Multivariate Gaussian covariance
>
> Covariance matrix of a 2D pose
>
> $$(x, y)$$
>
> $$\Sigma = \begin{bmatrix} 0.04 & 0.01 \\\\ 0.01 & 0.09 \end{bmatrix}$$
>
> Interpretation:
> -
>
> $$\sigma_x^2 = 0.04 \Rightarrow \sigma_x = 0.2\text{m}$$
>
> - (uncertainty in the X direction)
> -
>
> $$\sigma_y^2 = 0.09 \Rightarrow \sigma_y = 0.3\text{m}$$
>
> - (more uncertain in the Y direction)
>   Covariance
>
>   $$0.01 > 0$$
>
>   → X and Y are positively correlated (when X is larger, Y tends to be larger too)
>
> Find the eigenvalues and eigenvectors of
>
> $$\Sigma$$
>
> $$\det\begin{bmatrix} 0.04-\lambda & 0.01 \\\\ 0.01 & 0.09-\lambda \end{bmatrix} = (0.04-\lambda)(0.09-\lambda) - 0.0001 = 0$$
> $$\lambda^2 - 0.13\lambda + 0.0035 = 0 \Rightarrow \lambda_1 \approx 0.093,\ \lambda_2 \approx 0.037$$
>
> Semi-axis lengths of the uncertainty ellipse:
>
> $$\sqrt{\lambda_1} \approx 0.305\text{m}$$
>
> $$\sqrt{\lambda_2} \approx 0.192\text{m}$$
>
> The direction of the ellipse is determined by the corresponding eigenvectors.

**Geometric meaning of the covariance matrix**:
- Diagonal elements: variance of each variable
  Off-diagonal elements: correlation between variables
  Eigenvalues determine the axis lengths of the ellipsoid, eigenvectors determine its orientation

---

### 2.2.2 Bayes' Theorem

$$P(A \mid B) = \frac{P(B \mid A) \cdot P(A)}{P(B)}$$

-

  $$P(A)$$

  : **prior** (belief before seeing any data)

  $$P(B \mid A)$$

  : **likelihood** (probability of observing B given A)

  $$P(A \mid B)$$

  : **posterior** (updated belief about A after seeing B)

> **Example 20** — Bayes' Theorem: SLAM loop closure detection
>
> Suppose:
> - Prior probability of a loop closure
> -
>
> $$P(\text{loop}) = 0.1$$
>
> - (only 10% chance of passing through a previously visited place)
> - If it really is a loop, the probability of successful feature matching
> -
>
> $$P(\text{many matches} \mid \text{loop}) = 0.95$$
>
> - If it is not a loop, the probability of successful feature matching
> -
>
> $$P(\text{many matches} \mid \text{no loop}) = 0.05$$
>
> - (occasional false matches with many correspondences)
>
> Now we observe many matches. What is the posterior probability of a loop closure?
>
> **Step 1**: Compute the total probability
>
> $$P(\text{many matches}) = P(\text{many matches}\mid\text{loop})P(\text{loop}) + P(\text{many matches}\mid\text{no loop})P(\text{no loop})$$
> $$= 0.95 \times 0.1 + 0.05 \times 0.9 = 0.095 + 0.045 = 0.14$$
>
> **Step 2**: Bayesian update
>
> $$P(\text{loop} \mid \text{many matches}) = \frac{0.95 \times 0.1}{0.14} = \frac{0.095}{0.14} \approx 0.679$$
>
> After observing many matches, the loop closure probability updated from 10% to **68%**! But there is still a 32% chance it is not a loop.
>
> ORB-SLAM does exactly this: first use BoW matching to find candidate loop frames, then further confirm through geometric verification (Sim3 alignment).

#### The Bayesian Framework of SLAM

$$\underbrace{P(X_{1:t}, M \mid Z_{1:t}, U_{1:t})}_{\text{posterior: }P(\text{map, trajectory} \mid \text{observations, controls})} \propto \underbrace{P(Z_t \mid X_t, M)}_{\text{observation model}} \cdot \underbrace{P(X_t \mid X_{t-1}, U_t)}_{\text{motion model}}$$

> Every time new sensor data arrives, we use Bayes' theorem to **update** our belief about the state of the world.

---

### 2.2.3 Maximum Likelihood Estimation (MLE) and Maximum A Posteriori (MAP) Estimation

#### MLE (Maximum Likelihood)

$$\hat{\theta}_{\text{MLE}} = \arg\max_\theta P(D \mid \theta)$$

"Which parameter is most likely to have produced the data we observed?"

In SLAM: given matched points, the most probable camera pose → **this is the spirit of PnP!**

> **Example 21** — MLE: estimating the true depth from measurements
>
> Three depth measurements:
>
> $$z = [4.9, 5.1, 5.0]$$
>
> meters, assuming noise
>
> $$\mathcal{N}(0, \sigma^2)$$
>
> Likelihood function:
>
> $$P(z \mid \mu) = \prod_{i=1}^3 \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(z_i - \mu)^2}{2\sigma^2}\right)$$
>
> Taking the negative log (dropping constants):
>
> $$-\log P(z \mid \mu) \propto \sum_{i=1}^3 (z_i - \mu)^2 = (4.9-\mu)^2 + (5.1-\mu)^2 + (5.0-\mu)^2$$
>
> Differentiate and set to zero:
>
> $$\frac{d}{d\mu} = -2(4.9-\mu) - 2(5.1-\mu) - 2(5.0-\mu) = 0$$
> $$3\mu = 4.9 + 5.1 + 5.0 = 15.0 \quad \Rightarrow \quad \hat{\mu}_{\text{MLE}} = 5.0$$
>
> MLE = **sample mean**! (This is a natural consequence of the Gaussian assumption)

#### MAP (Maximum A Posteriori)

$$\hat{\theta}_{\text{MAP}} = \arg\max_\theta P(\theta \mid D) = \arg\max_\theta P(D \mid \theta)P(\theta)$$

Compared to MLE, there is an additional prior

$$P(\theta)$$

> **Example 22** — MAP: depth estimation with a prior
>
> Measurements:
>
> $$z = [5.2, 5.3]$$
>
> . Prior: depth
>
> $$\sim \mathcal{N}(4.5, 1.0^2)$$
>
> (we "believe" it is around 4.5m, but are not very certain).
>
> Likelihood
>
> $$P(z \mid \mu) \propto \exp(-\frac{(5.2-\mu)^2 + (5.3-\mu)^2}{2\sigma^2})$$
>
> Prior
>
> $$P(\mu) \propto \exp(-\frac{(\mu-4.5)^2}{2\times1.0})$$
>
> Negative log posterior (assuming
>
> $$\sigma^2=0.01$$
>
> ):
>
> $$-\log P(\mu \mid z) \propto \frac{(5.2-\mu)^2 + (5.3-\mu)^2}{2\times0.01} + \frac{(\mu-4.5)^2}{2\times1.0}$$
> $$= 50[(5.2-\mu)^2 + (5.3-\mu)^2] + 0.5(\mu-4.5)^2$$
>
> Minimizing:
>
> $$\frac{d}{d\mu} = -100(5.2-\mu) - 100(5.3-\mu) + (\mu-4.5) = 0$$
> $$200\mu - 1050 + \mu - 4.5 = 0 \quad \Rightarrow \quad 201\mu = 1054.5 \quad \Rightarrow \quad \hat{\mu}_{\text{MAP}} \approx 5.25$$
>
> Compare with MLE =
>
> $$(5.2+5.3)/2 = 5.25$$
>
> ... In this example MLE and MAP are the same, because
>
> $$\sigma^2$$
>
> is very small and the prior is wide. If the prior were narrow (high confidence), MAP would be "pulled" toward the prior.
>
> In practice: pure BA = MLE (minimizing reprojection error), BA with a prior = MAP (e.g., fixing the first frame's pose).

#### Why Does Minimizing Squared Error Equal MLE?

Assume the observation noise is Gaussian:

$$z = h(x) + \epsilon,\ \epsilon \sim \mathcal{N}(0, \sigma^2)$$

$$P(z \mid x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(z - h(x))^2}{2\sigma^2}\right)$$

Taking the negative log:

$$-\log P(z \mid x) = \frac{(z - h(x))^2}{2\sigma^2} + \text{const}$$

Therefore:

$$\arg\max_x P(z \mid x) = \arg\min_x (z - h(x))^2$$

> **Example 23** — Reprojection error = MLE
>
> 3D point projected to the image: predicted pixel
>
> $$(u_{\text{pred}}, v_{\text{pred}}) = h(X, R, t, K)$$
>
> Actual observed pixel:
>
> $$(u_{\text{obs}}, v_{\text{obs}}) = (321.5, 240.3)$$
>
> Reprojection error:
>
> $$e = \sqrt{(321.5 - u_{\text{pred}})^2 + (240.3 - v_{\text{pred}})^2}$$
>
> Minimizing
>
> $$\sum e^2$$
>
> = maximizing
>
> $$\prod \exp(-e^2/2\sigma^2)$$
>
> = **maximizing the likelihood under the Gaussian noise assumption**!
>
> **This is the mathematical essence of Bundle Adjustment!** Minimizing reprojection error ≈ maximizing the likelihood under the Gaussian noise assumption.

---

### 2.2.4 Taylor Expansion

$$f(x) \approx f(x_0) + f'(x_0)(x - x_0) + \frac{1}{2}f''(x_0)(x - x_0)^2 + \cdots$$

Key applications in SLAM:
- **First-order Taylor** → linearizing nonlinear functions (the foundation of the Gauss-Newton method)
- **Second-order Taylor** → Hessian matrix (Newton's method)
- **Lie algebra perturbation** → linearizing the pose optimization problem on se(3) using first-order Taylor expansion

> **Example 24** — First-order Taylor expansion: linearizing the projection function
>
> Suppose depth
>
> $$Z=5$$
>
> , the camera is at the origin. Projection of a 3D point's X coordinate:
>
> $$u(X) = f_x \cdot \frac{X}{Z} + c_x = 500 \cdot \frac{X}{5} + 320 = 100X + 320$$
>
> Expand around
>
> $$X_0 = 1.0$$
>
> $$u(X) \approx u(1.0) + u'(1.0)(X - 1.0)$$
> $$u(1.0) = 100\times1.0 + 320 = 420$$
> $$u'(X) = 100 \quad\text{(constant, linear function)}$$
> $$u(X) \approx 420 + 100(X - 1.0) = 100X + 320 \quad\text{(exact, linear function)}$$
>
> ---
>
> **Example 24b** — Taylor expansion of a nonlinear function
>
> $$f(x) = e^x, \quad x_0 = 0$$
>
> $$f(0) = 1, \quad f'(0) = 1, \quad f''(0) = 1$$
>
> First-order:
>
> $$f(x) \approx 1 + x$$
>
> (near
>
> $$x=0$$
>
> )
> Second-order:
>
> $$f(x) \approx 1 + x + \frac{1}{2}x^2$$
>
> At
>
> $$x=0.5$$
>
> : - True value:
>
>   $$e^{0.5} \approx 1.6487$$
>
> - First-order approximation:
>
>   $$1.5$$
>
>   (error 9%)
> - Second-order approximation:
>
>   $$1 + 0.5 + 0.125 = 1.625$$
>
>   (error 1.4%)
>
> This is the difference between **Gauss-Newton vs Newton's method**: Gauss-Newton uses a first-order approximation (saving the cost of computing the Hessian), while Newton's method uses a second-order approximation (more accurate but computationally expensive).

---

## 2.3 Calculus

### 2.3.1 Derivatives and Gradients

- First-order derivative → rate of change
- Gradient

  $$\nabla f$$

  → direction of steepest ascent for a multivariable function
- In SLAM: the **Jacobian matrix J** is the matrix of partial derivatives of a multivariable function with respect to all variables

> **Example 25** — Gradient of a multivariable function
>
> $$f(x, y) = x^2 + 3xy + y^2$$
>
> Partial derivatives:
>
> $$\frac{\partial f}{\partial x} = 2x + 3y$$
>
> $$\frac{\partial f}{\partial y} = 3x + 2y$$
>
> Gradient:
>
> $$\nabla f = [2x + 3y,\ 3x + 2y]^\top$$
>
> At point
>
> $$(x,y) = (1, 2)$$
>
> $$\nabla f(1,2) = [2\times1 + 3\times2,\ 3\times1 + 2\times2]^\top = [8,\ 7]^\top$$
>
> The function increases fastest at
>
> $$(1,2)$$
>
> in the direction of
>
> $$[8,7]^\top$$
>

> **Example 26** — Jacobian matrix (the most important derivative in SLAM)
>
> The reprojection function maps a 3D point
>
> $$[X,Y,Z]^\top$$
>
> to a pixel
>
> $$(u,v)$$
>
> $$u = f_x\frac{X}{Z} + c_x, \quad v = f_y\frac{Y}{Z} + c_y$$
>
> Jacobian matrix (partial derivatives with respect to the 3D point):
>
> $$J = \begin{bmatrix} \frac{\partial u}{\partial X} & \frac{\partial u}{\partial Y} & \frac{\partial u}{\partial Z} \\\\ \frac{\partial v}{\partial X} & \frac{\partial v}{\partial Y} & \frac{\partial v}{\partial Z} \end{bmatrix} = \begin{bmatrix} \frac{f_x}{Z} & 0 & -\frac{f_x X}{Z^2} \\\\ 0 & \frac{f_y}{Z} & -\frac{f_y Y}{Z^2} \end{bmatrix}$$
>
> When
>
> $$f_x=f_y=500$$
>
> $$[X,Y,Z]=[1, 0.5, 5]^\top$$
>
> $$J = \begin{bmatrix} \frac{500}{5} & 0 & -\frac{500\times1}{25} \\\\ 0 & \frac{500}{5} & -\frac{500\times0.5}{25} \end{bmatrix} = \begin{bmatrix} 100 & 0 & -20 \\\\ 0 & 100 & -10 \end{bmatrix}$$
>
> J tells us: changing X (3D) affects u (pixel) by 100 pixels/meter, changing Z affects u by -20 pixels/meter.
> The farther from the camera (larger Z), the smaller J becomes → distant points contribute less to pose optimization.

### 2.3.2 Logarithms and Exponentials

-

  $$\ln(e^x) = x$$
  $$\ln(ab) = \ln a + \ln b$$

In SLAM:
- Taking the logarithm turns products into sums → negative log-likelihood → least squares

  $$\exp$$

  maps se(3) → SE(3) (Lie algebra to Lie group)

  $$\log$$

  maps SE(3) → se(3) (Lie group to Lie algebra)

> **Example 27** — Applying the log transformation in SLAM
>
> Likelihood function (Gaussian noise, multiple observations):
>
> $$L = \prod_{i=1}^n \exp\left(-\frac{(z_i - h(x))^2}{2\sigma^2}\right)$$
>
> Taking the log turns it into a sum:
>
> $$\log L = \sum_{i=1}^n -\frac{(z_i - h(x))^2}{2\sigma^2} = -\frac{1}{2\sigma^2}\sum_{i=1}^n (z_i - h(x))^2$$
>
> Maximizing
>
> $$\log L$$
>
> = minimizing
>
> $$\sum (z_i - h(x))^2$$
>
> — this is the origin of **least squares**.
>
> ---
>
> **Example 27b** — Exponential map (Lie algebra)
>
> Rotation vector
>
> $$\phi = [0, 0, \frac{\pi}{4}]^\top$$
>
> (rotation about the Z axis by 45°)
>
> $$\phi^\wedge = \begin{bmatrix} 0 & -\frac{\pi}{4} & 0 \\\\ \frac{\pi}{4} & 0 & 0 \\\\ 0 & 0 & 0 \end{bmatrix}$$
>
> Exponential map (Rodrigues formula):
>
> $$R = \exp(\phi^\wedge) = I + \frac{\sin\|\phi\|}{\|\phi\|}\phi^\wedge + \frac{1-\cos\|\phi\|}{\|\phi\|^2}(\phi^\wedge)^2$$
>
> $$\|\phi\| = \frac{\pi}{4}, \quad \sin\frac{\pi}{4} = 0.7071, \quad 1-\cos\frac{\pi}{4} = 0.2929$$
> $$R = I + \frac{0.7071}{\pi/4}\phi^\wedge + \cdots = R_z(45^\circ) = \begin{bmatrix} 0.7071 & -0.7071 & 0 \\\\ 0.7071 & 0.7071 & 0 \\\\ 0 & 0 & 1 \end{bmatrix}$$
>
> $$\exp$$
>
> maps a 3-dimensional se(3) vector to a 4×4 SE(3) matrix — this is the core operation for optimizing poses.

---

## 📝 Self-Check Checklist

Before moving on to Level 2, make sure you can answer:

- [ ] What are the two core uses of the cross product in SLAM?
  [ ] After SVD decomposition, how do you solve

  $$Ax=0$$

  ?
  [ ] Why does the inverse of a rotation matrix equal its transpose?
  [ ] What do the eigenvalues of a covariance matrix tell you?
  [ ] Why does minimizing reprojection error = maximum likelihood estimation?
  [ ] What does the multivariate Gaussian distribution represent in SLAM?
  [ ] What are the four parts of the homogeneous transformation matrix T? What is the meaning of each?

---

> **Next step**: Complete `exercises/exercise_01_linear_algebra.py` and `exercises/exercise_02_probability.py` to solidify these mathematical concepts.

