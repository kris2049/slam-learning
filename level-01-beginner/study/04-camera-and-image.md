# Module 4: Camera Device and Image Processing

> Covers everything in "Camera Device" and "Image Data" from visual-slam-roadmap Level 1.
> Each formula is accompanied by a **complete numerical calculation example**.

---

## 4.1 Camera Hardware Basics

### 4.1.1 Lens

| Parameter | Meaning | SLAM Impact |
|------|------|-----------|
| **Focal Length f** | Distance from lens optical center to sensor | Determines field of view (FOV) and $f_x, f_y$ in K matrix |
| **Aperture** | Controls light intake (f/2.8, f/4...) | Affects depth of field and exposure |
| **Field of View FOV** | Visible horizontal/vertical angular range | $\tan(\text{FOV}/2) = (\text{sensor width} / 2) / f$ |
| **Distortion** | Actual projection deviates from ideal model | Requires calibration correction |

> **Example 1** — FOV Calculation
>
> Sensor width = 6.4mm, focal length f = 8mm
>
> $$\frac{\text{FOV}}{2} = \arctan\left(\frac{6.4/2}{8}\right) = \arctan\left(\frac{3.2}{8}\right) = \arctan(0.4) \approx 21.8^\circ$$
>
> $$\text{FOV}_{\text{horizontal}} \approx 43.6^\circ$$
>
> Calculate vertical FOV the same way (sensor height 4.8mm):
> $$\text{FOV}_{\text{vertical}} = 2 \times \arctan\left(\frac{2.4}{8}\right) \approx 33.4^\circ$$
>
> ---
>
> **Example 1b** — Back-calculating Focal Length from FOV
>
> Given horizontal FOV = 90°, sensor width = 6.4mm
> $$\tan(45^\circ) = \frac{3.2}{f} = 1 \quad \Rightarrow \quad f = 3.2\text{mm}$$
>
> Wide-angle lens! This is why many AR glasses use short focal lengths.

**Lens Types and SLAM Selection**:

| Type | Focal Length | FOV | Typical Use |
|------|------|-----|----------|
| Wide-angle | < 24mm | > 80° | Indoor SLAM (AR glasses) |
| Standard | 35-50mm | ~50° | General purpose |
| Telephoto | > 85mm | < 30° | Long-range SLAM |

**Impact of Wide-angle Lenses on SLAM**:
- ✅ Larger FOV → features stay in view across frames longer → better tracking
- ✅ Can see enough scene even at close range
- ❌ Severe distortion → must be corrected
- ❌ Larger angular span per pixel → reduced angular resolution

**MTF (Modulation Transfer Function)**:
Lens optical quality metric. High MTF → sharper images → better feature extraction.

---

### 4.1.2 Image Sensor

**CCD vs CMOS**:

| | CCD | CMOS |
|---|-----|------|
| Noise | Low | Higher |
| Speed | Slow | Fast |
| Power | High | Low |
| Cost | High | Low |
| Current mainstream | — | ✅ Almost all modern SLAM cameras |

**Global Shutter vs Rolling Shutter**

| | Global Shutter | Rolling Shutter |
|---|----------|----------|
| Principle | All pixels exposed simultaneously | Row-by-row exposure |
| Motion Blur | None | **Yes! (SLAM's enemy)** |
| Cost | High | Low |
| Mainstream Devices | Intel RealSense D435 | Most phones/webcams |

> **Example 2** — Quantifying Rolling Shutter Effect
>
> Resolution 1920×1080, 30fps, readout time ~33ms
>
> Time difference between rows:
> $$33\text{ms} / 1080 \approx 30.6 \mu\text{s}$$
>
> If the camera rotates at 0.5 rad/s (≈ 30°/s):
> Angular change from row 1 to row 1080:
> $$0.5 \times 0.033 = 0.0165\text{ rad} \approx 0.95^\circ$$
>
> Pixel shift (f=800):
> $$800 \times 0.0165 \approx 13\text{ px}$$
>
> 13 pixels of shift! The projection model
> $$P = K[R|t]P_w$$
> assumes the entire image was captured at the same instant → severe violation of assumptions → pose drift.

**Impact of Rolling Shutter on SLAM**:
During rapid motion, each row of the image corresponds to a different camera pose → projection model fails → pose estimation drifts.

**Mitigation**: Systems like DSO model and correct for rolling shutter.

---

### 4.1.3 Exposure and ISO

| Parameter | Controls | SLAM Impact |
|------|------|-----------|
| **Exposure Time** | Duration shutter is open | Long exposure → motion blur → tracking failure |
| **ISO** | Sensor gain | High ISO → increased noise → degraded feature quality |
| **Aperture** | Light intake aperture | Large aperture → shallow depth of field → blurred distant objects |

> **Example 3** — Motion Blur Quantification
>
> Camera translation speed 0.2 m/s, exposure time 16ms (1/60s)
>
> Camera displacement during exposure:
> $$0.2 \times 0.016 = 0.0032\text{m} = 3.2\text{mm}$$
>
> Scene distance 2m, focal length 800px (corresponding to ~8mm physical focal length, sensor 6.4mm)
> Blur on sensor:
> $$\frac{8\text{mm}}{2000\text{mm}} \times 3.2\text{mm} = 0.0128\text{mm}$$
>
> Pixel blur:
> $$\frac{0.0128\text{mm}}{6.4\text{mm}} \times 1280 \approx 2.6\text{ px}$$
>
> 2-3 pixels of blur is enough to make corner detection fail!

**Fatal Impact of Auto Exposure (AE) on Direct Methods**:
Direct methods rely on the **photometric constancy assumption** (same object has consistent brightness across frames).
AE changes the overall brightness → photometric constancy assumption is broken → pose estimation fails.

**DSO's Solution**: Photometric Calibration
- Pre-calibrate the camera's response function (nonlinear mapping) and vignetting (edge darkening)
- Compensate for these factors in the photometric error computation

---

### 4.1.4 Resolution

| Resolution | Pixel Count | SLAM Impact |
|--------|--------|-----------|
| VGA | 640×480 | Low computation, suitable for getting started |
| HD | 1280×720 | Common sweet spot |
| Full HD | 1920×1080 | Higher accuracy, more features |
| 4K | 3840×2160 | Extremely high computation |

> **Example 4** — Relationship Between Resolution and Computation
>
> VGA (640×480): 307,200 pixels → ORB feature extraction ~2ms
> HD (1280×720): 921,600 pixels → ~6ms (×3 pixels, linear)
> FHD (1920×1080): 2,073,600 pixels → ~14ms (×6.75 pixels)
>
> Within the 30fps (33ms/frame) budget, FHD uses 14ms just on feature extraction, leaving only 19ms for tracking and mapping. This is why most SLAM systems run at VGA-HD.

**Resolution vs. Accuracy Trade-off**:
- More pixels → richer features → more accurate matching
- But computation O(pixel count) → doubling resolution = ×4 processing time
- In practice: most SLAM systems run between VGA and HD

---

## 4.2 Image Processing Fundamentals

### 4.2.1 Color and Grayscale

**RGB → Grayscale** (weighted average, simulating human perception):
$$\text{Gray} = 0.299R + 0.587G + 0.114B$$

Why is G weighted highest? Because human eyes are most sensitive to green.

> **Example 5** — Pixel Grayscale Value Calculation
>
> | Pixel | R | G | B | Grayscale | Note |
> |------|---|---|---|------|------|
> | Red object | 255 | 0 | 0 |
> $$0.299\times255 = 76$$
> | Human eye sees red as relatively dark |
> | Green object | 0 | 255 | 0 |
> $$0.587\times255 = 150$$
> | Human eye sees green as brightest |
> | Blue object | 0 | 0 | 255 |
> $$0.114\times255 = 29$$
> | Human eye sees blue as very dark |
> | White | 255 | 255 | 255 |
> $$0.299\times255 + 0.587\times255 + 0.114\times255 = 255$$
> | |
> | Gray | 128 | 128 | 128 | 128 | |
>
> This is why the green channel carries the most luminance information.

**Why does SLAM typically use only grayscale images?**
1. Feature extraction does not depend on color
2. Single channel = 3× memory and computation savings
3. Grayscale is more robust to lighting changes

But there are exceptions: **Semantic SLAM** uses RGB for object recognition.

---

### 4.2.2 Gaussian Blur

Used for **denoising** and **building image pyramids**.

**Gaussian Kernel** (3×3 example, $\sigma=1.0$):
$$G(x,y) = \frac{1}{2\pi\sigma^2} e^{-\frac{x^2+y^2}{2\sigma^2}}$$

Discrete approximation:
$$\frac{1}{16}\begin{bmatrix} 1 & 2 & 1 \\ 2 & 4 & 2 \\ 1 & 2 & 1 \end{bmatrix}$$

> **Example 6** — Hand-computing Gaussian Kernel and Convolution
>
> Generate 3×3 kernel with
> $$\sigma=1.0$$
> :
>
> $$G(-1,-1) = \frac{1}{2\pi} e^{-2/2} = \frac{e^{-1}}{2\pi} \approx 0.159 \times 0.3679 \approx 0.0585$$
> $$G(0,-1) = \frac{1}{2\pi} e^{-1/2} = \frac{e^{-0.5}}{2\pi} \approx 0.159 \times 0.6065 \approx 0.0965$$
> $$G(0,0) = \frac{1}{2\pi} e^{0} = \frac{1}{2\pi} \approx 0.159$$
>
> Normalize (divide by sum
> $$0.0585\times4 + 0.0965\times4 + 0.159 = 0.234+0.386+0.159=0.779$$
> ):
> $$\text{kernel} \approx \frac{1}{16}\begin{bmatrix} 1 & 2 & 1 \\ 2 & 4 & 2 \\ 1 & 2 & 1 \end{bmatrix}$$
>
> ---
>
> **Example 6b** — Convolution Operation
>
> 5×5 image:
> $$I = \begin{bmatrix} 10 & 20 & 30 & 20 & 10 \\ 20 & 40 & 50 & 40 & 20 \\ 30 & 50 & 80 & 50 & 30 \\ 20 & 40 & 50 & 40 & 20 \\ 10 & 20 & 30 & 20 & 10 \end{bmatrix}$$
>
> Convolve the center pixel (row 3, col 3, value=80):
> $$I[2,2]_{\text{new}} = \frac{1}{16}\sum_{i=-1}^{1}\sum_{j=-1}^{1} w_{ij} \cdot I[2+i, 2+j]$$
>
> $$= \frac{1}{16}(1\times40 + 2\times50 + 1\times40 + 2\times50 + 4\times80 + 2\times50 + 1\times40 + 2\times50 + 1\times40)$$
> $$= \frac{1}{16}(40+100+40+100+320+100+40+100+40) = \frac{880}{16} = 55$$
>
> After blurring 80 → 55 (moving toward the mean of neighboring pixels), this is the **smoothing/denoising** effect.

**Image Pyramid** (the most essential SLAM preprocessing):

```text
Level 0: Original image (640×480)
Level 1: Downsampled + Gaussian blur (320×240)
Level 2: Further downsampled (160×120)
Level 3: Further downsampled (80×60)
```

**Why is a pyramid needed?**
- ORB-SLAM: extracts ORB features at each pyramid level → achieves **scale invariance**
- DSO: coarse-to-fine optimization → avoids local minima
- KLT optical flow: pyramid accelerates convergence

---

### 4.2.3 Thresholding

Convert grayscale image to binary:
$$B(x,y) = \begin{cases} 1 & I(x,y) > T \\ 0 & \text{otherwise} \end{cases}$$

> **Example 7** — Thresholding
>
> Thresholding of a 5×5 grayscale image (
> $$T=128$$
> ):
>
> $$I = \begin{bmatrix} 50 & 200 & 150 & 80 & 255 \\ 30 & 180 & 90 & 120 & 100 \\ 200 & 250 & 130 & 60 & 40 \\ 100 & 90 & 70 & 180 & 200 \\ 0 & 255 & 160 & 140 & 110 \end{bmatrix}$$
>
> $$B = \begin{bmatrix} 0 & 1 & 1 & 0 & 1 \\ 0 & 1 & 0 & 0 & 0 \\ 1 & 1 & 1 & 0 & 0 \\ 0 & 0 & 0 & 1 & 1 \\ 0 & 1 & 1 & 1 & 0 \end{bmatrix}$$
>
> This produces a binary mask that can be used for:
> - Scene segmentation (detecting floor/walls)
> - Occupancy grid mapping
> - ORB-SLAM uses brightness threshold to accelerate FAST corner detection

**Uses in SLAM**:
- Scene segmentation (detecting floor/walls)
- Occupancy grid mapping
- ORB-SLAM uses brightness threshold to accelerate FAST corner detection

---

### 4.2.4 Edge Detection

**Edge = locations where image intensity changes sharply**

**Sobel Operator** (first-order derivative approximation):

$$G_x = \begin{bmatrix} -1 & 0 & 1 \\ -2 & 0 & 2 \\ -1 & 0 & 1 \end{bmatrix} \quad G_y = \begin{bmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{bmatrix}$$

Gradient magnitude:
$$M = \sqrt{G_x^2 + G_y^2}$$
Gradient direction:
$$\theta = \arctan(G_y / G_x)$$

> **Example 8** — Sobel Gradient Calculation (by hand)
>
> 5×5 image (edge in the middle):
> $$I = \begin{bmatrix} 10 & 10 & 10 & 200 & 200 \\ 10 & 10 & 10 & 200 & 200 \\ 10 & 10 & 10 & 200 & 200 \\ 10 & 10 & 10 & 200 & 200 \\ 10 & 10 & 10 & 200 & 200 \end{bmatrix}$$
>
> Compute $G_x$ for center pixel (row 3, col 3, value=10):
>
> $$G_x = (-1)\times10 + 0\times10 + 1\times200 + (-2)\times10 + 0\times10 + 2\times200 + (-1)\times10 + 0\times10 + 1\times200$$
> $$= -10 + 0 + 200 - 20 + 0 + 400 - 10 + 0 + 200 = 760$$
>
> $G_y$:
> $$(-1)\times10 + (-2)\times10 + (-1)\times10 + 0 + 0 + 0 + 1\times200 + 2\times200 + 1\times200$$
> $$= -10 - 20 - 10 + 0 + 200 + 400 + 200 = 760$$
>
> Gradient magnitude:
> $$M = \sqrt{760^2 + 760^2} = 760\sqrt{2} \approx 1075$$
> — Very large! This is a vertical edge.
>
> Gradient direction:
> $$\theta = \arctan(760/760) = 45^\circ$$
> (diagonal direction, because there are changes in both X and Y)
>
> ---
>
> **Example 8b** — Flat Region
>
> For the top-left pixel (row 1, col 1, value=10), neighborhood all 10:
> $$G_x = 0, \quad G_y = 0, \quad M = 0$$
>
> Flat region has no gradient → not an edge.

**Canny Edge Detection** (multi-stage):

Step 1: Gaussian smoothing → denoise
Step 2: Sobel operator to compute gradient magnitude and direction
Step 3: **Non-maximum suppression** → keep only local maxima along gradient direction
Step 4: **Double thresholding + hysteresis** → strong edges connect to weak edges

> **Example 9** — Non-maximum Suppression Illustration
>
> Suppose the gradient magnitude along a horizontal edge (scanning vertically):
> $$[10, 25, 50, 80, 60, 30, 15]$$
>
> Gradient direction = 90° (vertical), so find local maximum along vertical direction:
> 80 is the local maximum (50 left, 60 right) → keep
> Others → suppress to 0
>
> Result:
> $$[0, 0, 0, 80, 0, 0, 0]$$
>
> This step thins "thick" edges down to single-pixel-width "thin" edges.

**Why are edges important for SLAM?**
- LSD-SLAM and DSO use **high-gradient pixels** (edge regions) for tracking
- Edges carry more information than flat regions
- Direct methods leverage photometric gradients at edges for optimization

---

### 4.2.5 Corner Detection

**Corner = point with large gradient changes in two directions**

**Harris Corner Detection** (1988, classic algorithm):

1. Compute image gradients $I_x, I_y$
2. Construct the Structure Tensor:
   $$M = \sum_{window} w(x,y) \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}$$
3. Compute the Harris response:
   $$R = \det(M) - k \cdot \text{trace}(M)^2$$
   where
   $$k \approx 0.04$$
   (empirical value)

4. Criteria:
   - $R > 0$ and large → **corner**
   - $R < 0$ → **edge**
   - $|R|$ small → **flat region**

> **Example 10** — Harris Corner Hand Calculation (3×3 window)
>
> 3×3 window gradients:
> $$I_x = \begin{bmatrix} 5 & 8 & 5 \\ 6 & 10 & 6 \\ 5 & 8 & 5 \end{bmatrix}, \quad I_y = \begin{bmatrix} 5 & 6 & 5 \\ 8 & 10 & 8 \\ 5 & 6 & 5 \end{bmatrix}$$
>
> (This is a corner! Large gradients in both X and Y)
>
> **Step 1**: Compute M elements
> $$\sum I_x^2 = 5^2+8^2+5^2+6^2+10^2+6^2+5^2+8^2+5^2 = 25+64+25+36+100+36+25+64+25 = 400$$
> $$\sum I_y^2 = 5^2+6^2+5^2+8^2+10^2+8^2+5^2+6^2+5^2 = 25+36+25+64+100+64+25+36+25 = 400$$
> $$\sum I_x I_y = 5\times5+8\times6+5\times5 + 6\times8+10\times10+6\times8 + 5\times5+8\times6+5\times5$$
> $$= 25+48+25+48+100+48+25+48+25 = 392$$
>
> **Step 2**: Construct M
> $$M = \begin{bmatrix} 400 & 392 \\ 392 & 400 \end{bmatrix}$$
>
> **Step 3**: Compute Harris response
> $$\det(M) = 400\times400 - 392\times392 = 160000 - 153664 = 6336$$
> $$\text{trace}(M) = 400 + 400 = 800$$
> $$R = 6336 - 0.04 \times 800^2 = 6336 - 0.04\times640000 = 6336 - 25600 = -19264$$
>
> $R < 0$ → **edge**! Although there are gradients in both directions,
> $I_x$ and $I_y$ are highly correlated (nearly proportional), so it's actually an edge.
>
> ---
>
> **Example 10b** — A True Corner
>
> $$I_x = \begin{bmatrix} 5 & 0 & 5 \\ 0 & 10 & 0 \\ 5 & 0 & 5 \end{bmatrix}, \quad I_y = \begin{bmatrix} 0 & 5 & 0 \\ 5 & 10 & 5 \\ 0 & 5 & 0 \end{bmatrix}$$
>
> $$\sum I_x^2 = 25+0+25+0+100+0+25+0+25 = 200$$
> $$\sum I_y^2 = 0+25+0+25+100+25+0+25+0 = 200$$
> $$\sum I_x I_y = 0+0+0+0+0+0+0+0+0 = 0$$
>
> $$M = \begin{bmatrix} 200 & 0 \\ 0 & 200 \end{bmatrix}$$
> $$\det(M) = 40000, \quad \text{trace}(M) = 400$$
> $$R = 40000 - 0.04\times160000 = 40000 - 6400 = 33600$$
>
> $R = 33600$ large positive → **corner**!
> $I_x$ and $I_y$ are uncorrelated ($I_x I_y = 0$), both directions independent.

**What do the two eigenvalues of the M matrix tell us?**

- $\lambda_1, \lambda_2$ both large → corner
- $\lambda_1 \gg \lambda_2$ → vertical edge
- $\lambda_2 \gg \lambda_1$ → horizontal edge
- Both small → flat

**Harris limitations** → superseded by **FAST** and **SIFT** (see Level 2).

---

### 4.2.6 Stereo Vision and Depth

**Binocular Vision Principle**:

Two cameras placed horizontally, baseline distance $B$

Horizontal pixel difference of the same 3D point in left and right images = **disparity**
$$d = u_L - u_R$$

$$Z = \frac{f \cdot B}{d}$$

**Key Relationships**:
- Disparity $d$ is **inversely proportional** to depth $Z$
- Larger baseline $B$ → larger maximum measurable distance
- Longer focal length $f$ → higher depth resolution

> **Example 11** — Binocular Depth Calculation
>
> Camera parameters:
> $$f_x = 800$$
> , baseline
> $$B = 0.12\text{m}$$
> (12cm, similar to human eyes)
>
> Disparity for a 3D point
> $$P = [0, 0, 3]^\top$$
> (3 meters straight ahead):
> $$d = \frac{f \cdot B}{Z} = \frac{800 \times 0.12}{3} = \frac{96}{3} = 32\text{ px}$$
>
> Left image pixel
> $$u_L = 320$$
> (principal point), right image pixel
> $$u_R = 320 - 32 = 288$$
>
> Verify:
> $$Z = \frac{800 \times 0.12}{32} = \frac{96}{32} = 3\text{ m}$$
> ✓
>
> ---
>
> **Example 11b** — Inverse Relationship Between Disparity and Depth
>
> | Depth Z (m) | Disparity d (px) | Relationship |
> |-----------|------------|------|
> | 1 | 96 |
> $$d \propto 1/Z$$
> |
> | 2 | 48 | |
> | 3 | 32 | |
> | 6 | 16 | |
> | 12 | 8 | |
> | ∞ | 0 | Point at infinity has zero disparity |
>
> As can be seen, from 1m to 2m disparity changes by 48px (easy to distinguish), but from 6m to 12m disparity only changes by 8px (hard to distinguish). This is the fundamental reason why **distant depth estimation is inaccurate**.

> **Example 11c** — Impact of Sub-pixel Accuracy
>
> Assuming disparity estimation accuracy of 0.5px:
>
> At Z=3m: disparity 32px, uncertainty
> $$\Delta Z = \frac{Z^2}{fB}\Delta d = \frac{9}{96}\times0.5 \approx 0.047\text{m}$$
> (4.7cm) — acceptable
>
> At Z=10m: disparity 9.6px, uncertainty
> $$\Delta Z = \frac{100}{96}\times0.5 \approx 0.52\text{m}$$
> (52cm) — very poor!
>
> This is the fundamental limitation of binocular vision: **uncertainty grows with the square of depth**.

**RGB-D Cameras**:
Instead of relying on binocular disparity, they actively project patterns:
- **Structured Light**: project a known pattern → deformation → depth
- **ToF (Time of Flight)**: emit infrared pulses → measure round-trip time → depth
- **Active Stereo**: project infrared texture + binocular matching (e.g., RealSense D435)

**RGB-D Depth Noise Model** (from the KinectFusion paper):

$$\sigma_z = \frac{Z^2}{f B} \sigma_d$$

Depth noise is proportional to the square of depth → **distant points are very unreliable**.

> **Example 12** — RGB-D Depth Noise
>
> Kinect v1 parameters:
> $$f=580$$,
>
> $$B=0.075\text{m}$$,
>
> $$\sigma_d = 0.5\text{px}$$
>
> | Depth Z (m) | Noise
> $$\sigma_z$$
> (m) | Measurement
> $$Z \pm 2\sigma$$
> (95% CI) |
> |-----------|-------------------|------|
> | 1 |
> $$\frac{1}{43.5}\times0.5 = 0.011$$
> |
> $$1.0 \pm 0.022$$
> m |
> | 3 |
> $$\frac{9}{43.5}\times0.5 = 0.103$$
> |
> $$3.0 \pm 0.207$$
> m |
> | 5 |
> $$\frac{25}{43.5}\times0.5 = 0.287$$
> |
> $$5.0 \pm 0.574$$
> m |
>
> At 5 meters, the uncertainty is already ~0.6m! This is why the effective range of RGB-D SLAM is typically < 5m.

---

## 📝 Self-Check Checklist

- [ ] Why does direct method SLAM need photometric calibration?
- [ ] Why does rolling shutter affect SLAM? Which systems model it?
- [ ] What are the different uses of image pyramids in ORB-SLAM vs DSO?
- [ ] What is the intuition behind the Harris corner criterion
  $$R = \det(M) - k\cdot\text{trace}(M)^2$$
  ?
- [ ] In the binocular disparity formula
  $$Z = fB/d$$
  , why does a larger baseline B give more accurate depth?
- [ ] What are the two key steps Canny adds beyond Sobel?

---

> **Next step**: Complete `exercises/exercise_04_image_processing.py` to implement these image processing algorithms hands-on.
