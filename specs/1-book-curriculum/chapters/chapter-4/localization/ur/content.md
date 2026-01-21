# Chapter 4: State Estimation (ریاست کا تخمینہ)

**Bilingual Edition | English | اردو**

---

## Learning Objectives (تعلیمی مقاصد)

| English | اردو |
|---------|------|
| 1. **Implement Kalman filters** for linear Gaussian state estimation (position/velocity tracking) | 1. لینیر گاسین ریاست کے تخمینے کے لیے **کالمن فلٹر** لاگو کریں (موقع/رفتار ٹریکنگ) |
| 2. **Apply Extended Kalman Filter (EKF)** to nonlinear systems (pendulum, mobile robot) | 2. غیر لینیر سسٹمز (پینڈلم، موبائل روبوٹ) پر **ایکسٹینڈڈ کالمن فلٹر (EKF)** لاگو کریں |
| 3. **Use particle filters** for non-Gaussian, multi-modal distributions | 3. غیر گاسین، ملٹی موڈل تقسیم کے لیے **پارٹیکل فلٹر** استعمال کریں |
| 4. **Fuse IMU and odometry data** for robust mobile robot localization | 4. مضبوط موبائل روبوٹ لوکلائزیشن کے لیے **IMU اور اوڈومیٹری ڈیٹا فیوژن** کریں |
| 5. **Evaluate estimation accuracy** using RMSE and computational cost metrics | 5. RMSE اور کمپیوٹیشنل لاگت میٹرکس استعمال کر کے **تخمینہ کی درستگی** جانچیں |

---

## 4.1: Bayesian Filtering Framework (بیزین فلٹرنگ فریم ورک)

### State estimation problem (ریاست کا تخمینہ کا مسئلہ)

| English | اردو |
|---------|------|
| Estimate hidden state x from noisy observations z | غیر واضح ریاست x کا اندازہ نویز والے مشاہدات z سے لگائیں |

### Bayes' rule (بیز کا اصول)

| English | اردو |
|---------|------|
| P(x|z) ∝ P(z|x) × P(x) | P(x\|z) ∝ P(z\|x) × P(x) (مشاہدات کی بنیاد پر ریاست کی احتمال تقسیم) |

### Recursive estimation (ریکرسیو تخمینہ)

| English | اردو |
|---------|------|
| Prediction step + Update step | پیڈکشن اسٹیپ + اپڈیٹ اسٹیپ |

### Gaussian assumption (گاسین فرض)

| English | اردو |
|---------|------|
| State and measurement noise ~ N(0, Σ) | ریاست اور پیمائش کا شور ~ N(0, Σ) (صفر اوسط اور کوویریئنس Σ والی گاسین تقسیم) |

---

## 4.2: Kalman Filter (KF) (کالمن فلٹر)

### Linear system model (لینیر سسٹم ماڈل)

```
x_k = A x_{k-1} + B u_k + w_k
z_k = H x_k + v_k
```

| English | اردو |
|---------|------|
| State transition: current state depends on previous state, control input, and process noise | ریاست ٹرانزیشن: موجودہ ریاست پچھلی ریاست، کنٹرول ان پٹ، اور پراسس نویز پر منحصر ہے |
| Measurement model: observation relates to state with measurement noise | ماڈل پیمائش: مشاہدہ ریاست سے پیمائش کے شور کے ساتھ جڑا ہوا ہے |

### Prediction (پیڈکشن)

```
x̂⁻ = A x̂
P⁻ = A P Aᵀ + Q
```

| English | اردو |
|---------|------|
| Predict next state using transition matrix | ٹرانزیشن میٹرکس استعمال کر کے اگلی ریاست کا پیڈکشن کریں |
| Propagate uncertainty through model | ماڈل کے ذریعے عدم یقین کو پھیلائیں |

### Update (اپڈیٹ)

```
K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹
x̂ = x̂⁻ + K(z - H x̂⁻)
P = (I - K H) P⁻
```

| English | اردو |
|---------|------|
| Kalman gain K balances prediction vs measurement | کالمن گین K پیڈکشن اور پیمائش کے درمیان توازن برقرار رکھتا ہے |
| Innovation: z - H x̂⁻ measures discrepancy | انوویشن: z - H x̂⁻ عدم مماثلت ناپتا ہے |
| Updated covariance reflects reduced uncertainty | اپڈیٹ شدہ کوویریئس کم ہوا عدم یقین ظاہر کرتا ہے |

### Example (مثال): 1D position tracking with noisy GPS

| English | اردو |
|---------|------|
| Track a robot moving along a straight line using noisy GPS measurements | GPS کی نویز والی پیمائشوں کا استعمال کرتے ہوئے سیدھی لکیر پر چلتے ہوئے روبوٹ کو ٹریک کریں |

---

## 4.3: Extended Kalman Filter (EKF) (ایکسٹینڈڈ کالمن فلٹر)

### Nonlinear system (غیر لینیر سسٹم)

```
x_k = f(x_{k-1}, u_k) + w_k
```

### Linearization (لینیرایزیشن)

| English | اردو |
|---------|------|
| Jacobian F = ∂f/∂x | جیکوبین F = ∂f/∂x (非线性函数的线性近似) |

### Prediction (پیڈکشن)

```
x̂⁻ = f(x̂, u)
P⁻ = F P Fᵀ + Q
```

| English | اردو |
|---------|------|
| Same as KF but using nonlinear function f | KF جیسا ہی لیکن غیر لینیر فنکشن f استعمال کرتے ہوئے |

### Update (اپڈیٹ)

| English | اردو |
|---------|------|
| Same as KF but with linearized H | KF جیسا ہی لیکن لینیرایزڈ H کے ساتھ |

### Examples (مثالیں)

| English | اردو |
|---------|------|
| Inverted pendulum: nonlinear dynamics from gravity | الٹا پینڈل: کشش ثقل سے غیر لینیر ڈائنامکس |
| Mobile robot with heading angle: nonlinear motion model | ہیڈنگ اینگل کے ساتھ موبائل روبوٹ: غیر لینیر موشن ماڈل |

---

## 4.4: Unscented Kalman Filter (UKF) (انسنٹڈ کالمن فلٹر)

### Sigma points (سگما پوائنٹس)

| English | اردو |
|---------|------|
| Deterministic sampling around mean | اوسط کے گرد ڈیٹرمنسٹک سیمپلنگ |

### Unscented transform (انسنٹڈ ٹرانسفارم)

| English | اردو |
|---------|------|
| Propagate sigma points through nonlinear function | سگما پوائنٹس کو غیر لینیر فنکشن کے ذریعے پھیلائیں |

### Advantage (فائدہ)

| English | اردو |
|---------|------|
| Better accuracy than EKF for highly nonlinear systems | انتہائی غیر لینیر سسٹمز کے لیے EKF سے بہتر درستگی |

### Computational cost (کمپیوٹیشنل لاگت)

| English | اردو |
|---------|------|
| O(n³) vs EKF's O(n²) | O(n³) جبکہ EKF کی O(n²) |

---

## 4.5: Particle Filters (Sequential Monte Carlo) (پارٹیکل فلٹر)

### Non-Gaussian distributions (غیر گاسین تقسیم)

| English | اردو |
|---------|------|
| Multi-modal, arbitrary shapes | ملٹی موڈل، جیسی بھی شکل |

### Particle representation (پارٹیکل نمائندگی)

```
{(x_i, w_i)} where Σw_i = 1
```

| English | اردو |
|---------|------|
| Each particle represents a possible state with weight | ہر پارٹیکل ایک ممکنہ ریاست کا وزن کے ساتھ نمائندہ ہے |

### Prediction (پیڈکشن)

| English | اردو |
|---------|------|
| Propagate particles through motion model | موشن ماڈل کے ذریعے پارٹیکلز کو پھیلائیں |

### Update (اپڈیٹ)

| English | اردو |
|---------|------|
| Reweight particles based on likelihood | لائکلی ہوڈ کی بنیاد پر پارٹیکلز کو دوبارہ وزن دیں |

### Resampling (ریسیمپلنگ)

| English | اردو |
|---------|------|
| Eliminate low-weight particles | کم وزن والے پارٹیکلز کو ختم کریں |

### Example (مثال): Robot localization with ambiguous landmarks

| English | اردو |
|---------|------|
| Track robot position when landmarks look similar | جب لینڈمارکس ایک جیسے نظر آئیں تو روبوٹ کی پوزیشن ٹریک کریں |

---

## 4.6: Sensor Fusion Architectures (سینسر فیوژن آرکیٹیکچر)

### Centralized (سینٹرلائزڈ)

| English | اردو |
|---------|------|
| Single filter with all sensor inputs | ایک فلٹر میں تمام سینسر ان پٹس |

### Decentralized (ڈیسینٹرلائزڈ)

| English | اردو |
|---------|------|
| Multiple filters, fuse estimates | متعدد فلٹر، تخمینوں کو فیوڈ کریں |

### ROS 2 robot_localization (ROS 2 روبوٹ لوکلائزیشن)

| English | اردو |
|---------|------|
| EKF node for IMU + odometry + GPS | IMU + odometry + GPS کے لیے EKF نوڈ |

---

## 4.7: SLAM Basics (سلم کی بنیادیں) (Preview)

### Simultaneous Localization and Mapping (ایک ساتھ لوکلائزیشن اور میپنگ)

| English | اردو |
|---------|------|
| Estimate robot pose AND map | روبوٹ کی پوز اور میپ دونوں کا اندازہ لگائیں |

### EKF-SLAM (EKF-سلم)

| English | اردو |
|---------|------|
| Augmented state [robot_pose, landmark_1, ..., landmark_n] | بڑھا ہوا ریاست [robot_pose, landmark_1, ..., landmark_n] |

### Particle Filter SLAM (FastSLAM) (پارٹیکل فلٹر سلم)

| English | اردو |
|---------|------|
| Particle for pose, EKF for landmarks | پوز کے لیے پارٹیکل، لینڈمارکس کے لیے EKF |

---

## Labs (لیبز)

### Lab 4.1: 1D Kalman Filter (30 min, Beginner)

| English | اردو |
|---------|------|
| Implement KF for position tracking | پوزیشن ٹریکنگ کے لیے KF لاگو کریں |
| Simulate noisy position measurements | نویز والی پوزیشن کی پیمائشیں سِمولیٹ کریں |
| Plot true vs estimated position, covariance over time | اصل بمقابلہ تخمینہ شدہ پوزیشن، وقت کے ساتھ کوویریئس پلاٹ کریں |
| Vary process/measurement noise, observe filter behavior | پراسس/پیمائش کے نویز میں تغیر، فلٹر کے برتاؤ کا مشاہدہ کریں |

### Lab 4.2: EKF for Nonlinear Pendulum (45 min, Intermediate)

| English | اردو |
|---------|------|
| Model: θ̈ = -g/L sin(θ) + u | ماڈل: θ̈ = -g/L sin(θ) + u |
| Measurements: Noisy angle from encoder | پیمائشیں: انکوڈر سے نویز والا زاویہ |
| Implement EKF with Jacobian computation | Jacobian کمپیوٹیشن کے ساتھ EKF لاگو کریں |
| Compare EKF vs true state | EKF بمقابلہ اصل ریاست کا موازنہ کریں |

### Lab 4.3: IMU + Odometry Fusion (60 min, Advanced)

| English | اردو |
|---------|------|
| ROS 2 robot_localization package | ROS 2 robot_localization پیکیج |
| Subscribe to `/imu/data` and `/odom` topics | `/imu/data` اور `/odom` ٹاپکس کو سبسکرائب کریں |
| Configure EKF parameters (process noise, sensor covariances) | EKF پیرامیٹرز کانفیگر کریں (پراسس نویز، سینسر کوویریئسز) |
| Compare fused estimate vs ground truth (motion capture) | فیوڈ شدہ تخمینہ بمقابلہ گراؤنڈ ٹروتی (موشن کیپچر) |

---

## Assessments (امتحانات)

### Multiple Choice Questions (سوالات انتخاب کریں)

**MC 1:** What does the PREDICT step compute in the Bayes filter algorithm?

| English | اردو |
|---------|------|
| A. The posterior probability distribution given the new observation | A. نئے مشاہدے کی بنیاد پر پوسٹیریر احتمال تقسیم |
| B. The prior probability distribution by propagating the previous estimate through the motion model | B. موشن ماڈل کے ذریعے پچھلے تخمینے کو پھیلا کر پرائر احتمال تقسیم |
| C. The likelihood of the current observation given the state | C. ریاست کی بنیاد پر موجودہ مشاہدے کی لائکلی ہوڈ |
| D. The normalization constant for the probability distribution | D. احتمال تقسیم کے لیے نرملائزیشن کنسٹینٹ |

**Correct Answer: B** | صحیح جواب: B

**MC 2:** The Extended Kalman Filter (EKF) linearizes nonlinear functions using which method?

| English | اردو |
|---------|------|
| A. Unscented transformation with sigma points | A. سگما پوائنٹس کے ساتھ انسنٹڈ ٹرانسفارمیشن |
| B. First-order Taylor series expansion around the current estimate | B. موجودہ تخمینے کے گرد پہلے درجے کی ٹیلر سیریز ایکسپینشن |
| C. Second-order Gaussian approximation | C. دوسرے درجے کا گاسین اپروکسیمیشن |
| D. Particle representation of the distribution | D. تقسیم کی پارٹیکل نمائندگی |

**Correct Answer: B** | صحیح جواب: B

### Short Answer (مختصر جواب)

**SA 1:** Explain why the Unscented Kalman Filter (UKF) is often preferred over the Extended Kalman Filter (EKF) for highly nonlinear systems.

| English | اردو |
|---------|------|
| UKF uses deterministic sigma points that capture the true mean and covariance more accurately than EKF's linearization. UKF propagates multiple points through the nonlinearity, capturing higher-order moments that EKF's first-order Taylor series expansion misses. Additionally, UKF does not require Jacobian computation, which can be difficult for non-differentiable functions. | UKF ڈیٹرمنسٹک سگما پوائنٹس استعمال کرتا ہے جو EKF کی لینیرایزیشن سے زیادہ درستگی سے اصل اوسط اور کوویریئس کو پکڑتے ہیں۔ UKF غیر لینیریٹی کے ذریعے متعدد پوائنٹس کو پھیلاتا ہے، اور اعلی درجے کے لمحے پکڑتا ہے جو EKF کی پہلے درجے کی ٹیلر سیریز ایکسپینشن سے چھوٹ جاتے ہیں۔ مزید برآں، UKF کو Jacobian کمپیوٹیشن کی ضرورت نہیں جو غیر differentiate-able فنکشنز کے لیے مشکل ہو سکتی ہے۔ |

---

## Key Equations (اہم مساوات)

| Equation | Description (وضاحت) |
|----------|---------------------|
| `x_k = A x_{k-1} + B u_k + w_k` | Linear state transition (لینیر ریاست ٹرانزیشن) |
| `z_k = H x_k + v_k` | Measurement model (پیمائش ماڈل) |
| `x̂⁻ = A x̂` | State prediction (ریاست کا پیڈکشن) |
| `P⁻ = A P Aᵀ + Q` | Covariance prediction (کوویریئس کا پیڈکشن) |
| `K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹` | Kalman gain (کالمن گین) |
| `x̂ = x̂⁻ + K(z - H x̂⁻)` | State update (ریاست کا اپڈیٹ) |
| `P = (I - K H) P⁻` | Covariance update (کوویریئس کا اپڈیٹ) |

---

## Glossary Terms (گلوسری کی اصطلاحات)

| English | Urdu (اردو) | Definition |
|---------|-------------|------------|
| State Estimation | ریاست کا تخمینہ | Process of determining hidden system state from noisy observations |
| Kalman Filter | کالمن فلٹر | Optimal recursive estimator for linear Gaussian systems |
| EKF | EKF | Extended Kalman Filter for nonlinear systems via linearization |
| UKF | UKF | Unscented Kalman Filter using sigma points for nonlinear systems |
| SLAM | سلم | Simultaneous Localization and Mapping |
| Bayes Filter | بیزین فلٹر | General framework for recursive state estimation |
| Particle Filter | پارٹیکل فلٹر | Non-parametric filter using particle representation |
| Sensor Fusion | سینسر فیوڈنگ | Combining multiple sensor readings for better estimate |
| IMU | IMU | Inertial Measurement Unit (accelerometer + gyroscope) |
| Odometry | اوڈومیٹری | Position estimation from wheel encoders |

---

**End of Chapter 4 | باب 4 کا اختتام**
