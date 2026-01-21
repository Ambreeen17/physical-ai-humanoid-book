# Glossary | گلوسری

**Chapter 4: State Estimation (ریاست کا تخمینہ)**

---

## A-E

### Bayes Filter (بیزین فلٹر)
**English:** A general recursive framework for state estimation that alternates between prediction and update steps using Bayes' rule.

**اردو:** ریاست کے تخمینے کے لیے ایک عام ریکرسیو فریم ورک جو بیز کے اصول کا استعمال کرتے ہوئے پیڈکشن اور اپڈیٹ اسٹیپس کے درمیان alternate کرتا ہے।

**Key Concept:** P(x|z) ∝ P(z|x) P(x) | مشاہدات کی بنیاد پر ریاست کا احتمال حساب کریں۔

---

### Covariance (کوویریئس)
**English:** A measure of how two variables change together. In state estimation, the covariance matrix P represents uncertainty in the state estimate.

**اردو:** دو متغیرات کے ساتھ کتنے بدلتے ہیں کا پیمانہ۔ ریاست کے تخمینے میں، کوویریئس میٹرکس P ریاست کے تخمینے میں عدم یقین کی نمائندگی کرتی ہے۔

**Formula:** E[(X - μ)(X - μ)ᵀ] | اوسط سے انحراف کا مربع۔

---

### EKF (ایکسٹینڈڈ کالمن فلٹر)
**English:** Extended Kalman Filter. Adapts the Kalman Filter to nonlinear systems by linearizing the transition and measurement functions using Jacobian matrices (first-order Taylor expansion).

**اردو:** کالمن فلٹر کو غیر لینیر سسٹمز کے لیے موزو کرتا ہے۔ Jacobian میٹرکس (پہلے درجے کی ٹیلر ایکسپینشن) کا استعمال کرتے ہوئے ٹرانزیشن اور پیمائش فنکشنز کو لینیرایز کرتا ہے۔

**Limitation:** Linearization errors for highly nonlinear systems | انتہائی غیر لینیر سسٹمز میں لینیرایزیشن غلطیاں۔

---

### Estimation (تخمینہ)
**English:** The process of determining the true state of a system from incomplete and noisy measurements.

**اردو:** غیر مکمل اور نویز والی پیمائشوں سے سسٹم کی اصل حالت طے کرنے کا عمل۔

**Types:** Point estimation, Bayesian estimation | نقطہ تخمینہ، بیزین تخمینہ۔

---

## F-J

### Gaussian Distribution (گاسین تقسیم)
**English:** A probability distribution characterized by its mean and covariance. Many natural phenomena and sensor noises follow this "bell curve" distribution.

**اردو:** اوسط اور کوویریئس کی خصوصیت والی احتمال تقسیم۔ بہت سی قدرتی ظواهر اور سینسر نویز اس "گھنٹی منحنی" تقسیم کی پیروی کرتے ہیں۔

**Notation:** N(μ, Σ) where μ is mean and Σ is covariance | N(μ, Σ) جہاں μ اوسط ہے اور Σ کوویریئس ہے۔

---

### IMU (انرشیل میزورمنٹ یونٹ)
**English:** Inertial Measurement Unit. A device containing accelerometers and gyroscopes that measures linear acceleration and angular velocity.

**اردو:** انرشیل میزورمنٹ یونٹ۔ ایک آلہ جو ایکسلرومیٹر اور جائروسکوپ پر مشتمل ہے جو لینیر ایکسیلریشن اور زاویائی رفتار ناپتا ہے۔

**Use in Robotics:** Dead reckoning, sensor fusion, attitude estimation | روبوٹکس میں: ڈیڈ ریکننگ، سینسر فیوڈنگ، ایٹیٹیوڈ تخمینہ۔

---

### Innovation (انوویشن)
**English:** The difference between the actual measurement and the predicted measurement. Also called "measurement residual" or "innovation signal."

**اردو:** اصل پیمائش اور پیڈکشن شدہ پیمائش کے درمیان فرق۔ "پیمائش ریزیڈوئل" یا "انوویشن سگنل" بھی کہا جاتا ہے۔

**Formula:** y = z - H x̂⁻ | پیمائش سے پیڈکشن شدہ پیمائش کو منہا کریں۔

---

### Jacobian (جیکوبین)
**English:** A matrix of partial derivatives that describes how a vector-valued function changes with respect to its inputs. Used in EKF for linearization.

**اردو:** جزوی مشتقات کی میٹرکس جو بیان کرتی ہے کہ ایک ویکٹر-قدر فنکشن اپنے ان پٹس کے ساتھ کیسے بدلتا ہے۔ EKF میں لینیرایزیشن کے لیے استعمال ہوتا ہے۔

**Formula:** J = ∂f/∂x where f is a vector function | J = ∂f/∂x جہاں f ایک ویکٹر فنکشن ہے۔

---

## K-O

### Kalman Filter (کالمن فلٹر)
**English:** An optimal recursive estimator for linear systems with Gaussian noise. It combines predictions from a motion model with measurements to produce an optimal state estimate.

**اردو:** گاسین نویز والے لینیر سسٹمز کے لیے ایک آپٹیمل ریکرسیو تخمینہ لگانے والا۔ یہ موشن ماڈل سے پیڈکشنز کو پیمائشوں کے ساتھ ملاتا ہے تاکہ آپٹیمل ریاست کا تخمینہ پیدا ہو۔

**History:** Developed by Rudolf Kalman in 1960 for aerospace applications | 1960 میں رڈولف کالمن نے ایروسپیس ایپلیکیشنز کے لیے تیار کیا۔

**Key Insight:** The Kalman gain K balances prediction confidence vs measurement confidence | کالمن گین K پیڈکشن کے اعتماد اور پیمائش کے اعتماد کے درمیان توازن برقرار رکھتا ہے۔

---

### Kalman Gain (کالمن گین)
**English:** The weight given to the measurement when updating the state estimate. Higher K means more trust in measurements; lower K means more trust in predictions.

**اردو:** ریاست کے تخمینے کو اپڈیٹ کرتے وقت پیمائش کو دیا جانے والا وزن۔ زیادہ K کا مطلب ہے پیمائشوں پر زیادہ اعتماد؛ کم K کا مطلب ہے پیڈکشنز پر زیادہ اعتماد۔

**Formula:** K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹ | کوویریئس اور نویز کی بنیاد پر حساب کیا جاتا ہے۔

---

### Likelihood (لائکلی ہوڈ)
**English:** The probability of observing a particular measurement given a hypothesized state.

**اردو:** ایک مفروضہ ریاست کو دیکھتے ہوئے ایک خاص پیمائش کے مشاہدے کا احتمال۔

**Note:** Different from probability - likelihood is not a proper probability distribution | احتمال سے مختلف - لائکلی ہوڈ ایک صحیح احتمال تقسیم نہیں۔

---

### Measurement Model (پیمائش ماڈل)
**English:** A mathematical model describing how sensor observations relate to the system state, typically: z = Hx + v where v is measurement noise.

**اردو:** ایک ریاضیاتی ماڈل جو بیان کرتا ہے کہ سینسر کے مشاہدات سسٹم کی ریاست سے کیسے جڑے ہوئے ہیں، عام طور پر: z = Hx + v جہاں v پیمائش کا شور ہے۔

---

### Motion Model (موشن ماڈل)
**English:** A mathematical model describing how the system state evolves over time, typically: x_k = f(x_{k-1}, u_k) + w_k.

**اردو:** ایک ریاضیاتی ماڈل جو بیان کرتا ہے کہ سسٹم کی ریاست وقت کے ساتھ کیسے تبدیل ہوتی ہے، عام طور پر: x_k = f(x_{k-1}, u_k) + w_k۔

---

## P-S

### Particle Filter (پارٹیکل فلٹر)
**English:** A non-parametric recursive estimator that represents the belief as a set of weighted particles (samples). Can represent arbitrary, multi-modal distributions.

**اردو:** ایک غیر پیرامیٹرک ریکرسیو تخمینہ لگانے والا جو بیلیف کو وزن والے پارٹیکلز (نمونوں) کے سیٹ کے طور پر نمائندہ کرتا ہے۔ جیسی بھی، ملٹی موڈل تقسیم کی نمائندگی کر سکتا ہے۔

**Advantages:** Handles non-Gaussian distributions, multi-modal beliefs | غیر گاسین تقسیم، ملٹی موڈل بیلیف سنبھالتا ہے۔

**Disadvantages:** Computationally expensive for high dimensions | اعلی ڈائمنشنز کے لیے کمپیوٹیشنلی مہنگا۔

---

### Posterior (پوسٹیریر)
**English:** The probability distribution of the state after incorporating new observations. This is what we want to compute in state estimation.

**اردو:** نئے مشاہدات کو شامل کرنے کے بعد ریاست کی احتمال تقسیم۔ یہ وہ ہے جس کا ہم ریاست کے تخمینے میں حساب لگانا چاہتے ہیں۔

**Notation:** P(x | z) | P(x | z) ریاست کو دیکھتے ہوئے مشاہدات کا احتمال۔

---

### Prediction (پیڈکشن)
**English:** The first step of a Bayes filter that propagates the current belief forward in time using the motion model.

**اردو:** بیزین فلٹر کا پہلا مرحلہ جو موشن ماڈل کا استعمال کرتے ہوئے موجودہ بیلیف کو آگے وقت میں پھیلاتا ہے۔

**Output:** Prior distribution P(x_t | u_{1:t-1}, z_{1:t-1}) | پرائر تقسیم کا پیڈکشن۔

---

### Prior (پرائر)
**English:** The probability distribution of the state before incorporating new observations. The result of the prediction step.

**اردو:** نئے مشاہدات کو شامل کرنے سے پہلے ریاست کی احتمال تقسیم۔ پیڈکشن اسٹیپ کا نتیجہ۔

**Notation:** P(x_t | u_{1:t-1}, z_{1:t-1}) | پیڈکشن کی بنیاد پر۔

---

### Process Noise (پراسس نویز)
**English:** Uncertainty in the motion model, representing unmodeled dynamics or disturbances. Denoted as w_k in state equations.

**اردو:** موشن ماڈل میں عدم یقین، غیر ماڈلڈ ڈائنامکس یا پریشانیوں کی نمائندگی کرتا ہے۔ ریاست مساوات میں w_k سے ظاہر کیا جاتا ہے۔

**Covariance:** Q matrix | کوویریئس: Q میٹرکس۔

---

### Resampling (ریسیمپلنگ)
**English:** In particle filters, the process of discarding low-weight particles and duplicating high-weight particles to maintain a finite set.

**اردو:** پارٹیکل فلٹر میں، کم وزن والے پارٹیکلز کو ہٹانے اور زیادہ وزن والے پارٹیکلز کو ڈپلیکیٹ کرنے کا عمل تاکہ ایک محدود سیٹ برقرار رہے۔

**Purpose:** Prevents particle depletion problem | پارٹیکل ختم ہونے کے مسئلے سے بچاتا ہے۔

---

### RMSE (آر ایم ایس ای)
**English:** Root Mean Square Error. A common metric for evaluating estimation accuracy: sqrt(mean((true - estimated)^2)).

**اردو:** روٹ مین اسکوئیر ایرر۔ تخمینہ کی درستگی جانچنے کا ایک عام میٹرک: sqrt(mean((true - estimated)^2))۔

**Use:** Lower RMSE = better estimation | کم RMSE = بہتر تخمینہ۔

---

## S-Z

### Sensor Fusion (سینسر فیوڈنگ)
**English:** The process of combining data from multiple sensors to produce a more accurate and reliable estimate than using any single sensor.

**اردو:** متعدد سینسرز کے ڈیٹا کو ملا کر ایک زیادہ درست اور قابل اعتماد تخمینہ پیدا کرنے کا عمل، بجائے کسی ایک سینسر کے۔

**Example:** Fusing IMU + odometry + GPS for robot localization | مثال: روبوٹ لوکلائزیشن کے لیے IMU + odometry + GPS کو فیوڈ کرنا۔

---

### Sigma Points (سگما پوائنٹس)
**English:** Deterministically selected points around the mean that capture the covariance of a distribution. Used in the Unscented Transform.

**اردو:** اوسط کے گرد ڈیٹرمنسٹک طور پر چنے گئے پوائنٹس جو تقسیم کے کوویریئس کو پکڑتے ہیں۔ Unscented Transform میں استعمال ہوتے ہیں۔

**Advantage:** Captures higher-order moments better than linearization | لینیرایزیشن سے بہتر اعلی درجے کے لمحے پکڑتا ہے۔

---

### SLAM (سلم)
**English:** Simultaneous Localization and Mapping. The problem of simultaneously building a map of an unknown environment while localizing the robot within it.

**اردو:** ایک ساتھ لوکلائزیشن اور میپنگ۔ ایک نامعلوم ماحول کا نقشہ بناتے ہوئے ساتھ ہی روبوٹ کو اس کے اندر لوکلائز کرنے کا مسئلہ۔

**Applications:** Autonomous navigation, exploration robots | ایپلیکیشنز: خودمختار نیویگیشن، ایکسپلوریشن روبوٹس۔

---

### State (ریاست)
**English:** A complete description of a system at a given time that is sufficient to predict its future behavior. In robotics, typically includes position, velocity, orientation.

**اردو:** ایک مخصوص وقت پر سسٹم کی مکمل وضاحت جو اس کے مستقبل کے برتاؤ کو پیڈکٹ کرنے کے لیے کافی ہو۔ روبوٹکس میں، عام طور پر پوزیشن، رفتار، اور اورینٹیشن شامل ہوتے ہیں۔

**Notation:** x (lowercase for vector) | x (ویکٹر کے لیے چھوٹے حروف میں)۔

---

### State Estimation (ریاست کا تخمینہ)
**English:** The process of determining the hidden state of a dynamic system from noisy, indirect measurements.

**اردو:** نویز والی، بالواسطہ پیمائشوں سے کسی ڈائنامک سسٹم کی چھپی ہوئی ریاست طے کرنے کا عمل۔

---

### UKF (انسنٹڈ کالمن فلٹر)
**English:** Unscented Kalman Filter. A variant of the Kalman Filter that uses the Unscented Transform (sigma points) to handle nonlinearities more accurately than EKF.

**اردو:** کالمن فلٹر کا ایک ویریئنٹ جو EKF سے زیادہ درستگی سے غیر لینیریٹی سنبھالنے کے لیے Unscented Transform (سگما پوائنٹس) استعمال کرتا ہے۔

**Advantage:** No Jacobian required, better for highly nonlinear systems | Jacobian کی ضرورت نہیں، انتہائی غیر لینیر سسٹمز کے لیے بہتر۔

---

### Uncertainty (عدم یقین)
**English:** The lack of complete knowledge about the true state of a system. Represented by probability distributions in Bayesian estimation.

**اردو:** سسٹم کی اصل حالت کے بارے میں مکمل علم کی کمی۔ بیزین تخمینہ میں احتمال تقسیموں کی نمائندگی کی جاتی ہے۔

**Types:** Epistemic (reducible) vs Aleatory (irreducible) | اپیسٹمک (قابل کم) بمقابلہ ایلیٹری (غیر قابل کم)۔

---

### Unscented Transform (انسنٹڈ ٹرانسفارم)
**English:** A method for propagating mean and covariance through a nonlinear transformation by evaluating the function at deterministic sigma points.

**اردو:** ڈیٹرمنسٹک سگما پوائنٹس پر فنکشن کی تشخیص کر کے اوسط اور کوویریئس کو غیر لینیر ٹرانسفارمیشن کے ذریعے پھیلانے کا ایک طریقہ۔

---

**End of Glossary | گلوسری کا اختتام**
