# Chapter 3: Sensors and Actuators

**Estimated Reading Time**: 45 minutes

**Learning Objectives Achieved**:
- Characterize sensor performance using resolution, accuracy, noise, and latency metrics
- Process LiDAR point clouds for obstacle detection and environment mapping
- Calibrate RGB-D cameras using checkerboard patterns and intrinsic/extrinsic parameters
- Select actuators based on torque-speed curves, precision requirements, and power constraints
- Implement sensor fusion using complementary filters and Kalman filters for IMU data

---

## 3.1 Sensor Taxonomy

### The Foundation of Robotic Perception

A robot's ability to interact intelligently with its environment depends fundamentally on its capacity to perceive both its internal state and the world around it. Sensors are the interface between the digital brain of a robot and the physical reality it inhabits. Without sensors, a robot would be blind to its surroundings and oblivious to its own configuration—incapable of the feedback necessary for meaningful action. Understanding how to classify, characterize, and select sensors is therefore one of the most practical and essential skills in robotics engineering.

### Proprioception versus Exteroception

Sensors fall into two broad categories based on what they measure. **Proprioceptive sensors** report on the robot's own internal state: the angles of its joints, the speed of its wheels, the orientation of its body. These measurements stay within the robot, telling the control system where each part is and how it is moving. Joint encoders on the Unitree G1's 23 degrees of freedom exemplify proprioception—they track the angular position of each motor with 0.01 degree resolution, providing the feedback necessary for precise limb control.

**Exteroceptive sensors**, by contrast, reach outward to measure properties of the environment. LiDAR scanners map the geometry of a room, cameras capture visual scenes, microphones listen for sounds. These sensors provide the contextual information that allows a robot to navigate, recognize objects, and interact with people and things that are not part of its own body. The Unitree G1 combines both types: its RealSense D435i RGB-D camera is exteroceptive, sensing the world in front of it, while its BMI088 IMU is proprioceptive, reporting the robot's own acceleration and rotation.

### Active versus Passive Sensors

Another dimension of classification concerns whether a sensor emits energy into the environment or merely receives what is already present. **Passive sensors** are observers: they wait for energy to arrive and then measure it. A standard camera is passive—it does not illuminate the scene but merely captures whatever light happens to be there. This makes passive sensors energy-efficient and unobtrusive, but they depend on adequate ambient conditions.

**Active sensors** take a more assertive approach: they emit their own energy and measure the response. LiDAR scanners fire laser pulses and measure the returning light. Structured-light projectors cast known patterns onto surfaces and observe the deformation. Ultrasonic sensors emit sound waves and listen for echoes. Active sensors can operate in complete darkness and often provide more precise distance measurements, but they consume more power and can interfere with other robots using similar sensing modalities. The Unitree G1's Mid-360 LiDAR is active, continuously scanning its environment with rotating laser beams even in pitch-black conditions.

### Performance Metrics for Sensor Characterization

When selecting and comparing sensors, engineers evaluate several key performance characteristics. Understanding these metrics enables informed design decisions and realistic expectations for system performance.

**Resolution** describes the smallest change in the measured quantity that the sensor can detect. A joint encoder with 0.01 degree resolution can distinguish positions differing by one-hundredth of a degree. Higher resolution provides more detailed measurements but generates more data that must be processed and stored.

**Accuracy** measures how close a sensor's readings are to the true value. A thermometer reading 20.1°C when the actual temperature is 20.0°C has an accuracy of 0.1°C. Accuracy is limited by systematic errors like calibration drift and is distinct from precision.

**Precision** (or repeatability) describes how consistently the sensor produces the same reading for the same true value. A precise sensor has low random variation, even if it is inaccurate due to calibration error. A sensor can be precise without being accurate, and accurate without being precise—the ideal sensor is both.

**Range** defines the minimum and maximum values the sensor can measure. The RealSense D435i on the Unitree G1 operates from 0.3 to 10 meters; measurements outside this interval are unreliable or unavailable.

**Field of view (FOV)** specifies the angular extent of the sensor's coverage. The Mid-360 LiDAR provides 360 degrees horizontally and 59 degrees vertically, enabling nearly omnidirectional perception for the robot's navigation.

**Latency** is the delay between a physical change and the sensor's report of that change. For real-time control systems, latency must be significantly shorter than the control loop period. High-latency sensors can cause instability in feedback loops.

**Power consumption** matters for mobile robots running on batteries. Active sensors like LiDAR typically draw more power than passive sensors like cameras, constraining the design of untethered systems.

> **FOUNDATIONS BOX: The Nyquist-Shannon Sampling Theorem**
>
> Any continuous signal can be perfectly reconstructed from its samples only if the sampling frequency exceeds twice the highest frequency component in the signal. For a sensor measuring a phenomenon that changes at frequency $f_{max}$, the sampling rate $f_s$ must satisfy $f_s > 2f_{max}$. If an IMU gyroscope measures rotations up to 100 Hz, it must sample at least 200 Hz to avoid aliasing—frequencies above 50 Hz would be misrepresented as lower frequencies in the data.

### Gotchas and Common Pitfalls

A common beginner mistake is confusing accuracy with resolution. A high-resolution sensor can report values to many decimal places while being entirely inaccurate if miscalibrated. Always calibrate sensors before relying on their absolute measurements.

Another pitfall is ignoring the relationship between sampling rate and power consumption. Higher sampling rates provide better temporal resolution but drain batteries faster and generate more data. For the Unitree G1's IMU, the 1 kHz sampling rate provides excellent temporal fidelity for dynamic motions but represents a significant portion of the robot's power budget.

---

## 3.2 LiDAR Sensors

### Ranging Through Time of Flight

Light Detection and Ranging (LiDAR) sensors measure distance by timing the journey of light. A laser pulse travels at the speed of light—approximately 299,792,458 meters per second—and the sensor measures the elapsed time until a fraction of that light returns after reflecting from a surface. The distance $d$ is computed from the round-trip time $\Delta t$ as:

$$d = \frac{c \cdot \Delta t}{2}$$

where $c$ is the speed of light. This time-of-flight (ToF) principle enables precise distance measurements without physical contact with the target.

Two primary implementations of ToF exist in commercial LiDAR. **Pulsed LiDAR** emits short laser pulses (typically nanoseconds in duration) and precisely measures the arrival time of returning photons. This approach supports long ranges (100+ meters) and high power, making it common in autonomous vehicles. **Phase-shift LiDAR** emits continuous laser beams at known frequencies and measures the phase shift between emitted and returned signals. While phase-shift systems typically have shorter range, they can achieve higher precision at close distances and are more compact.

### Two-Dimensional versus Three-Dimensional LiDAR

**2D LiDAR scanners** rotate a single laser beam in a horizontal plane, producing a 360-degree profile of the environment at a fixed height. These sensors excel at indoor navigation and obstacle detection for ground robots. The Hokuyo URG-04LX-UG01, for example, scans at 10 Hz with 0.36 degree angular resolution over a 4-meter range. Such scanners are compact, inexpensive, and power-efficient, making them ideal for smaller robots operating in structured indoor environments.

**3D LiDAR sensors** add vertical scanning to create point clouds—collections of (x, y, z) coordinates representing surfaces in three-dimensional space. The Velodyne VLP-16 uses 16 laser beams stacked vertically, rotating to produce a 360 × 30 degree field of view at up to 10 Hz. More advanced sensors like the Ouster OS1-64 offer 64 beams and higher resolution. The Unitree G1 employs the Mid-360, which provides 360-degree horizontal coverage with a 59-degree vertical field of view, enabling the robot to perceive both the horizontal layout of a room and the vertical structure of objects within it.

### Point Cloud Processing with the Point Cloud Library

Raw LiDAR data arrives as streams of distance measurements with associated angles. Converting these into useful representations for navigation and perception requires several processing steps. The Point Cloud Library (PCL) provides standardized algorithms for this purpose.

```python
# Python code using PCL (via python-pcl)
import pcl
import numpy as np

# Load a point cloud from ROS 2 bag or PCD file
cloud = pcl.load_XYZI('lidar_scan.pcd')

# Step 1: Statistical outlier removal
# Remove points that are isolated (likely noise)
outlier_filter = cloud.make_statistical_outlier_filter()
outlier_filter.set_mean_k(50)
outlier_filter.set_std_dev_mul_thresh(1.0)
cloud_filtered = outlier_filter.filter()

# Step 2: Voxel grid downsampling
# Create a 5cm voxel grid, keeping one point per voxel
vox = cloud_filtered.make_voxel_grid_filter()
vox.set_leaf_size(0.05, 0.05, 0.05)
cloud_downsampled = vox.filter()

# Step 3: RANSAC plane segmentation
# Detect the dominant ground plane
seg = cloud_downsampled.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_distance_threshold(0.03)
inliers, coefficients = seg.segment()

# Extract ground points and obstacles
ground_cloud = cloud_downsampled.extract(inliers, negative=False)
obstacle_cloud = cloud_downsampled.extract(inliers, negative=True)

print(f"Original points: {cloud.size}")
print(f"After filtering: {cloud_filtered.size}")
print(f"After downsampling: {cloud_downsampled.size}")
print(f"Ground points: {ground_cloud.size}")
print(f"Obstacle points: {obstacle_cloud.size}")
```

### ROS 2 Integration

In ROS 2 systems, LiDAR data typically flows through the `sensor_msgs/msg/LaserScan` message type for 2D scanners or `sensor_msgs/msg/PointCloud2` for 3D sensors. The following ROS 2 Python node demonstrates subscribing to the Unitree G1's Mid-360 LiDAR topic and detecting obstacles within a safety zone.

```python
# ROS 2 Python node for LiDAR obstacle detection
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/mid360_ros2/cloud',
            self.pointcloud_callback,
            10
        )
        self.safety_publisher = self.create_publisher(
            Float32MultiArray,
            '/obstacles/safety_zone',
            10
        )
        self.subscription  # prevent unused variable warning
        self.safety_threshold = 0.5  # meters

    def pointcloud_callback(self, msg):
        # Parse PointCloud2 data
        points = self.parse_pointcloud2(msg)

        # Find points within safety threshold
        distances = np.linalg.norm(points[:, :3], axis=1)
        close_points = points[distances < self.safety_threshold]

        # Publish obstacle warnings
        if len(close_points) > 0:
            obstacle_msg = Float32MultiArray()
            obstacle_msg.data = [float(len(close_points)),
                                 float(np.min(distances))]
            self.safety_publisher.publish(obstacle_msg)
            self.get_logger().warning(
                f'Detected {len(close_points)} obstacles, '
                f'min distance: {np.min(distances):.2f}m'
            )

    def parse_pointcloud2(self, msg):
        # Convert PointCloud2 to numpy array
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ])
        data = np.frombuffer(msg.data, dtype=dtype)
        return np.column_stack([data['x'], data['y'],
                                 data['z'], data['intensity']])
```

### Unitree G1 LiDAR Specifications

The Unitree G1 humanoid robot integrates the Mid-360 LiDAR sensor specifically designed for mobile robotics applications. This sensor provides 360-degree horizontal coverage with 0.33-degree angular resolution, enabling the robot to perceive its complete surroundings without blind spots. The vertical field of view spans 59 degrees, from 7 degrees below the horizontal to 52 degrees above, allowing detection of both ground obstacles and elevated objects like table edges or door frames.

Operating range extends from 0.5 to 30 meters, with typical ranging accuracy of 2 centimeters. The sensor rotates at 10 Hz, producing 43,200 points per revolution at maximum resolution. Data transmission occurs via gigabit Ethernet, with the Unitree SDK providing pre-built ROS 2 drivers that publish point cloud data on the `/mid360_ros2/cloud` topic. Power consumption of approximately 8 watts makes the Mid-360 suitable for battery-powered humanoid platforms.

> **FOUNDATIONS BOX: The Radar Equation**
>
> The fundamental relationship governing active ranging sensors relates transmitted power, received power, distance, and target properties. For a LiDAR, the received power $P_r$ from a target at distance $R$ with reflectivity $\rho$ is:
>
> $$P_r = \frac{P_t G_t G_r \lambda^2 \rho A_t}{(4\pi)^3 R^4}$$
>
> where $P_t$ is transmitted power, $G_t$ and $G_r$ are antenna gains, $\lambda$ is wavelength, and $A_t$ is the target area. The $R^4$ dependence explains why long-range sensing requires either higher transmitted power or more reflective targets—this is why autonomous vehicles use LiDAR rather than competing with smartphone sensors.

---

## 3.3 RGB-D Cameras

### The Fusion of Color and Depth

RGB-D cameras combine conventional color imaging with depth sensing, providing both the visual texture of a scene and the geometric structure of surfaces within it. The "D" stands for depth, and these sensors have transformed robotic manipulation and indoor navigation by making three-dimensional perception accessible at low cost and compact form factors. Where a standard camera produces a 2D array of color values, an RGB-D camera produces a 2D array where each pixel encodes both color and distance from the sensor.

Three distinct technologies enable commercial RGB-D cameras today. Understanding their principles helps explain their different strengths and limitations in robotic applications.

### Structured Light and Pattern Projection

Structured-light cameras project a known pattern onto the scene and observe how that pattern deforms on surfaces. By analyzing the deformation, the system triangulates the depth of each point. The Intel RealSense D400 series, including the D435i used on the Unitree G1, employs this approach with a laser projector that casts a pseudorandom infrared dot pattern. A dedicated depth processing ASIC computes the disparity between the projected pattern as observed from two perspectives, then converts this to depth measurements.

Structured-light systems excel at close range and provide accurate depth without the ambiguity problems of other approaches. They work well in indoor environments with textured surfaces but struggle with highly reflective surfaces that distort the projected pattern, bright ambient light that washes out the infrared pattern, and very dark surfaces that absorb too much of the projected light.

### Stereo Vision and Passive Depth

Stereo cameras use two conventional color cameras separated by a known baseline distance. The depth at each pixel is computed through triangulation: points that appear at different horizontal positions in the two images correspond to surfaces at different distances. The greater the separation between apparent positions (the disparity), the closer the surface. The ZED 2 camera from Stereolabs implements this approach with two 12-megapixel sensors and onboard processing for depth computation up to 20 meters.

Stereo vision is entirely passive, requiring no illumination beyond ambient light and consuming minimal power. This makes it ideal for outdoor applications and battery-powered robots. However, stereo matching algorithms struggle with textureless surfaces where no distinguishing features exist to correspond between the two images, and accuracy degrades quadratically with distance.

### Time-of-Flight Depth Cameras

ToF cameras measure depth by timing the round-trip of emitted light, similar to LiDAR but typically using modulated rather than pulsed illumination. The Microsoft Azure Kinect and Intel RealSense L515 use this approach, with the L515 employing a MEMS-based scanning system to build depth maps at up to 90 Hz.

### Camera Calibration and the Intrinsic Matrix

Accurate depth and position measurements from RGB-D cameras require precise calibration of the relationship between image coordinates and physical directions. This relationship is captured in the **camera intrinsic matrix**, which transforms normalized coordinates to pixel coordinates:

$$
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}
=
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
X / Z \\
Y / Z \\
1
\end{bmatrix}
$$

The parameters are the focal lengths ($f_x$, $f_y$) in pixels, the optical center ($c_x$, $c_y$) in pixel coordinates, and the depth scaling factor. Radial and tangential distortion coefficients account for lens imperfections.

```python
# OpenCV Python calibration for RGB-D camera
import cv2
import numpy as np
import glob

# Prepare object points (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points
obj_points = []  # 3D points in real world
img_points = []  # 2D points in image plane

# Load calibration images
images = glob.glob('calibration_images/*.png')
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

    if ret:
        obj_points.append(objp)
        # Refine corner positions to subpixel accuracy
        refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        img_points.append(refined)
        # Draw and display corners
        cv2.drawChessboardCorners(img, (7, 6), refined, ret)
        cv2.imshow('corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)

print("Camera Matrix:")
print(camera_matrix)
print("\nDistortion Coefficients:")
print(dist_coeffs)

# Save calibration for later use
np.save('camera_matrix.npy', camera_matrix)
np.save('distortion_coeffs.npy', dist_coeffs)
```

### Unitree G1 RealSense D435i Integration

The Unitree G1 incorporates the Intel RealSense D435i, an updated version of the popular D435 with an integrated IMU. This camera provides synchronized RGB and depth imaging at resolutions up to 1280 × 720 pixels for depth and 1920 × 1080 for color. The RGB sensor has an 87-degree diagonal field of view, while the depth module covers 69 degrees horizontally and 42 degrees vertically. Operating range extends from 0.3 to 10 meters, with depth accuracy typically within 2% of the measured distance.

```python
# ROS 2 Python node for Unitree G1 RealSense D435i
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class RealsenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.bridge = CvBridge()

        # Subscriptions to camera topics
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10
        )

        self.camera_matrix = None
        self.latest_depth = None

    def color_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('RGB', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Color conversion failed: {e}')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'Camera matrix: {self.camera_matrix}')

    def get_point_cloud(self, u, v):
        """Convert pixel coordinates to 3D point using depth."""
        if self.camera_matrix is None or self.latest_depth is None:
            return None

        Z = self.latest_depth[v, u]
        if np.isnan(Z) or Z == 0:
            return None

        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        return np.array([X, Y, Z])
```

### Gotchas and Common Pitfalls

RGB-D cameras exhibit systematic depth errors that vary with distance and surface properties. Structured-light cameras like the D435i typically show increasing error with distance, and their accuracy degrades on dark, shiny, or transparent surfaces. When deploying these sensors for manipulation tasks, calibrate depth measurements against ground truth for your specific operating conditions.

Another common issue is the temporal synchronization between color and depth streams. The RealSense D435i reports depth and color images at slightly different timestamps, which can cause visible misalignment when rendering colorized depth maps. Use the hardware-synchronized timestamps provided by the SDK to properly align data streams.

---

## 3.4 Inertial Measurement Units

### Measuring Motion Directly

Inertial Measurement Units (IMUs) measure the forces acting on a body, from which its acceleration and angular rotation can be inferred. These sensors provide direct measurements of motion without reference to external landmarks, making them essential for tracking robot orientation and detecting impacts. The Unitree G1 incorporates a Bosch BMI088 IMU that combines a 3-axis accelerometer and 3-axis gyroscope, sampling at 1 kHz to capture rapid motions during locomotion and manipulation.

### Accelerometer Principles

An accelerometer measures specific force—the vector sum of gravitational acceleration and the kinematic acceleration of the sensor body. In the sensor frame, the reading $\mathbf{a}_{raw}$ relates to the true acceleration $\mathbf{a}_{true}$ and gravitational acceleration $\mathbf{g}$ as:

$$\mathbf{a}_{raw} = \mathbf{R} \cdot \mathbf{a}_{true} + \mathbf{g} + \mathbf{b}_a + \mathbf{n}_a$$

where $\mathbf{R}$ is the rotation matrix from world to sensor frame, $\mathbf{b}_a$ is the accelerometer bias, and $\mathbf{n}_a$ is measurement noise. When the sensor is stationary, the accelerometer reads only gravity (approximately 9.81 m/s² along the vertical axis), which enables orientation estimation.

### Gyroscope Principles

A gyroscope measures angular velocity $\boldsymbol{\omega}$ directly, providing the rate of change of orientation. The gyroscope reading is:

$$\boldsymbol{\omega}_{raw} = \boldsymbol{\omega}_{true} + \mathbf{b}_g + \mathbf{n}_g$$

where $\mathbf{b}_g$ is the gyroscope bias (which drifts over time due to temperature changes and device aging) and $\mathbf{n}_g$ is random noise. Integration of angular velocity gives orientation, but bias errors accumulate as constant offset in the integrated angle:

$$\theta_{error}(t) = \int_0^t b_g(\tau) d\tau = b_g \cdot t$$

A gyroscope bias of just 0.1°/s accumulates to a 36-degree orientation error after 6 minutes—a significant problem for any system relying on gyroscope integration alone.

### Noise Characteristics and Error Sources

IMU measurements are corrupted by several error sources that must be understood for proper filtering. **White noise** appears as rapid random fluctuations with zero mean; it cannot be removed by averaging but can be reduced by low-pass filtering at the cost of bandwidth. The noise density is typically specified in mg/√Hz for accelerometers and °/s/√Hz for gyroscopes.

**Bias instability** (also called flicker noise or 1/f noise) causes the sensor bias to wander slowly over time. This low-frequency noise cannot be distinguished from true bias changes and represents a fundamental limit on long-term orientation estimation from gyroscopes alone.

**Temperature sensitivity** causes both bias and scale factor to vary with temperature. Many IMUs include temperature sensors and compensation algorithms, but residual temperature effects remain significant for precision applications.

### Complementary Filtering for Sensor Fusion

The accelerometer and gyroscope provide complementary information about orientation. The gyroscope accurately tracks rapid orientation changes but drifts over time. The accelerometer provides absolute orientation reference (through gravity direction) but is corrupted by linear accelerations during motion. A **complementary filter** combines these sources by applying high-pass filtering to gyroscope data and low-pass filtering to accelerometer data:

$$\hat{\theta}(s) = \frac{s}{s + \alpha} \cdot \frac{\omega_{gyro}(s)}{s} + \frac{\alpha}{s + \alpha} \cdot \theta_{accel}(s)$$

In discrete time with time constant $\tau$, the filter takes the form:

```python
# Complementary filter for roll/pitch estimation
import numpy as np

class ComplementaryFilter:
    def __init__(self, alpha=0.98, sample_time=0.001):
        self.alpha = alpha
        self.dt = sample_time
        self.roll = 0.0
        self.pitch = 0.0
        self.tau = (1 - alpha) / alpha * sample_time if alpha > 0 else 0

    def update(self, ax, ay, az, gx, gy, gz):
        # Convert gyroscope readings from deg/s to rad/s
        gx_rad = np.deg2rad(gx)
        gy_rad = np.deg2rad(gy)
        gz_rad = np.deg2rad(gz)

        # Integrate gyroscope for orientation change
        self.roll += gx_rad * self.dt
        self.pitch += gy_rad * self.dt

        # Compute orientation from accelerometer
        accel_roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # Complementary filter: blend gyro integration with accel reference
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

        return self.roll, self.pitch
```

### Advanced Sensor Fusion: Madgwick and Mahony Filters

For applications requiring full 3D orientation estimation with higher accuracy, the Madgwick and Mahony filters provide sophisticated solutions. These filters use quaternion representation to avoid gimbal lock and incorporate magnetometer data for heading (yaw) reference. The Madgwick filter update equations are:

$$\dot{q} = \frac{1}{2} q \otimes \omega + \beta \cdot \frac{\nabla f}{\|\nabla f\|}$$

where $q$ is the quaternion representing orientation, $\omega$ is the angular velocity quaternion, and $\beta$ is a filter gain parameter controlling how aggressively the filter corrects accelerometer-based orientation estimates.

```python
# Madgwick filter implementation for 3D orientation
import numpy as np

class MadgwickFilter:
    def __init__(self, beta=0.1, sample_period=0.001):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
        self.sample_period = sample_period

    def update(self, accel, gyro, mag=None):
        q = self.q.copy()
        ax, ay, az = accel
        gx, gy, gz = gyro

        # Normalize accelerometer measurement
        norm = np.sqrt(ax**2 + ay**2 + az**2)
        ax, ay, az = ax/norm, ay/norm, az/norm

        # Quaternion multiplication
        q0, q1, q2, q3 = q

        # Gradient descent step
        f = np.array([
            2*(q1*q3 - q0*q2) - ax,
            2*(q0*q1 + q2*q3) - ay,
            2*(0.5 - q1**2 - q2**2) - az
        ])

        J = np.array([
            [-2*q2, 2*q3, -2*q0, 2*q1],
            [2*q1, 2*q0, 2*q3, 2*q2],
            [0, -4*q1, -4*q2, 0]
        ])

        step = J.T @ f
        step_norm = np.linalg.norm(step)
        if step_norm > 0:
            step = step / step_norm

        # Gyroscope quaternion rate of change
        q_dot = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
            q0*gx + q2*gz - q3*gy,
            q0*gy - q1*gz + q3*gx,
            q0*gz + q1*gy - q2*gx
        ])

        # Apply filter update
        q = q + (q_dot - self.beta * step) * self.sample_period
        self.q = q / np.linalg.norm(q)

        return self.q
```

### Unitree G1 IMU Specifications

The BMI088 IMU on the Unitree G1 provides accelerometer measurements over ranges of ±3g, ±6g, ±12g, or ±16g, with 16-bit resolution giving 2048 counts per g at the maximum range. Gyroscope ranges span ±125°/s, ±250°/s, ±500°/s, ±1000°/s, or ±2000°/s, again with 16-bit resolution. The 1 kHz sampling rate ensures that the sensor captures the full dynamics of the robot's movements during locomotion, where joint torques and ground reaction forces change rapidly.

> **FOUNDATIONS BOX: Quaternion Representation of Orientation**
>
> Quaternions extend complex numbers to represent 3D rotations without the gimbal lock problem of Euler angles. A quaternion $q = w + xi + yj + zk$ with $w^2 + x^2 + y^2 + z^2 = 1$ represents a rotation of angle $2\arccos(w)$ around the axis $(x, y, z)$. Quaternions compose through multiplication: $q_{combined} = q_1 \otimes q_2$, and rotation of a vector $\mathbf{v}$ is computed as $q \otimes \mathbf{v} \otimes q^*$. The Unitree G1's control system uses quaternions internally to avoid the discontinuities that plague Euler angle representations during continuous rotation.

### Gotchas and Common Pitfalls

IMU data sheets typically specify noise density, but this specification alone is insufficient for system design. The Allan variance plot provides a more complete picture of noise at different averaging times, revealing the characteristic "V" shape where accelerometer bias instability dominates at long time scales. When designing filters for your application, consider the frequency content of expected motions and select filtering strategies that attenuate noise while preserving signal components of interest.

---

## 3.5 Force-Torque Sensors

### Measuring Interaction Forces

Force-torque (F/T) sensors measure the full six-dimensional wrench—three forces and three torques—applied at a sensing interface. These sensors enable robots to perform delicate manipulation tasks where precise control of contact forces is essential, and they provide the feedback necessary for legged robots to maintain balance on uncertain terrain. While the Unitree G1 primarily relies on joint torque estimation and foot pressure sensors, industrial robots routinely incorporate 6-axis F/T sensors at the wrist for assembly, polishing, and collaborative tasks.

### Strain Gauge-Based Sensing

Most F/T sensors operate on the principle of elastic deformation under load. When a force or torque is applied to the sensor body, internal structures deform slightly—typically on the order of micrometers. Precision strain gauges bonded to these structures convert the deformation into electrical resistance changes, which are then measured using Wheatstone bridge circuits.

A full 6-axis F/T sensor typically uses multiple strain gauges arranged in bridge configurations that are sensitive to specific force components while rejecting cross-axis interference. The ATI Nano17, a common sensor for research applications, uses a monolithic aluminum structure with strain gauges arranged to measure all six components with minimal crosstalk.

### Calibration and Zero Offsetting

F/T sensors require calibration to convert raw bridge readings into force and torque units. Factory calibration provides a transformation matrix that maps raw readings to the six output channels. However, mounting the sensor and attaching the tool create additional loads—most significantly, the weight of the tool acting at its center of mass—that must be compensated.

```python
# Force-Torque sensor calibration and gravity compensation
import numpy as np

class ForceTorqueSensor:
    def __init__(self, calibration_matrix, load_mass, load_com):
        # Calibration matrix: maps raw readings to wrench
        self.calibration = calibration_matrix  # 6x6 matrix
        self.load_mass = load_mass  # kg
        self.load_com = load_com  # (x, y, z) in sensor frame, meters
        self.g = np.array([0, 0, -9.81])  # Gravity vector
        self.zero_offset = np.zeros(6)

    def set_zero(self, raw_readings):
        """Set zero offset with current readings."""
        self.zero_offset = self.calibration @ raw_readings

    def read_wrench(self, raw_readings):
        """Convert raw readings to calibrated wrench with compensation."""
        # Apply calibration
        wrench = self.calibration @ raw_readings - self.zero_offset

        # Compensate for load weight
        # Force component: gravity times mass
        wrench[:3] -= self.load_mass * self.g

        # Torque component: cross product of COM offset with weight
        com = np.array(self.load_com)
        wrench[3:] -= np.cross(com, self.load_mass * self.g)

        return wrench

    def detect_contact(self, wrench, thresholds):
        """Detect if contact forces exceed thresholds."""
        contact = False
        for i in range(6):
            if abs(wrench[i]) > thresholds[i]:
                contact = True
                break
        return contact
```

### Applications in Robotics

Force-torque sensors enable several important robotic capabilities. **Impedance control** uses F/T feedback to regulate the apparent stiffness and damping of the robot's interaction with the environment, allowing safe physical human-robot collaboration. **Assembly operations** require controlled force application during peg-in-hole insertions and other tasks where visual guidance alone is insufficient. **Grappling and manipulation** of deformable objects benefits from continuous force feedback that prevents crushing or dropping.

For legged robots like the Unitree G1, foot-mounted pressure sensors serve a related function, estimating ground reaction forces from the distribution of pressure across the foot. These estimates feed into balance controllers that adjust joint torques to maintain stability.

### Gotchas and Common Pitfalls

F/T sensors are sensitive to temperature changes, which cause both zero drift and scale factor changes. In applications requiring high accuracy, allow the sensor to thermally equilibrate before operation and periodically recalibrate during extended use. Additionally, F/T sensors have limited overload ratings—applying forces beyond the sensor's rated capacity can cause permanent damage or shift the calibration.

---

## 3.6 Actuators

### Converting Electrical Energy to Mechanical Motion

Actuators are the muscles of robotic systems, converting electrical, hydraulic, or pneumatic energy into controlled motion. The choice of actuator technology fundamentally shapes a robot's capabilities, determining its strength, speed, precision, and efficiency. Understanding actuator principles enables engineers to select appropriate motors for each joint and design transmission systems that match actuator characteristics to task requirements.

### DC Motor Fundamentals

The workhorse of modern robotics is the brushed or brushless DC motor. When current flows through a motor's windings in the presence of a magnetic field, Lorentz forces produce rotation. The motor's behavior is governed by several fundamental relationships.

The **back EMF** voltage $E$ generated by a spinning motor is proportional to its angular velocity:

$$E = k_e \cdot \omega$$

where $k_e$ is the back EMF constant (typically in V·s/rad or mV/rpm) and $\omega$ is the angular velocity in rad/s. The **torque constant** $k_t$ relates current to developed torque:

$$\tau = k_t \cdot I$$

For SI units, $k_e = k_t$ (this is a fundamental property of electromechanical energy conversion). The **electrical time constant** $\tau_e = L/R$ (inductance divided by resistance) governs how quickly current can change in response to voltage commands, while the **mechanical time constant** $\tau_m = J/B$ (inertia divided by damping) determines acceleration response.

```python
# DC motor model for simulation and control design
import numpy as np

class DCMotor:
    def __init__(self, R=1.0, L=0.001, k_e=0.01, k_t=0.01, J=0.001, B=0.001):
        self.R = R      # Terminal resistance (ohms)
        self.L = L      # Terminal inductance (henries)
        self.k_e = k_e  # Back EMF constant (V·s/rad)
        self.k_t = k_t  # Torque constant (N·m/A)
        self.J = J      # Rotor inertia (kg·m²)
        self.B = B      # Viscous damping (N·m·s/rad)

    def simulate(self, voltage, load_torque, dt, initial_velocity=0.0):
        """Simulate motor response to voltage step."""
        t = np.arange(0, 1.0, dt)  # 1 second simulation
        n_steps = len(t)
        velocity = np.zeros(n_steps)
        current = np.zeros(n_steps)
        velocity[0] = initial_velocity

        for i in range(1, n_steps):
            # Electrical equation: V = IR + L dI/dt + E
            # dI/dt = (V - IR - k_e * omega) / L
            i_dot = (voltage - self.R * current[i-1] - self.k_e * velocity[i-1]) / self.L
            current[i] = current[i-1] + i_dot * dt

            # Mechanical equation: J dω/dt = τ - Bω - τ_load
            # τ = k_t * I
            torque = self.k_t * current[i]
            omega_dot = (torque - self.B * velocity[i-1] - load_torque) / self.J
            velocity[i] = velocity[i-1] + omega_dot * dt

        return t, velocity, current

    def torque_speed_curve(self, voltage, load_torque=0):
        """Compute steady-state torque-speed relationship."""
        # At steady state: dI/dt = 0, dω/dt = 0
        # V = IR + k_e * ω
        # ω = (V - IR) / k_e
        # τ = k_t * I
        currents = np.linspace(0, voltage / self.R, 100)
        torques = self.k_t * currents
        speeds = (voltage - self.R * currents) / self.k_e - load_torque / self.B
        return torques, speeds
```

### Servo Motors and Position Control

Servo motors integrate a DC motor with position feedback from an encoder or potentiometer, along with control electronics that maintain a commanded position. The typical hobby servo accepts PWM signals with pulse widths from 1 to 2 milliseconds corresponding to positions across a 180-degree range. Industrial servomotors provide more sophisticated interfaces (CAN, EtherCAT, RS-485) and support velocity and torque modes in addition to position control.

When selecting servomotors for a robotic joint, consider the **torque-speed curve**: the motor delivers maximum torque at zero speed (stall torque) and zero torque at maximum speed (no-load speed). The operating point for a given application should lie well below these limits to ensure responsiveness and avoid thermal overload.

### Stepper Motors and Open-Loop Control

Stepper motors divide full rotation into discrete steps, typically 200 steps per revolution (1.8 degrees per step) for common hybrid steppers. By energizing windings in sequence, the motor can be positioned precisely without feedback—this is the key advantage of stepper systems for many applications.

However, stepper motors have limitations. At high speeds, torque decreases due to winding inductance and back EMF. If the load torque exceeds the available torque at a given speed, the motor can **miss steps**, losing position without any indication. For this reason, closed-loop systems using DC motors with encoders provide greater reliability in critical applications.

### Unitree G1 Direct-Drive Actuators

The Unitree G1 employs a direct-drive actuator architecture where each joint is driven by a brushless motor without reduction gearing. This design eliminates the backslash, friction, and wear associated with gear trains, providing precise and responsive control of joint torques. The peak torque rating of 70 Nm per joint enables the robot to support its own weight and perform dynamic movements like walking and squatting.

Direct-drive actuators require high-torque, low-speed motors, which are larger and more massive than the high-speed motors typically used with gear reductions. The Unitree design balances this trade-off by using custom motors optimized for the robot's joint requirements and by accepting the mass penalty in exchange for control fidelity.

> **FOUNDATIONS BOX: The DC Motor Power Equation**
>
> Electrical input power to a DC motor is $P_{in} = V \cdot I$. Mechanical output power is $P_{out} = \tau \cdot \omega$. The efficiency $\eta = P_{out} / P_{in}$ relates these quantities. The power flow can also be expressed as:
>
> $$P_{in} = I^2 R + E \cdot I = I^2 R + k_e \omega I$$
>
> The first term represents copper losses (heating in the windings), while the second term is converted to mechanical power. At any operating point, the product of torque and speed is maximized when the back EMF equals half the supply voltage—a useful guideline for selecting operating points that maximize power output.

### Gotchas and Common Pitfalls

Motor datasheets often quote peak (stall) torques and no-load speeds that cannot be achieved simultaneously. When sizing a motor for an application, plot the required torque-speed curve for your task and verify that it lies within the motor's continuous operating region, not just the peak capability. Continuous operation at peak torque will cause thermal shutdown or damage.

Another common mistake is neglecting the reflected inertia from the load. When a motor drives a link through a gear reduction of ratio $N$, the reflected inertia at the motor shaft is $J_{reflected} = J_{load} / N^$. High gear ratios therefore reduce the effective inertia that the motor must accelerate, improving dynamic response but reducing available torque at the output.

---

## Summary and Key Takeaways

This chapter established the foundation for understanding the sensors and actuators that enable robotic perception and action. You should now be able to:

- **Classify sensors** by their measurement domain (proprioceptive vs. exteroceptive) and energy interaction (active vs. passive), and evaluate them using resolution, accuracy, precision, range, field of view, and latency metrics.
- **Process LiDAR point clouds** using the Point Cloud Library, including filtering, downsampling, and plane segmentation for obstacle detection.
- **Calibrate RGB-D cameras** by computing the intrinsic matrix and distortion coefficients using checkerboard patterns in OpenCV.
- **Implement sensor fusion** using complementary filters and the Madgwick filter to combine accelerometer and gyroscope data for robust orientation estimation.
- **Select actuators** by analyzing torque-speed curves, understanding motor constants, and matching motor characteristics to application requirements.

### Prerequisites for This Chapter
- Basic signal processing concepts (sampling, filtering, noise)
- Linear algebra (matrices, vectors, coordinate transformations)
- Python programming for sensor data processing

### Topics for Further Study
- Extended Kalman filtering for multi-sensor fusion
- Visual-inertial odometry combining camera and IMU data
- Model-predictive control for legged robot locomotion
- Adaptive impedance control for robotic manipulation

### Chapter Assessment Checklist

- [ ] I can explain the difference between proprioceptive and exteroceptive sensors with examples from the Unitree G1.
- [ ] I can compute distance from LiDAR time-of-flight measurements and process point clouds using PCL.
- [ ] I can calibrate an RGB-D camera and convert between pixel coordinates and 3D points using the intrinsic matrix.
- [ ] I can implement a complementary filter combining accelerometer and gyroscope data for orientation estimation.
- [ ] I can analyze a DC motor's torque-speed curve and select appropriate operating points.
