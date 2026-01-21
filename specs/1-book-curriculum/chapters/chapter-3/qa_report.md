# Chapter 3 QA Report: Sensors and Actuators

**Date**: 2026-01-04
**Reviewer**: QA Reviewer (Robotics Documentation Specialist)
**Chapter Path**: `C:\boook\specs\1-book-curriculum\chapters\chapter-3\`

---

## Summary

| Category | Pass | Fail | Critical Issues | High Issues | Medium Issues | Low Issues |
|----------|------|------|-----------------|-------------|---------------|------------|
| Terminology | 8 | 3 | 0 | 1 | 2 | 0 |
| Version | 5 | 3 | 0 | 2 | 1 | 0 |
| Assumptions | 4 | 2 | 0 | 1 | 1 | 0 |
| Hardware | 3 | 1 | 0 | 0 | 1 | 0 |
| **Total** | **20** | **9** | **0** | **4** | **5** | **0** |

**Overall Status**: CONDITIONAL PASS

---

## Issues List

### Terminology Issues

**Issue T1: Inconsistent ROS Topic Naming Convention**
- **Severity**: High
- **Location**: `content.md`, line 142; `lab\lab-3-1-lidar-point-cloud\starter.py`, line 72
- **Problem**: Topic naming uses inconsistent prefixes. Some topics use `/mid360_ros2/cloud` (Unitree SDK convention) while others use generic `/point_cloud_raw`. The PRODUCTION_SPEC specifies `/mid360_ros2/cloud` but the lab uses `/point_cloud_raw` without explanation.
- **Impact**: Students may confuse ROS topic naming conventions. Code examples in content.md will not work with the lab setup without modification.
- **Reference**: [ROS 2 Topic Naming Convention](https://docs.ros.org/en/humble/Concepts/About-Topic-.html)

**Issue T2: "Complementary Filtering" Misattributed in Lab 3.1**
- **Severity**: Medium
- **Location**: `lab\lab-3-1-lidar-point-cloud\README.md`, line 57-59
- **Problem**: The README states "While not the focus of this lab, the same filtering principles apply to spatial data. We'll use RANSAC to separate ground planes from obstacles." This incorrectly associates RANSAC with complementary filtering concepts. RANSAC is a model fitting algorithm, not a filter with complementary frequency characteristics.
- **Impact**: Conceptual confusion for students learning sensor fusion. RANSAC and complementary filters serve fundamentally different purposes.
- **Reference**: [PCL RANSAC Documentation](https://pointclouds.org/documentation/group__sample__consensus.html)

**Issue T3: Confusing Use of "ROS 2 bag"**
- **Severity**: Medium
- **Location**: `content.md`, line 89-90
- **Problem**: Code comment states "Load a point cloud from ROS 2 bag or PCD file". ROS 2 uses "bag files" (singular) not "ROS 2 bags". Additionally, the code uses `pcl.load_XYZI()` which is not a standard PCL function for ROS bag data.
- **Impact**: Misleading code example that won't work as written. `pcl.load_XYZI()` does not exist in python-pcl.
- **Reference**: [ROS 2 Bag Format](https://docs.ros.org/en/humble/Concepts/About-rosbag2.html)

---

### Version Issues

**Issue V1: python-pcl Package Unmaintained**
- **Severity**: High
- **Location**: `lab\lab-3-1-lidar-point-cloud\Dockerfile.lab`, line 12; `lab\lab-3-1-lidar-point-cloud\starter.py`, line 16
- **Problem**: The Dockerfile installs `python3-pcl` which is the old, unmaintained Python PCL binding. Modern ROS 2 Humble installations should use `pcl` (via `pip install pclpy` or the open3d library for Python-native PCL functionality). The `python3-pcl` package has known compatibility issues with Python 3.10+ and ROS 2 Humble.
- **Impact**: Students will encounter import errors when attempting to run the lab code in the Docker environment. The package may not install correctly on Ubuntu 22.04.
- **Reference**: [PCL Python Bindings Status](https://github.com/strawlab/python-pcl)

**Issue V2: Incomplete ROS 2 Service Import**
- **Severity**: High
- **Location**: `lab\lab-3-2-rgbd-calibration\starter.py`, line 16
- **Problem**: The solution.py imports `from std_srvs.srv import Trigger` but starter.py has the import commented out/missing in the TODO section. The code references `Trigger` service without proper import in starter.py, causing a NameError when students uncomment the TODO code.
- **Impact**: Students following the TODO instructions will encounter NameError at runtime.
- **Reference**: [ROS 2 Service Documentation](https://docs.ros.org/en/humble/Concepts/About-Services.html)

**Issue V3: cv_bridge API Inconsistency**
- **Severity**: Medium
- **Location**: `lab\lab-3-2-rgbd-calibration\solution.py`, line 166
- **Problem**: The code uses `self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')` with positional argument `desired_encoding`, but in starter.py the comment suggests using positional encoding. The correct keyword argument is `encoding=` not `desired_encoding=`.
- **Impact**: cv_bridge will raise TypeError with the incorrect keyword argument.
- **Reference**: [cv_bridge API Documentation](https://docs.ros.org/en/humble/How-To/Guides/CvBridge.html)

---

### Assumption Issues

**Issue A1: Missing Dependencies in package.xml**
- **Severity**: High
- **Location**: `lab\lab-3-1-lidar-point-cloud\package.xml`
- **Problem**: The package.xml declares `ament_python` as build_type but does not declare dependencies on `rclpy`, `sensor_msgs`, or `pcl`. Additionally, `setup.py` is referenced in Dockerfile but does not exist in the lab directory.
- **Impact**: `colcon build` will fail due to missing dependencies and missing setup.py file referenced by Dockerfile.
- **Reference**: [ROS 2 Package.xml Specification](https://docs.ros.org/en/humble/How-To/Developing-a-ROS-2-Package.html)

**Issue A2: Undocumented Hardware Requirement for Lab 3.2**
- **Severity**: Medium
- **Location**: `lab\lab-3-2-rgbd-calibration\README.md`
- **Problem**: The lab assumes access to a physical RGB-D camera (RealSense D435) for calibration. The Dockerfile installs `ros-humble-realsense2-camera` but there's no simulated data source or mock mode documented. Students without hardware cannot complete the lab.
- **Impact**: Lab is not completable in simulation-only environments. No fallback option documented.
- **Reference**: [Realsense ROS 2 Package](https://github.com/IntelRealSense/realsense-ros)

---

### Hardware Issues

**Issue H1: Unrealistic Point Count in Expected Output**
- **Severity**: Medium
- **Location**: `lab\lab-3-1-lidar-point-cloud\README.md`, line 302
- **Problem**: The expected output shows "Raw points: 72000" which implies a 3D LiDAR with 720 points per revolution at 100Hz. The Mid-360 typically produces ~200,000 points/sec, but the expected output of 72,000 points per scan is unrealistic for typical LiDAR configurations. For a 10Hz scan rate with the Mid-360's 200k pts/s, expected points would be ~20,000 per scan.
- **Impact**: Students may be confused when their actual point counts differ significantly from documented expectations.
- **Reference**: [Livox Mid-360 Specifications](https://www.livox.com/mid-360/)

---

## Fix Suggestions

### Issue T1: Inconsistent ROS Topic Naming Convention

**Issue ID**: T1
**Recommended Fix**: Update `lab\lab-3-1-lidar-point-cloud\starter.py` to use `/mid360_ros2/cloud` as the default topic, and add a comment explaining that this matches the Unitree G1's native topic naming.

```python
# Topic for Unitree G1 Mid-360 LiDAR
# Default: /mid360_ros2/cloud (Unitree SDK convention)
# For simulation: /point_cloud_raw
self.create_subscription(
    PointCloud2,
    '/mid360_ros2/cloud',  # Changed from /point_cloud_raw
    self.point_cloud_callback,
    10
)
```

**Rationale**: Aligns lab code with PRODUCTION_SPEC and provides clarity on hardware vs. simulation topics.

**Verification**: Verify topic exists by running `ros2 topic list` on Unitree G1 or simulation.

---

### Issue T2: Complementary Filtering Misattribution

**Issue ID**: T2
**Recommended Fix**: Revise the README to remove the misleading reference to complementary filtering:

```markdown
### RANSAC for Ground Plane Detection

While not the focus of this lab, RANSAC (Random Sample Consensus) is a robust estimation algorithm that fits models to noisy data. We'll use RANSAC to separate ground planes from obstacles by fitting a plane model and identifying points that deviate significantly from the model.
```

**Rationale**: Accurately describes RANSAC without conflating it with unrelated filter concepts.

**Verification**: Review PCL documentation to confirm RANSAC is a model fitting algorithm.

---

### Issue T3: ROS 2 bag Misuse

**Issue ID**: T3
**Recommended Fix**: Update content.md line 89-90 with correct terminology and working code:

```python
# Load a point cloud from ROS 2 bag file or PCD file
# Note: Use pcl_ros to convert from ROS bag
# cloud = pcl.PointCloud()
# from pcl_ros import transforms
# cloud = transforms.convertFromROS(msg)  # Example only
```

**Rationale**: Corrects terminology and provides accurate guidance on point cloud loading.

**Verification**: Test code against PCL Python bindings documentation.

---

### Issue V1: python-pcl Package

**Issue ID**: V1
**Recommended Fix**: Update Dockerfile.lab to use Open3D or remove PCL-specific dependencies:

```dockerfile
# Install Open3D for point cloud processing (modern alternative to python-pcl)
RUN pip3 install --no-cache-dir open3d numpy transforms3d

# Remove python3-pcl and use Open3D API instead
# RUN apt-get install -y python3-pcl  # REMOVE THIS LINE
```

Update starter.py imports:
```python
# Use Open3D instead of pcl
import open3d as o3d
```

**Rationale**: Open3D is actively maintained, has Python 3.10+ support, and provides equivalent PCL functionality.

**Verification**: Test Open3D import and basic point cloud operations in Docker container.

---

### Issue V2: Missing Service Import

**Issue ID**: V2
**Recommended Fix**: Add the missing import in the starter.py TODO section comment:

```python
# =====================================================================
# TODO 2.2: Create services for calibration triggers
# =====================================================================
# Required import: from std_srvs.srv import Trigger
# HINT: Create services for:
#       - /calibrate (Trigger) -> run calibration
#       - /save_images (Trigger) -> save collected images
#
# Your code here:
# self.calibrate_srv = self.create_service(Trigger, '/calibrate', self.calibrate_callback)
# self.save_srv = self.create_service(Trigger, '/save_images', self.save_images_callback)
```

**Rationale**: Provides complete instruction including necessary imports.

**Verification**: Run syntax check on starter.py after modifications.

---

### Issue V3: cv_bridge API

**Issue ID**: V3
**Recommended Fix**: Correct the cv_bridge call in solution.py:

```python
# In image_callback:
cv_image = self.bridge.imgmsg_to_cv2(msg, encoding='bgr8')  # Changed desired_encoding to encoding

# In depth_callback:
self.last_depth = self.bridge.imgmsg_to_cv2(msg, encoding='passthrough')  # Fixed
```

**Rationale**: Uses correct cv_bridge API keyword argument.

**Verification**: Test cv_bridge conversion with correct encoding parameter.

---

### Issue A1: Missing Dependencies

**Issue ID**: A1
**Recommended Fix**: Create missing setup.py and update package.xml:

**setup.py** (new file):
```python
from setuptools import setup
import os

setup(
    name='lab_3_1',
    version='0.1.0',
    packages=['lab_3_1'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lab_3_1']),
        ('share/lab_3_1', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Student',
    maintainer='Student',
    description='Lab 3.1: LiDAR Point Cloud Processing',
    license='MIT',
    tests_require=['pytest'],
)
```

**package.xml update**:
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>pcl_ros</exec_depend>
<exec_depend>std_srvs</exec_depend>
```

**Rationale**: Provides complete ROS 2 package structure for successful build.

**Verification**: Run `colcon build --packages-select lab_3_1` and verify success.

---

### Issue A2: Hardware Requirement Documentation

**Issue ID**: A2
**Recommended Fix**: Add simulation mode documentation to README.md:

```markdown
## Running Without Physical Camera

If you do not have access to a RealSense D435, you can use the Intel RealSense SDK's simulator or generate synthetic calibration data:

### Option 1: Use realsense-ros with simulated device
```bash
# Install librealsense simulation tools
sudo apt install ros-humble-realsense2-camera
```

### Option 2: Generate synthetic checkerboard images
See `tools/generate_calibration_images.py` in the lab directory.

### Option 3: Use provided sample dataset
Sample calibration images are available in `/tmp/sample_calibration_images/` in the Docker container.
```

**Rationale**: Provides alternative paths for students without hardware.

**Verification**: Confirm synthetic data generation script works or sample data exists.

---

### Issue H1: Point Count Documentation

**Issue ID**: H1
**Recommended Fix**: Update README.md expected output to reflect realistic values:

```
# Updated expected output
[lidar_processor] INFO: Raw points: 20000    # Typical for Mid-360 at 10Hz
[lidar_processor] INFO: After voxel filter: 1500
[lidar_processor] INFO: Ground plane inliers: 1200
[lidar_processor] INFO: Obstacles detected: 300
```

Add explanation:
> Note: Actual point counts depend on LiDAR model and configuration. The Mid-360 produces approximately 200,000 points/second. At 10Hz scan rate, expect ~20,000 points per scan.

**Rationale**: Sets realistic expectations and provides context for variation.

**Verification**: Verify Mid-360 specifications against published datasheet.

---

## Files Reviewed

| File | Path | Status |
|------|------|--------|
| content.md | `chapters\chapter-3\content.md` | Reviewed |
| PRODUCTION_SPEC.md | `chapters\chapter-3\PRODUCTION_SPEC.md` | Reviewed |
| research_notes.md | `chapters\chapter-3\research_notes.md` | Reviewed |
| Lab 3.1 README.md | `lab\lab-3-1-lidar-point-cloud\README.md` | Reviewed |
| Lab 3.1 starter.py | `lab\lab-3-1-lidar-point-cloud\starter.py` | Reviewed |
| Lab 3.1 solution.py | `lab\lab-3-1-lidar-point-cloud\solution.py` | Reviewed |
| Lab 3.1 Dockerfile.lab | `lab\lab-3-1-lidar-point-cloud\Dockerfile.lab` | Reviewed |
| Lab 3.1 package.xml | `lab\lab-3-1-lidar-point-cloud\package.xml` | Reviewed |
| Lab 3.2 README.md | `lab\lab-3-2-rgbd-calibration\README.md` | Reviewed |
| Lab 3.2 starter.py | `lab\lab-3-2-rgbd-calibration\starter.py` | Reviewed |
| Lab 3.2 solution.py | `lab\lab-3-2-rgbd-calibration\solution.py` | Reviewed |
| Lab 3.2 Dockerfile.lab | `lab\lab-3-2-rgbd-calibration\Dockerfile.lab` | Reviewed |
| Lab 3.3 README.md | `lab\lab-3-3-imu-filter\README.md` | Reviewed |
| mcq.json | `assessments\mcq.json` | Reviewed |
| short_answer.json | `assessments\short_answer.json` | Reviewed |
| coding_challenge.json | `assessments\coding_challenge.json` | Reviewed |
| rubric.json | `assessments\rubric.json` | Reviewed |
| variants/ | `variants\` | Not present |
| localization/ | `localization\` | Not present |

---

## Recommendations

1. **Before Publishing**: Address all High severity issues (T1, V1, V2, A1) to ensure functional code examples
2. **Before Teaching**: Verify all code examples run in the Docker environment
3. **Before Next Iteration**: Add simulation modes for labs that require physical hardware
4. **Documentation**: Add troubleshooting section to each lab README with common error messages and solutions

---

**Report Generated**: 2026-01-04
**Next Review**: After fixes are applied
