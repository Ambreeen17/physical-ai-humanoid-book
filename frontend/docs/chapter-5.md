---
sidebar_position: 6
title: "Chapter 5: Computer Vision"
---

# Chapter 5: Computer Vision for Robotics

<PersonalizationToggle chapterId="5" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Apply camera calibration** to correct lens distortion and obtain metric measurements
2. **Implement feature detection and matching** using ORB, SIFT, and deep learning methods
3. **Perform depth estimation** from stereo cameras and RGB-D sensors
4. **Deploy object detection models** (YOLO, Faster R-CNN) on robotic platforms
5. **Build a complete perception pipeline** integrating detection, tracking, and pose estimation

---

## Introduction

Imagine a robot trying to pick up a coffee mug from a cluttered table. It needs to answer several questions instantly: *Where is the mug? What angle is the handle at? How far away is it? Are there obstacles between the gripper and the mug?*

These questions all require **computer vision**—the ability to extract meaningful information from visual sensors. For robots, vision isn't just about "seeing"—it's about building a 3D understanding of the world that enables physical interaction.

In this chapter, you'll learn how robots transform raw pixel data into actionable intelligence. From camera geometry to deep learning-based object detection, you'll build the visual perception stack that enables robots to understand and interact with their environment.

---

## Section 5.1: Camera Models and Calibration

### The Pinhole Camera Model

The **pinhole camera model** describes how 3D world points project onto a 2D image:

```
┌─────────────────────────────────────────────────────────────┐
│                           World                              │
│                           Point                              │
│                           (X,Y,Z)                            │
│                              │                               │
│                              │                               │
│                              ▼                               │
│    ┌─────────────────────────┼─────────────────────────────┐│
│    │         Camera          │          Lens                ││
│    │         Center ─────────┼─────────────────────────────►││
│    │           O             │                              ││
│    └─────────────────────────┼─────────────────────────────┘│
│                              │                               │
│                              ▼                               │
│                         Image Point                          │
│                           (u,v)                              │
└─────────────────────────────────────────────────────────────┘
```

**Projection Equations:**
```
u = fx * (X/Z) + cx
v = fy * (Y/Z) + cy
```

Where:
- `(X, Y, Z)`: 3D point in camera coordinates
- `(u, v)`: 2D pixel coordinates
- `fx, fy`: Focal lengths in pixels
- `(cx, cy)`: Principal point (image center)

### The Camera Matrix

These parameters form the **intrinsic matrix K**:

```python
import numpy as np

# Camera intrinsic matrix
K = np.array([
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1]
])

# Project 3D point to 2D
def project_point(P_world, K):
    """
    P_world: 3D point [X, Y, Z]
    Returns: 2D pixel [u, v]
    """
    p_homogeneous = K @ P_world
    u = p_homogeneous[0] / p_homogeneous[2]
    v = p_homogeneous[1] / p_homogeneous[2]
    return np.array([u, v])
```

### Lens Distortion

Real lenses introduce distortion:

- **Radial distortion**: Lines curve toward (barrel) or away from (pincushion) the center
- **Tangential distortion**: Image plane not perfectly parallel to lens

**Distortion Correction:**
```python
import cv2

def undistort_image(image, K, dist_coeffs):
    """
    K: Camera intrinsic matrix
    dist_coeffs: [k1, k2, p1, p2, k3] distortion coefficients
    """
    h, w = image.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, K, dist_coeffs, None, new_K)
    return undistorted, new_K
```

### Camera Calibration with OpenCV

```python
import cv2
import numpy as np
import glob

def calibrate_camera(checkerboard_images_path, pattern_size=(9, 6), square_size=0.025):
    """
    Calibrate camera using checkerboard images.
    """
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []
    images = glob.glob(checkerboard_images_path)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            objpoints.append(objp)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

    ret, K, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print(f"Calibration RMS error: {ret:.4f} pixels")
    return K, dist_coeffs, rvecs, tvecs
```

:::tip Real-World Practice
For robotics applications:
- Take 15-20 calibration images from different angles
- Include images with the checkerboard at different distances
- RMS reprojection error should be < 0.5 pixels for precision work
:::

---

## Section 5.2: Feature Detection and Matching

### Classical Feature Detectors

**ORB (Oriented FAST and Rotated BRIEF):**
```python
def detect_and_match_orb(img1, img2, n_features=500):
    """
    Detect ORB features and match between two images.
    """
    orb = cv2.ORB_create(nfeatures=n_features)
    kp1, desc1 = orb.detectAndCompute(img1, None)
    kp2, desc2 = orb.detectAndCompute(img2, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)

    return kp1, kp2, matches
```

### Feature Matching with RANSAC

```python
def match_with_ransac(kp1, kp2, matches, ransac_threshold=5.0):
    """
    Match features with RANSAC outlier rejection.
    """
    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

    H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, ransac_threshold)
    inliers = mask.ravel() == 1
    n_inliers = np.sum(inliers)

    print(f"Found {n_inliers}/{len(matches)} inliers ({100*n_inliers/len(matches):.1f}%)")
    return H, inliers
```

---

## Section 5.3: Depth Perception

### Stereo Vision

```python
def compute_stereo_depth(img_left, img_right, focal_length, baseline):
    """
    Compute depth map from stereo images.
    """
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,
        blockSize=5,
        P1=8 * 3 * 5**2,
        P2=32 * 3 * 5**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )

    disparity = stereo.compute(img_left, img_right).astype(np.float32) / 16.0
    depth_map = np.where(disparity > 0, (focal_length * baseline) / disparity, 0)
    return depth_map
```

### RGB-D Cameras with Intel RealSense

```python
import pyrealsense2 as rs
import numpy as np

class RealSenseCamera:
    """Intel RealSense D435 interface for robotics."""

    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(config)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.align = rs.align(rs.stream.color)

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data()) * self.depth_scale
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        return color_image, depth_image, intrinsics

    def close(self):
        self.pipeline.stop()
```

---

## Section 5.4: Object Detection with YOLO

### Real-Time Detection for Robotics

```python
from ultralytics import YOLO
import cv2

class YOLODetector:
    """YOLOv8 object detector for robotics."""

    def __init__(self, model_size='n'):
        self.model = YOLO(f'yolov8{model_size}.pt')

    def detect(self, image, conf_threshold=0.5, classes=None):
        results = self.model(image, conf=conf_threshold, classes=classes, verbose=False)

        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                cls_id = int(box.cls[0].cpu().numpy())
                cls_name = result.names[cls_id]

                detections.append({
                    'bbox': xyxy,
                    'class': cls_name,
                    'confidence': float(conf)
                })

        return detections

    def detect_and_draw(self, image, conf_threshold=0.5):
        detections = self.detect(image, conf_threshold)

        for det in detections:
            x1, y1, x2, y2 = det['bbox'].astype(int)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det['class']}: {det['confidence']:.2f}"
            cv2.putText(image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image, detections
```

### 3D Object Localization

```python
def detection_to_3d(detection, depth_image, K):
    """
    Convert 2D detection + depth to 3D position.
    """
    x1, y1, x2, y2 = detection['bbox'].astype(int)
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2

    roi = depth_image[cy-5:cy+5, cx-5:cx+5]
    depth = np.median(roi[roi > 0])

    if depth == 0:
        return None

    fx, fy = K[0, 0], K[1, 1]
    px, py = K[0, 2], K[1, 2]

    X = (cx - px) * depth / fx
    Y = (cy - py) * depth / fy
    Z = depth

    return np.array([X, Y, Z])
```

---

## Section 5.5: Pose Estimation with ArUco Markers

```python
class ArUcoPoseEstimator:
    """Estimate object pose using ArUco markers."""

    def __init__(self, marker_size=0.05, dict_type=cv2.aruco.DICT_6X6_250):
        self.marker_size = marker_size
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def estimate_pose(self, image, K, dist_coeffs):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is None:
            return {}

        poses = {}
        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i:i+1], self.marker_size, K, dist_coeffs
            )
            poses[marker_id] = {
                'rvec': rvec[0][0],
                'tvec': tvec[0][0],
                'corners': corners[i][0]
            }

        return poses
```

---

## Section 5.6: ROS 2 Perception Pipeline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D
from cv_bridge import CvBridge
import numpy as np

class PerceptionNode(Node):
    """Complete perception pipeline for robotic manipulation."""

    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        self.detector = YOLODetector(model_size='s')
        self.K = None

        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        self.detection_pub = self.create_publisher(Detection3DArray, '/detections_3d', 10)
        self.color_image = None
        self.depth_image = None

    def info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_frame()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        if self.depth_image.dtype == np.uint16:
            self.depth_image = self.depth_image.astype(np.float32) / 1000.0

    def process_frame(self):
        if self.color_image is None or self.depth_image is None or self.K is None:
            return

        detections = self.detector.detect(self.color_image)
        detection_msg = Detection3DArray()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = 'camera_color_optical_frame'

        for det in detections:
            pos_3d = detection_to_3d(det, self.depth_image, self.K)
            if pos_3d is None:
                continue

            det_3d = Detection3D()
            det_3d.bbox.center.position.x = float(pos_3d[0])
            det_3d.bbox.center.position.y = float(pos_3d[1])
            det_3d.bbox.center.position.z = float(pos_3d[2])
            detection_msg.detections.append(det_3d)

        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Section 5.7: Hardware Considerations

### Intel RealSense D435 for Unitree G1

| Parameter | Value | Notes |
|-----------|-------|-------|
| RGB Resolution | 1920x1080 @ 30fps | Can reduce for speed |
| Depth Resolution | 1280x720 @ 30fps | Optimal for manipulation |
| Depth Range | 0.1m - 10m | Best accuracy 0.3-3m |
| Field of View | 87° x 58° (depth) | Wide for scene understanding |

---

## Summary

### Key Takeaways

1. **Camera calibration** is essential for metric measurements—always calibrate before deployment

2. **Feature detection** enables visual odometry and SLAM; ORB is efficient for real-time applications

3. **Depth sensing** from stereo or RGB-D cameras enables 3D understanding required for manipulation

4. **YOLO provides real-time object detection** suitable for robotics; combine with depth for 3D positions

5. **6-DoF pose estimation** is required for precise grasping; ArUco markers are simple and reliable

### Looking Ahead

In Chapter 6, we'll explore **Control Theory**—how to use perception outputs to generate robot motion commands.

---

## Exercises

1. **Stereo Calibration**: Calibrate a stereo camera pair and compute the essential matrix.

2. **Custom Object Detection**: Fine-tune YOLOv8 on a custom dataset of household objects.

3. **Real-Time Optimization**: Profile your perception pipeline and optimize to achieve 30 FPS.

4. **Point Cloud Generation**: Create colored point clouds from RGB-D data and visualize in RViz.

---

**Chapter completed**: 2026-01-20
**Chapter**: 5 - Computer Vision
