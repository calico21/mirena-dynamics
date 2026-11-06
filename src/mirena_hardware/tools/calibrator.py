#!/usr/bin/env python3

import sys
import os
import math
import yaml
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# The pip wheel of OpenCV ships its own Qt build and points Qt at it on import.
# That plugin set is incompatible with the system PyQt5 used below ("Could not
# load the Qt platform plugin xcb"), so drop the hints before PyQt5 is imported.
os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
os.environ.pop('QT_QPA_FONTDIR', None)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QTabWidget, QLabel, QPushButton,
                             QDoubleSpinBox, QSpinBox, QFormLayout, QMessageBox,
                             QGroupBox, QComboBox, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

# --- CONFIGURATION ---
DEFAULT_CAMERA_YAML = 'camera_info.yaml'
DEFAULT_EXTRINSIC_YAML = 'lidar_camera_transform.yaml'
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
FISHEYE_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# OpenCV 5 moved the fisheye calibration flags out of the cv2.fisheye namespace.
FISHEYE_FLAGS = (getattr(cv2, 'CALIB_RECOMPUTE_EXTRINSIC', 0) |
                 getattr(cv2, 'CALIB_CHECK_COND', 0) |
                 getattr(cv2, 'CALIB_FIX_SKEW', 0))

# calibrateCamera needs a well conditioned homography per view.
MIN_CHARUCO_CORNERS = 6


# --- IMAGE CONVERSION ---
# cv_bridge is skipped on purpose: its compiled extension in /opt/ros is built
# against NumPy 1.x and aborts the process under the installed NumPy 2.x.
_ENCODING_DTYPES = {
    'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1),
    'bgr8': (np.uint8, 3), '8UC3': (np.uint8, 3), 'rgb8': (np.uint8, 3),
    'bgra8': (np.uint8, 4), '8UC4': (np.uint8, 4), 'rgba8': (np.uint8, 4),
    'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1),
}


def imgmsg_to_bgr(msg):
    """Convert a sensor_msgs/Image into a contiguous BGR8 numpy array."""
    encoding = msg.encoding.lower()
    if encoding in ('yuv422', 'yuv422_yuy2', 'yuyv'):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
        code = cv2.COLOR_YUV2BGR_UYVY if encoding == 'yuv422' else cv2.COLOR_YUV2BGR_YUY2
        return cv2.cvtColor(raw, code)

    if encoding not in _ENCODING_DTYPES:
        raise ValueError(f"Unsupported image encoding '{msg.encoding}'")

    dtype, channels = _ENCODING_DTYPES[encoding]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')

    img = np.frombuffer(msg.data, dtype=dtype)
    # step may include row padding, so index by row stride and then crop.
    img = img.reshape(msg.height, msg.step // dtype.itemsize)[:, :msg.width * channels]
    img = img.reshape(msg.height, msg.width, channels)

    if dtype.itemsize == 2:
        img = cv2.convertScaleAbs(img, alpha=255.0 / 65535.0)

    if channels == 1:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if channels == 4:
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if encoding.startswith('bgra') or encoding == '8UC4'
                            else cv2.COLOR_RGBA2BGR)
    if encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(img)


# --- ROS 2 NODE ---
class CalibrationRosNode(Node):
    def __init__(self):
        super().__init__('calibration_backend_node')
        self.latest_image = None
        self.latest_pc_msg = None
        self.img_sub = None
        self.lidar_sub = None

    def update_camera_sub(self, topic_name):
        if self.img_sub is not None:
            self.destroy_subscription(self.img_sub)
        self.latest_image = None
        self.img_sub = self.create_subscription(Image, topic_name, self.image_cb,
                                                qos_profile_sensor_data)

    def update_lidar_sub(self, topic_name):
        if self.lidar_sub is not None:
            self.destroy_subscription(self.lidar_sub)
        self.latest_pc_msg = None
        self.lidar_sub = self.create_subscription(PointCloud2, topic_name, self.lidar_cb,
                                                  qos_profile_sensor_data)

    def image_cb(self, msg):
        try:
            self.latest_image = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

    def lidar_cb(self, msg):
        self.latest_pc_msg = msg


# --- MAIN GUI APPLICATION ---
class CalibrationTool(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("Unified ROS 2 Lidar-Camera Calibrator")
        self.resize(1300, 850)

        # Global State Variables
        self.latest_cv_image = None
        self.latest_pc_msg = None
        self.mtx = None
        self.dist = None
        self.is_fisheye = False
        
        # Calibration Buffers
        self.objpoints = [] 
        self.imgpoints = []
        self.all_charuco_corners = []
        self.all_charuco_ids = []
        self.captured_count = 0

        # Pattern State Objects
        self.board = None
        self.aruco_dict = None
        self.detector = None

        self.init_ui()
        self.apply_pattern_settings()
        self.attempt_load_camera_yaml(DEFAULT_CAMERA_YAML)
        self.refresh_ros_topics() 

        # Timers
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(33)

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.node.latest_image is not None:
            self.latest_cv_image = self.node.latest_image
        if self.node.latest_pc_msg is not None:
            self.latest_pc_msg = self.node.latest_pc_msg

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # --- TOP BAR: Topic Selection ---
        topic_group = QGroupBox("ROS 2 Network")
        topic_layout = QHBoxLayout()
        
        topic_layout.addWidget(QLabel("Camera:"))
        self.cb_camera = QComboBox()
        self.cb_camera.currentTextChanged.connect(self.on_camera_topic_changed)
        topic_layout.addWidget(self.cb_camera, stretch=1)

        topic_layout.addWidget(QLabel("LiDAR:"))
        self.cb_lidar = QComboBox()
        self.cb_lidar.currentTextChanged.connect(self.on_lidar_topic_changed)
        topic_layout.addWidget(self.cb_lidar, stretch=1)

        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self.refresh_ros_topics)
        topic_layout.addWidget(btn_refresh)

        topic_group.setLayout(topic_layout)
        main_layout.addWidget(topic_group)

        # --- TABS ---
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self.tab_intrinsic = QWidget()
        self.init_intrinsic_tab()
        self.tabs.addTab(self.tab_intrinsic, "1. Camera Intrinsics")

        self.tab_extrinsic = QWidget()
        self.init_extrinsic_tab()
        self.tabs.addTab(self.tab_extrinsic, "2. LiDAR-Camera Extrinsics")

    def init_intrinsic_tab(self):
        layout = QHBoxLayout()
        
        # Left Image Canvas
        self.int_image_label = QLabel("Waiting for Camera Feed...")
        self.int_image_label.setMinimumSize(800, 600)
        self.int_image_label.setAlignment(Qt.AlignCenter)
        self.int_image_label.setStyleSheet("background-color: black; color: white;")
        layout.addWidget(self.int_image_label, stretch=3)

        # Right Control Panel
        ctrl_layout = QVBoxLayout()
        
        # --- PATTERN SETTINGS ---
        settings_group = QGroupBox("Configuration")
        form_layout = QFormLayout()

        self.cb_camera_model = QComboBox()
        self.cb_camera_model.addItems(["Standard (Pinhole)", "Fisheye"])
        form_layout.addRow("Lens Model:", self.cb_camera_model)

        self.cb_pattern_type = QComboBox()
        self.cb_pattern_type.addItems(["Chessboard", "ChArUco"])
        self.cb_pattern_type.currentTextChanged.connect(self.toggle_pattern_ui)
        form_layout.addRow("Pattern Type:", self.cb_pattern_type)

        self.sb_cols = QSpinBox()
        self.sb_cols.setRange(3, 30)
        self.sb_cols.setValue(8)
        self.lbl_cols = QLabel("Columns (Inner):")
        form_layout.addRow(self.lbl_cols, self.sb_cols)

        self.sb_rows = QSpinBox()
        self.sb_rows.setRange(3, 30)
        self.sb_rows.setValue(6)
        self.lbl_rows = QLabel("Rows (Inner):")
        form_layout.addRow(self.lbl_rows, self.sb_rows)

        self.sb_square = QDoubleSpinBox()
        self.sb_square.setRange(0.001, 1.0)
        self.sb_square.setDecimals(3)
        self.sb_square.setSingleStep(0.005)
        self.sb_square.setValue(0.025)
        form_layout.addRow("Square Size (m):", self.sb_square)

        # ChArUco Specific settings
        self.sb_marker = QDoubleSpinBox()
        self.sb_marker.setRange(0.001, 1.0)
        self.sb_marker.setDecimals(3)
        self.sb_marker.setSingleStep(0.005)
        self.sb_marker.setValue(0.018)
        self.lbl_marker = QLabel("Marker Size (m):")
        form_layout.addRow(self.lbl_marker, self.sb_marker)

        self.cb_dict = QComboBox()
        self.cb_dict.addItems(list(ARUCO_DICTS.keys()))
        self.cb_dict.setCurrentText("DICT_5X5_100")
        self.lbl_dict = QLabel("Dictionary:")
        form_layout.addRow(self.lbl_dict, self.cb_dict)

        btn_apply_pattern = QPushButton("Apply & Reset Buffers")
        btn_apply_pattern.clicked.connect(self.apply_pattern_settings)
        form_layout.addRow(btn_apply_pattern)

        settings_group.setLayout(form_layout)
        ctrl_layout.addWidget(settings_group)
        self.toggle_pattern_ui(self.cb_pattern_type.currentText())

        # --- CAPTURE & CALIBRATE ---
        action_group = QGroupBox("Calibration Pipeline")
        action_layout = QVBoxLayout()

        self.lbl_cap_count = QLabel("Captured Frames: 0")
        self.lbl_cap_count.setStyleSheet("font-size: 14px; font-weight: bold;")
        
        self.lbl_reproj_error = QLabel("Reprojection Error: N/A")
        self.lbl_reproj_error.setStyleSheet("font-size: 14px;")
        
        btn_capture = QPushButton("Capture Frame")
        btn_capture.setMinimumHeight(40)
        btn_capture.clicked.connect(self.capture_frame)
        
        btn_calc = QPushButton("Calculate & Save YAML")
        btn_calc.setMinimumHeight(40)
        btn_calc.clicked.connect(self.calculate_intrinsics)

        btn_reset = QPushButton("Reset Data")
        btn_reset.clicked.connect(self.reset_intrinsics)

        action_layout.addWidget(self.lbl_cap_count)
        action_layout.addWidget(self.lbl_reproj_error)
        action_layout.addWidget(btn_capture)
        action_layout.addWidget(btn_calc)
        action_layout.addWidget(btn_reset)
        action_group.setLayout(action_layout)

        ctrl_layout.addWidget(action_group)
        ctrl_layout.addStretch()
        
        layout.addLayout(ctrl_layout, stretch=1)
        self.tab_intrinsic.setLayout(layout)

    def init_extrinsic_tab(self):
        layout = QHBoxLayout()
        
        self.ext_image_label = QLabel("Waiting for Camera & LiDAR Data...")
        self.ext_image_label.setMinimumSize(800, 600)
        self.ext_image_label.setAlignment(Qt.AlignCenter)
        self.ext_image_label.setStyleSheet("background-color: black; color: white;")
        layout.addWidget(self.ext_image_label, stretch=3)

        ctrl_layout = QVBoxLayout()
        
        self.lbl_cam_status = QLabel("Camera Calib: None Loaded")
        self.lbl_cam_status.setStyleSheet("color: red; font-weight: bold;")
        ctrl_layout.addWidget(self.lbl_cam_status)

        group_box = QGroupBox("Extrinsic Fine-Tuning")
        form_layout = QFormLayout()

        self.spinboxes = {}
        ranges = {'X (m)': (-2.0, 2.0, 0.01), 'Y (m)': (-2.0, 2.0, 0.01), 'Z (m)': (-2.0, 2.0, 0.01),
                  'Roll (deg)': (-180, 180, 0.5), 'Pitch (deg)': (-180, 180, 0.5), 'Yaw (deg)': (-180, 180, 0.5)}

        for name, (vmin, vmax, step) in ranges.items():
            sb = QDoubleSpinBox()
            sb.setRange(vmin, vmax)
            sb.setSingleStep(step)
            sb.setDecimals(3)
            sb.setValue(0.0)
            form_layout.addRow(name, sb)
            self.spinboxes[name] = sb

        group_box.setLayout(form_layout)
        
        btn_save_yaml = QPushButton("Export Transform YAML")
        btn_save_yaml.clicked.connect(self.save_extrinsics_yaml)

        ctrl_layout.addWidget(group_box)
        ctrl_layout.addWidget(btn_save_yaml)
        ctrl_layout.addStretch()

        layout.addLayout(ctrl_layout, stretch=1)
        self.tab_extrinsic.setLayout(layout)

    # --- TOPIC MANAGEMENT ---
    def refresh_ros_topics(self):
        topic_names_and_types = self.node.get_topic_names_and_types()
        img_topics = [name for name, types in topic_names_and_types if 'sensor_msgs/msg/Image' in types]
        pc_topics = [name for name, types in topic_names_and_types if 'sensor_msgs/msg/PointCloud2' in types]
                
        self.cb_camera.blockSignals(True)
        self.cb_lidar.blockSignals(True)
        self.cb_camera.clear()
        self.cb_lidar.clear()
        self.cb_camera.addItems(sorted(img_topics))
        self.cb_lidar.addItems(sorted(pc_topics))
        self.cb_camera.blockSignals(False)
        self.cb_lidar.blockSignals(False)

        if img_topics: self.on_camera_topic_changed(self.cb_camera.currentText())
        if pc_topics: self.on_lidar_topic_changed(self.cb_lidar.currentText())

    def on_camera_topic_changed(self, text):
        if text: self.node.update_camera_sub(text)

    def on_lidar_topic_changed(self, text):
        if text: self.node.update_lidar_sub(text)

    # --- PATTERN CONFIGURATION LOGIC ---
    def toggle_pattern_ui(self, pattern_type):
        is_charuco = (pattern_type == "ChArUco")
        self.lbl_marker.setVisible(is_charuco)
        self.sb_marker.setVisible(is_charuco)
        self.lbl_dict.setVisible(is_charuco)
        self.cb_dict.setVisible(is_charuco)
        # A ChArUco board is defined by its square count, a chessboard by its inner corners.
        self.lbl_cols.setText("Columns (Squares):" if is_charuco else "Columns (Inner):")
        self.lbl_rows.setText("Rows (Squares):" if is_charuco else "Rows (Inner):")

    def apply_pattern_settings(self):
        self.reset_intrinsics() 
        
        ptype = self.cb_pattern_type.currentText()
        cols = self.sb_cols.value()
        rows = self.sb_rows.value()
        square = self.sb_square.value()

        if ptype == "Chessboard":
            self.board = np.zeros((cols * rows, 3), np.float32)
            self.board[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
        
        elif ptype == "ChArUco":
            marker = self.sb_marker.value()
            if marker >= square:
                QMessageBox.warning(self, "Invalid Setting", "Marker Size MUST be smaller than Square Size.")
                return

            dict_id = ARUCO_DICTS[self.cb_dict.currentText()]
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

            self.board = cv2.aruco.CharucoBoard((cols, rows), square, marker, self.aruco_dict)
            # OpenCV 5 dropped interpolateCornersCharuco(); CharucoDetector replaces it
            # and returns the marker detections in the same call.
            self.detector = cv2.aruco.CharucoDetector(self.board)

        self.node.get_logger().info(f"Applied settings for {ptype}: {cols}x{rows}")

    def detect_charuco(self, gray):
        """Run the ChArUco detector, normalising the output to (N, 1, ...) shapes.

        OpenCV 5 returns flat (N, 2) corners / (N,) ids, which the drawing helpers
        reject, so reshape once here for every consumer.
        """
        charuco_corners, charuco_ids, corners, ids = self.detector.detectBoard(gray)
        if charuco_corners is not None and len(charuco_corners) > 0:
            charuco_corners = charuco_corners.reshape(-1, 1, 2)
            charuco_ids = charuco_ids.reshape(-1, 1)
        else:
            charuco_corners, charuco_ids = None, None
        return charuco_corners, charuco_ids, corners, ids

    # --- UI UPDATE LOOP ---
    def update_ui(self):
        if self.latest_cv_image is None: return

        current_tab = self.tabs.currentIndex()
        frame = self.latest_cv_image.copy()

        if current_tab == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ptype = self.cb_pattern_type.currentText()

            if ptype == "Chessboard":
                ret, corners = cv2.findChessboardCorners(gray, (self.sb_cols.value(), self.sb_rows.value()), None)
                if ret: cv2.drawChessboardCorners(frame, (self.sb_cols.value(), self.sb_rows.value()), corners, ret)
            
            elif ptype == "ChArUco" and self.detector is not None:
                charuco_corners, charuco_ids, corners, ids = self.detect_charuco(gray)
                if ids is not None and len(ids) > 0:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                if charuco_corners is not None and len(charuco_corners) > 0:
                    cv2.aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids, (0, 255, 0))

            self.display_image(frame, self.int_image_label)
            
        elif current_tab == 1:
            if self.mtx is not None and self.latest_pc_msg is not None:
                frame = self.project_lidar(frame)
            self.display_image(frame, self.ext_image_label)


    def display_image(self, cv_img, label):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        # tobytes() hands Qt a buffer it owns, so the numpy array can be freed freely.
        q_img = QImage(rgb_image.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888)
        label.setPixmap(QPixmap.fromImage(q_img).scaled(label.width(), label.height(), Qt.KeepAspectRatio))

    # --- INTRINSIC CAPTURE & CALIBRATION ---
    def capture_frame(self):
        if self.latest_cv_image is None: return
        gray = cv2.cvtColor(self.latest_cv_image, cv2.COLOR_BGR2GRAY)
        ptype = self.cb_pattern_type.currentText()

        if ptype == "Chessboard":
            ret, corners = cv2.findChessboardCorners(gray, (self.sb_cols.value(), self.sb_rows.value()), None)
            if ret:
                self.objpoints.append(self.board)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
                self.imgpoints.append(corners2)
                self.increment_capture()
            else:
                QMessageBox.warning(self, "Capture Failed", "Chessboard not completely visible.")

        elif ptype == "ChArUco":
            if self.detector is None:
                QMessageBox.warning(self, "Capture Failed", "Apply the ChArUco settings first.")
                return
            charuco_corners, charuco_ids, _, ids = self.detect_charuco(gray)
            if ids is None or len(ids) == 0:
                QMessageBox.warning(self, "Capture Failed", "No ArUco markers found.")
            elif charuco_corners is not None and len(charuco_corners) >= MIN_CHARUCO_CORNERS:
                self.all_charuco_corners.append(charuco_corners)
                self.all_charuco_ids.append(charuco_ids)
                self.increment_capture()
            else:
                QMessageBox.warning(self, "Capture Failed",
                                    f"Not enough ChArUco corners detected (need at least {MIN_CHARUCO_CORNERS}).")

    def increment_capture(self):
        self.captured_count += 1
        self.lbl_cap_count.setText(f"Captured Frames: {self.captured_count}")

    def charuco_point_pairs(self):
        """Yield (object_points, image_points) per captured ChArUco view."""
        for corners, ids in zip(self.all_charuco_corners, self.all_charuco_ids):
            objp, imgp = self.board.matchImagePoints(corners, ids)
            if objp is None or len(objp) < MIN_CHARUCO_CORNERS:
                continue
            yield objp.astype(np.float32), imgp.astype(np.float32)

    def calculate_intrinsics(self):
        if self.captured_count < 10:
            QMessageBox.warning(self, "Need Data", "Capture at least 10 frames for mathematical stability.")
            return

        img_h, img_w = self.latest_cv_image.shape[:2]
        gray_shape = (img_w, img_h)
        ptype = self.cb_pattern_type.currentText()
        is_fisheye_calc = (self.cb_camera_model.currentText() == "Fisheye")

        self.node.get_logger().info("Calculating calibration... please wait.")

        try:
            if is_fisheye_calc:
                # OpenCV 5's fisheye solver only accepts the row-vector layout:
                # objpoints (1, N, 3), imgpoints (1, N, 2). The (N, 1, ...) layout
                # that calibrateCamera() takes fails its internal size check.
                fs_objpoints = []
                fs_imgpoints = []

                if ptype == "Chessboard":
                    fs_objpoints = [p.reshape(1, -1, 3).astype(np.float64) for p in self.objpoints]
                    fs_imgpoints = [p.reshape(1, -1, 2).astype(np.float64) for p in self.imgpoints]
                elif ptype == "ChArUco":
                    for objp, imgp in self.charuco_point_pairs():
                        fs_objpoints.append(objp.reshape(1, -1, 3).astype(np.float64))
                        fs_imgpoints.append(imgp.reshape(1, -1, 2).astype(np.float64))

                K = np.zeros((3, 3))
                D = np.zeros((4, 1))
                ret, mtx, dist, _, _ = cv2.fisheye.calibrate(
                    fs_objpoints, fs_imgpoints, gray_shape, K, D,
                    flags=FISHEYE_FLAGS, criteria=FISHEYE_CRITERIA
                )
                self.is_fisheye = True

            else:
                # Standard Pinhole Logic
                if ptype == "Chessboard":
                    objpoints, imgpoints = self.objpoints, self.imgpoints
                elif ptype == "ChArUco":
                    # OpenCV 5 dropped calibrateCameraCharuco(); the board itself maps
                    # detected corners to their 3D model points for calibrateCamera().
                    pairs = list(self.charuco_point_pairs())
                    objpoints = [p[0] for p in pairs]
                    imgpoints = [p[1] for p in pairs]

                ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)
                self.is_fisheye = False

        except Exception as e:
            QMessageBox.critical(self, "Math Error", f"Calculation crashed:\n{e}")
            return

        if ret is not None:
            self.mtx = mtx
            self.dist = dist

            color = "green" if ret < 0.5 else "orange" if ret < 1.0 else "red"
            self.lbl_reproj_error.setText(f"Reprojection Error: {ret:.4f} px")
            self.lbl_reproj_error.setStyleSheet(f"font-size: 14px; color: {color}; font-weight: bold;")

            distortion_model = "equidistant" if self.is_fisheye else "plumb_bob"
            dist_list = dist.flatten().tolist()

            yaml_data = {
                "image_width": img_w,
                "image_height": img_h,
                "camera_name": "mirena_camera",
                "camera_matrix": { "rows": 3, "cols": 3, "data": mtx.flatten().tolist() },
                "distortion_model": distortion_model,
                "distortion_coefficients": { "rows": 1, "cols": len(dist_list), "data": dist_list },
                "rectification_matrix": { "rows": 3, "cols": 3, "data": np.eye(3).flatten().tolist() },
                "projection_matrix": { "rows": 3, "cols": 4, "data": np.hstack((mtx, np.zeros((3, 1)))).flatten().tolist() }
            }

            with open(DEFAULT_CAMERA_YAML, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)
            
            self.lbl_cam_status.setText(f"Loaded: {DEFAULT_CAMERA_YAML} ({distortion_model})")
            self.lbl_cam_status.setStyleSheet("color: green; font-weight: bold;")
            QMessageBox.information(self, "Success", f"Calibration saved!\n\nError: {ret:.4f} px\nSaved to: {DEFAULT_CAMERA_YAML}")
        else:
            QMessageBox.critical(self, "Failed", "Calibration algorithm failed to converge.")

    def reset_intrinsics(self):
        self.objpoints.clear()
        self.imgpoints.clear()
        self.all_charuco_corners.clear()
        self.all_charuco_ids.clear()
        self.captured_count = 0
        self.lbl_cap_count.setText("Captured Frames: 0")
        self.lbl_reproj_error.setText("Reprojection Error: N/A")
        self.lbl_reproj_error.setStyleSheet("font-size: 14px;")

    def attempt_load_camera_yaml(self, filepath):
        if not os.path.exists(filepath): return
        try:
            with open(filepath, 'r') as f:
                calib = yaml.safe_load(f)
            self.mtx = np.array(calib['camera_matrix']['data'], dtype=np.float64).reshape((3, 3))
            self.dist = np.array(calib['distortion_coefficients']['data'], dtype=np.float64)
            self.is_fisheye = (calib.get('distortion_model', '') == 'equidistant')
            model_str = "equidistant" if self.is_fisheye else "plumb_bob"
            
            self.lbl_cam_status.setText(f"Loaded: {filepath} ({model_str})")
            self.lbl_cam_status.setStyleSheet("color: green; font-weight: bold;")
        except Exception:
            pass

    # --- EXTRINSIC LOGIC ---
    def project_lidar(self, frame):
        x = self.spinboxes['X (m)'].value()
        y = self.spinboxes['Y (m)'].value()
        z = self.spinboxes['Z (m)'].value()
        roll = math.radians(self.spinboxes['Roll (deg)'].value())
        pitch = math.radians(self.spinboxes['Pitch (deg)'].value())
        yaw = math.radians(self.spinboxes['Yaw (deg)'].value())

        # The GUI now natively represents the transform from Lidar -> Standard Camera Frame
        t_cam = np.array([x, y, z], dtype=np.float32)
        R_cam = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix().astype(np.float32)

        cloud_data = pc2.read_points(self.latest_pc_msg, field_names=("x", "y", "z"), skip_nans=True)
        pts_3d = np.column_stack((cloud_data['x'], cloud_data['y'], cloud_data['z'])).astype(np.float32) if isinstance(cloud_data, np.ndarray) else np.array(list(cloud_data), dtype=np.float32)
        
        if len(pts_3d) == 0: return frame

        # 1. Transform Lidar points into the Standard Camera Frame (X-forward, Y-left, Z-up)
        pts_cam = (R_cam @ pts_3d.T).T + t_cam

        # 2. Filter points strictly behind the camera (X < 0.1 in standard camera frame)
        valid_front_mask = pts_cam[:, 0] > 0.1
        pts_cam = pts_cam[valid_front_mask]
        
        if len(pts_cam) == 0: return frame

        distances = np.linalg.norm(pts_cam, axis=1)

        # 3. Convert from Standard Camera Frame to OpenCV Optical Frame (Z-forward, X-right, Y-down)
        # X_opt = -Y_cam | Y_opt = -Z_cam | Z_opt = X_cam
        pts_opt = np.column_stack((-pts_cam[:, 1], -pts_cam[:, 2], pts_cam[:, 0]))

        # Since points are already fully transformed into the camera's optical frame,
        # we pass zero vectors for OpenCV's extrinsics
        rvec_zero = np.zeros((3, 1), dtype=np.float64)
        tvec_zero = np.zeros((3, 1), dtype=np.float64)

        if self.is_fisheye:
            pts_opt_fs = pts_opt.reshape(1, -1, 3).astype(np.float64)
            dist_fs = np.asarray(self.dist, dtype=np.float64).reshape(4, 1)
            pts_2d, _ = cv2.fisheye.projectPoints(pts_opt_fs, rvec_zero, tvec_zero, self.mtx, dist_fs)
        else:
            pts_2d, _ = cv2.projectPoints(pts_opt.astype(np.float64), rvec_zero, tvec_zero,
                                          self.mtx, np.asarray(self.dist, dtype=np.float64).reshape(1, -1))
        
        pts_2d = pts_2d.reshape(-1, 2)

        valid_mask = np.isfinite(pts_2d).all(axis=1)
        pts_2d, distances = pts_2d[valid_mask], distances[valid_mask]

        img_h, img_w = frame.shape[:2]
        overlay = frame.copy()
        for i in range(len(pts_2d)):
            u, v = int(pts_2d[i][0]), int(pts_2d[i][1])
            if 0 <= u < img_w and 0 <= v < img_h:
                color_intensity = min(255, int((distances[i] / 15.0) * 255))
                cv2.circle(frame, (u, v), 2, (255 - color_intensity, 0, color_intensity), -1)

        opacity = 0.3 
        cv2.addWeighted(overlay, opacity, frame, 1.0 - opacity, 0, dst=frame)
        return frame

    def save_extrinsics_yaml(self):
        if self.mtx is None: 
            return QMessageBox.warning(self, "Error", "Load Camera Intrinsics first.")
        
        x = self.spinboxes['X (m)'].value()
        y = self.spinboxes['Y (m)'].value()
        z = self.spinboxes['Z (m)'].value()
        r_deg = self.spinboxes['Roll (deg)'].value()
        p_deg = self.spinboxes['Pitch (deg)'].value()
        y_deg = self.spinboxes['Yaw (deg)'].value()

        roll, pitch, yaw = math.radians(r_deg), math.radians(p_deg), math.radians(y_deg)
        
        # 1. Transform from Lidar to Standard Camera Frame (from GUI)
        quat_lidar_to_cam = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        # 2. Transform from Standard Camera Frame to Optical Frame (Static)
        # Represents Roll = -90, Pitch = 0, Yaw = -90 mapping
        quat_cam_to_optical = [-0.5, 0.5, -0.5, 0.5] 

        # Build the exact dictionary structure with both transforms
        yaml_data = {
            "transforms": [
                {
                    "parent_frame_id": "car/lidar",           
                    "child_frame_id": "car/camera",
                    "translation": [-float(x), -float(y), -float(z)],
                    "rotation": quat_lidar_to_cam.tolist()
                },
                {
                    "parent_frame_id": "car/camera",
                    "child_frame_id": "camera_optical_frame",
                    "translation": [0.0, 0.0, 0.0],
                    "rotation": quat_cam_to_optical
                }
            ]
        }

        with open(DEFAULT_EXTRINSIC_YAML, 'w') as f:
            # Setting default_flow_style=None allows PyYAML to format lists nicely 
            yaml.dump(yaml_data, f, default_flow_style=None, sort_keys=False)
            
        QMessageBox.information(self, "Saved", f"Extrinsics exported to:\n{DEFAULT_EXTRINSIC_YAML}")


def main():
    rclpy.init()
    ros_node = CalibrationRosNode()
    app = QApplication(sys.argv)
    gui = CalibrationTool(ros_node)
    gui.show()
    exit_code = app.exec_()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()