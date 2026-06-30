#!/usr/bin/env python3

import threading
import customtkinter as ctk
from PIL import Image, ImageTk
import cv2

import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CompressedImage as RosImage
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.context import Context

from sensor_msgs.msg import JointState
from tf2_ros import Buffer
from tf2_ros import TransformListener
import pinocchio as pin
import xml.etree.ElementTree as ET

from pyopengltk import OpenGLFrame
from OpenGL.GL import *
from OpenGL.arrays import vbo
from OpenGL.GLU import *

import subprocess
import os
import time

import json
import shlex
from pathlib import Path

image_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
CAMERA_DOMAIN_ID = 10
ROBOT_DOMAIN_ID = 20


class CameraSubscriber(Node):
    def __init__(self, context):
        super().__init__(
            'image_subscriber',
            context=context
        )

        self.lock = threading.Lock()

        self.images = {
            "camera1": None,
            "camera2": None,
            "camera3": None,
        }

        self.create_subscription(
            RosImage,
            '/camera1/image_raw/compressed',
            lambda msg: self.image_callback(msg, "camera1"),
            image_qos
        )

        self.create_subscription(
            RosImage,
            '/camera2/image_raw/compressed',
            lambda msg: self.image_callback(msg, "camera2"),
            image_qos
        )

        self.create_subscription(
            RosImage,
            '/camera3/image_raw/compressed',
            lambda msg: self.image_callback(msg, "camera3"),
            image_qos
        )

    def image_callback(self, msg, cam_name):
        # self.get_logger().info(f"{cam_name} frame received")
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                self.get_logger().warning(f"Failed to decode image: {cam_name}")
                return
            with self.lock:
                self.images[cam_name] = cv_img
        except Exception as e:
            self.get_logger().error(f"{cam_name}: {e}")


class RobotSubscriber(Node):
    def __init__(self, context):
        super().__init__('robot_subscriber', context=context)

        self.lock = threading.Lock()
        self.notify_lock = threading.Lock()

        self.cur_mode = False
        self.cur_mode_version = 0
        self.cur_onoff = False
        self.cur_onoff_version = 0

        self.current_joint_state = None
        self.joint_state_count = 0
        self.urdf_joints = {}
        self.current_name_to_pos = {}

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            qos_profile_sensor_data
        )

        notify_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Bool,
            '/tracer_mode',
            self.notify_Tracer_mode_callback,
            notify_qos
        )

        self.create_subscription(
            Bool,
            '/on_tracer',
            self.notify_on_Tracer_callback,
            notify_qos
        )

        self.lifter_forward_lean_publisher = self.create_publisher(
            Bool,
            '/on_lifter_forward_lean',
            notify_qos
        )

        urdf_path = (
            "/home/seed/ros2/jazzy/src/seed_robot_ros2_pkg/robots/noid_lifter_mover/model/noid_lifter_mover.urdf"
        )
        self.model = pin.buildModelFromUrdf(
            urdf_path
        )
        self.data = self.model.createData()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        tree = ET.parse(urdf_path)
        root = tree.getroot()

        for joint in root.findall("joint"):
            joint_name = joint.attrib["name"]

            origin = joint.find("origin")
            axis = joint.find("axis")
            parent = joint.find("parent")
            child = joint.find("child")

            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]

            if origin is not None:
                xyz = [
                    float(v)
                    for v in origin.attrib.get("xyz", "0 0 0").split()
                ]
                rpy = [
                    float(v)
                    for v in origin.attrib.get("rpy", "0 0 0").split()
                ]

            axis_xyz = [0.0, 0.0, 1.0]

            if axis is not None:
                axis_xyz = [
                    float(v)
                    for v in axis.attrib.get("xyz", "0 0 1").split()
                ]

            self.urdf_joints[joint_name] = {
                "parent_link": parent.attrib["link"] if parent is not None else None,
                "child_link": child.attrib["link"] if child is not None else None,
                "xyz": np.array(xyz, dtype=float),
                "rpy": np.array(rpy, dtype=float),
                "axis": np.array(axis_xyz, dtype=float),
            }

    def joint_callback(self, msg):
        with self.lock:
            self.current_joint_state = msg
            self.current_name_to_pos = {
                name: pos for name, pos in zip(msg.name, msg.position)
            }
            self.joint_state_count += 1

    def notify_Tracer_mode_callback(self, msg):
        with self.notify_lock:
            self.cur_mode = bool(msg.data)
            if self.cur_mode:
                self.cur_mode_version = 1
            else:
                self.cur_mode_version = 0

    def notify_on_Tracer_callback(self, msg):
        with self.notify_lock:
            self.cur_onoff = bool(msg.data)
            if self.cur_onoff:
                self.cur_onoff_version = 1
            else:
                self.cur_onoff_version = 0

    def notify_lifter_forward_lean(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.lifter_forward_lean_publisher.publish(msg)

    def get_link_transform(self, link_name):
        frame_id = self.model.getFrameId(link_name)
        if frame_id == len(self.model.frames):
            return None
        return self.data.oMf[frame_id]

    def compute_leg_display_fk(self):
        with self.lock:
            name_to_pos = dict(self.current_name_to_pos)

        ankle = name_to_pos.get("ankle_joint", 0.0)
        knee = name_to_pos.get("knee_joint", 0.0)

        T_lifter_bottom = self.get_link_transform("lifter_bottom_link")
        if T_lifter_bottom is None:
            return None

        T_leg_shank = (
            T_lifter_bottom
            * self.urdf_joint_transform("ankle_joint", ankle)
        )
        T_leg_knee = (
            T_leg_shank
            * self.urdf_joint_transform("ankle_joint_mimic", -ankle)
        )
        T_leg_thigh = (
            T_leg_knee
            * self.urdf_joint_transform("knee_joint", knee)
        )
        T_leg_base = (
            T_leg_thigh
            * self.urdf_joint_transform("knee_joint_mimic", -knee)
        )

        return {
            "lifter_bottom_link": T_lifter_bottom,
            "leg_shank_link": T_leg_shank,
            "leg_knee_link": T_leg_knee,
            "leg_thigh_link": T_leg_thigh,
            "leg_base_link": T_leg_base,
        }

    def rpy_to_rot(self, rpy):
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        Rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr],
        ])
        Ry = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp],
        ])
        Rz = np.array([
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1],
        ])
        return Rz @ Ry @ Rx

    def axis_angle_to_rot(self, axis, angle):
        axis = np.array(axis, dtype=float)
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            return np.eye(3)
        axis = axis / norm

        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        C = 1.0 - c

        return np.array([
            [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
            [z*x*C - y*s,   z*y*C + x*s, c + z*z*C],
        ])

    def urdf_joint_transform(self, joint_name, q_value):
        info = self.urdf_joints[joint_name]
        T_origin = pin.SE3(
            self.rpy_to_rot(info["rpy"]),
            info["xyz"]
        )
        T_motion = pin.SE3(
            self.axis_angle_to_rot(info["axis"], q_value),
            np.zeros(3)
        )
        return T_origin * T_motion

    def get_tf_position(self, target_frame, source_frame="base_link"):
        try:
            tf = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time()
            )
            t = tf.transform.translation
            return np.array(
                [t.x, t.y, t.z],
                dtype=np.float32
            )
        except Exception:
            return None

    def build_q_from_joint_state(self, js):
        q = pin.neutral(self.model)
        name_to_pos = {
            name: pos
            for name, pos in zip(js.name, js.position)
        }

        for joint_name, pos in name_to_pos.items():
            joint_id = self.model.getJointId(joint_name)
            if joint_id == self.model.njoints:
                continue
            joint = self.model.joints[joint_id]
            if joint.nq == 0:
                continue
            q[joint.idx_q] = pos
        return q

    def compute_fk(self):
        with self.lock:
            js = self.current_joint_state
            count = self.joint_state_count
        if js is None:
            return False
        if count < 5:
            return False
        q = self.build_q_from_joint_state(js)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data


class SkeletonViewer(OpenGLFrame):
    def __init__(self, parent, ros_node):
        self.node = ros_node
        self.gl_ready = False

        super().__init__(parent, width=320, height=540)

        self.point_vbo = None
        self.line_vbo  = None
        self.point_count = 0
        self.line_count  = 0

        self.upper_joint_ids = self.get_upper_joint_ids()

    def initgl(self):
        self.gl_ready = True
        glEnable(GL_DEPTH_TEST)
        self.animate = 1
        glClearColor(
            0.1,
            0.1,
            0.1,
            1.0
        )
        glDepthFunc(GL_LESS)

    def apply_delta(self, T_delta, p):
        return T_delta.rotation @ p + T_delta.translation

    def get_upper_joint_ids(self):
        waist_id = self.node.model.getJointId("waist_y_joint")
        ids = set()
        for joint_id in range(1, self.node.model.njoints):
            cur = joint_id
            while cur > 0:
                if cur == waist_id:
                    ids.add(joint_id)
                    break
                cur = self.node.model.parents[cur]
        return ids

    def is_draw_joint(self, joint_id):
        name = self.node.model.names[joint_id]
        if "mimic" in name:
            return False
        if "dummy" in name:
            return False
        return True

    def get_joint_translation(self, joint_name):
        joint_id = self.node.model.getJointId(joint_name)
        if joint_id == self.node.model.njoints:
            return None
        return self.node.data.oMi[joint_id].translation

    def add_leg_corrected_lines(self, line_vertices, leg_fk, T_delta):
        if leg_fk is None:
            return
        def pos(link_name):
            T = leg_fk.get(link_name)
            if T is None:
                return None
            return T.translation

        special_links = [
            ("lifter_bottom_link", "leg_shank_link"),
            ("leg_shank_link", "leg_knee_link"),
            ("leg_knee_link", "leg_thigh_link"),
            ("leg_thigh_link", "leg_base_link"),
        ]

        for parent_link, child_link in special_links:
            p0 = pos(parent_link)
            p1 = pos(child_link)
            if p0 is None or p1 is None:
                continue
            line_vertices.extend([
                p0[0], p0[1], p0[2],
                p1[0], p1[1], p1[2],
            ])

        waist = self.get_joint_translation("waist_y_joint")
        if waist is not None and T_delta is not None:
            waist = self.apply_delta(T_delta, waist)
            leg_base = pos("leg_base_link")
            if leg_base is not None:
                line_vertices.extend([
                    leg_base[0], leg_base[1], leg_base[2],
                    waist[0], waist[1], waist[2],
                ])

    def update_vbo(self):
        if not self.gl_ready:
            return

        leg_fk = self.node.compute_leg_display_fk()
        T_delta = None
        if leg_fk is not None:
            T_leg_base_corrected = leg_fk["leg_base_link"]
            T_leg_base_original = self.node.get_link_transform("leg_base_link")
            if T_leg_base_original is not None:
                T_delta = (
                    T_leg_base_corrected
                    * T_leg_base_original.inverse()
                )

        point_vertices = []
        for joint_id in range(1, self.node.model.njoints):
            if not self.is_draw_joint(joint_id):
                continue
            p = self.node.data.oMi[joint_id].translation
            if T_delta is not None and joint_id in self.upper_joint_ids:
                p = self.apply_delta(T_delta, p)
            point_vertices.extend([
                p[0], p[1], p[2]
            ])

        line_vertices = []
        skip_links = {
            ("ankle_joint", "knee_joint"),
            ("knee_joint", "waist_y_joint"),
            ("ankle_joint", "ankle_joint_mimic"),
            ("ankle_joint_mimic", "knee_joint"),
            ("knee_joint", "knee_joint_mimic"),
            ("knee_joint_mimic", "waist_y_joint"),
        }
        for joint_id in range(1, self.node.model.njoints):
            if not self.is_draw_joint(joint_id):
                continue

            parent = self.node.model.parents[joint_id]
            if parent <= 0:
                continue

            parent_name = self.node.model.names[parent]
            child_name = self.node.model.names[joint_id]

            if (parent_name, child_name) in skip_links:
                continue

            p0 = self.node.data.oMi[parent].translation
            p1 = self.node.data.oMi[joint_id].translation
            if T_delta is not None and parent in self.upper_joint_ids:
                p0 = self.apply_delta(T_delta, p0)
            if T_delta is not None and joint_id in self.upper_joint_ids:
                p1 = self.apply_delta(T_delta, p1)
            line_vertices.extend([
                p0[0], p0[1], p0[2],
                p1[0], p1[1], p1[2],
            ])

        self.add_leg_corrected_lines(
            line_vertices,
            leg_fk,
            T_delta
        )

        point_vertices = np.array(point_vertices, dtype=np.float32)
        line_vertices = np.array(line_vertices, dtype=np.float32)

        self.point_count = len(point_vertices) // 3
        self.line_count = len(line_vertices) // 3

        if self.point_vbo is None:
            self.point_vbo = vbo.VBO(point_vertices, usage='GL_DYNAMIC_DRAW')
        else:
            self.point_vbo.set_array(point_vertices)
        if self.line_vbo is None:
            self.line_vbo = vbo.VBO(line_vertices, usage='GL_DYNAMIC_DRAW')
        else:
            self.line_vbo.set_array(line_vertices)

    def redraw(self):
        glClear(
            GL_COLOR_BUFFER_BIT |
            GL_DEPTH_BUFFER_BIT
        )
        w = max(
            self.winfo_width(),
            1
        )
        h = max(
            self.winfo_height(),
            1
        )
        glViewport(
            0,
            0,
            w,
            h
        )
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(
            45,
            w / h,
            0.01,
            100
        )
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(
            -1.0,
            -1.5,
            1.5,

            0.3,
            0.2,
            0.8,

            0,
            0,
            1
        )

        if self.point_vbo is None or self.line_vbo is None:
            return
        glEnableClientState(GL_VERTEX_ARRAY)
        glPointSize(8)
        self.point_vbo.bind()
        glColor3f(1.0, 0.0, 0.0)
        glVertexPointer(
            3,
            GL_FLOAT,
            0,
            None
        )
        glDrawArrays(
            GL_POINTS,
            0,
            self.point_count
        )
        self.point_vbo.unbind()

        self.line_vbo.bind()
        glColor3f(0.0, 0.0, 1.0)
        glLineWidth(7.0)
        glVertexPointer(
            3,
            GL_FLOAT,
            0,
            None
        )
        glDrawArrays(
            GL_LINES,
            0,
            self.line_count
        )
        self.line_vbo.unbind()
        glDisableClientState(GL_VERTEX_ARRAY)
        glFlush()

    def cleanup_gl(self):
        if self.point_vbo:
            self.point_vbo.delete()
        if self.line_vbo:
            self.line_vbo.delete()


class MainGUI(ctk.CTk):
    def __init__(self, camera_node, robot_node):
        super().__init__()

        self.camera_node = camera_node
        self.robot_node = robot_node

        self.robot_process = None
        self.tracer_process = None

        self.finish_requested = False
        self.process_lock = threading.Lock()

        self.title("MotionTracerGUI ROS2")
        self.geometry("1920x1080")
        # self.attributes('-fullscreen',True)
        self.after(0,lambda:self.attributes('-zoomed',True))

        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        self.labels = {}
        self.left_current_percent = 30
        self.right_current_percent = 30

        self.ssh_config = self.load_ssh_config()

        self.main_frame = ctk.CTkFrame(self)
        self.main_frame.pack(fill="both", expand=True)
        self.main_frame.grid_rowconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(1, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_columnconfigure(1, weight=1)

        head_image_frame = ctk.CTkFrame(self.main_frame)
        head_image_frame.grid(row=0, column=0, sticky="nsew")

        head_label = ctk.CTkLabel(head_image_frame, width=960, height=540, text="/camera1/image_raw/compressed\nWaiting for image...")
        head_label.pack(padx=10)
        self.labels["camera1"] = head_label

        right_image_frame = ctk.CTkFrame(self.main_frame)
        right_image_frame.grid(row=0, column=1, sticky="nsew")

        right_label = ctk.CTkLabel(right_image_frame, width=960, height=540, text="/camera2/image_raw/compressed\nWaiting for image...")
        right_label.pack(padx=10)
        self.labels["camera2"] = right_label

        left_image_frame = ctk.CTkFrame(self.main_frame)
        left_image_frame.grid(row=1, column=0, sticky="nsew")

        left_label = ctk.CTkLabel(left_image_frame, width=960, height=540, text="/camera3/image_raw/compressed\nWaiting for image...")
        left_label.pack(padx=10)
        self.labels["camera3"] = left_label

        control_frame = ctk.CTkFrame(self.main_frame, width=960, height=540)
        control_frame.grid(row=1, column=1, sticky="nsew")
        control_frame.grid_rowconfigure(0, weight=1)
        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=2)

        viewer_frame = ctk.CTkFrame(control_frame)
        viewer_frame.grid(row=0, column=0, sticky="nsew")

        self.skeleton_viewer = (SkeletonViewer(viewer_frame, robot_node))
        self.skeleton_viewer.pack(fill="both", expand=True)

        operation_frame = ctk.CTkFrame(control_frame)
        operation_frame.grid(row=0, column=1, sticky="nsew")
        operation_frame.grid_rowconfigure(0, weight=0)
        operation_frame.grid_rowconfigure(1, weight=1)
        operation_frame.grid_rowconfigure(2, weight=1)
        operation_frame.grid_rowconfigure(3, weight=0)
        operation_frame.grid_rowconfigure(4, weight=0)
        operation_frame.grid_rowconfigure(5, weight=0)
        operation_frame.grid_columnconfigure(0, weight=1)

        ssh_config_frame = ctk.CTkFrame(operation_frame)
        ssh_config_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        ssh_config_frame.grid_columnconfigure(1, weight=1)
        ssh_config_frame.grid_columnconfigure(3, weight=1)

        ssh_target_label = ctk.CTkLabel(ssh_config_frame, text="Connect to [user@host]:")
        ssh_target_label.grid(row=0, column=0, padx=5, pady=5)
        self.robot_ssh_target_entry = ctk.CTkEntry(ssh_config_frame, width=220)
        self.robot_ssh_target_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.robot_ssh_target_entry.insert(0, self.ssh_config["robot_ssh_target"])

        ssh_password_label = ctk.CTkLabel(ssh_config_frame, text="password:")
        ssh_password_label.grid(row=0, column=2, padx=5, pady=5)
        self.robot_ssh_password_entry = ctk.CTkEntry(ssh_config_frame, width=180,show="*")
        self.robot_ssh_password_entry.grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        self.robot_ssh_password_entry.insert(0, self.ssh_config["robot_ssh_password"])

        control_button_frame = ctk.CTkFrame(operation_frame)
        control_button_frame.grid(row=1, column=0, sticky="")
        for c in range(3):
            control_button_frame.grid_columnconfigure(c, weight=1)

        robot_bringup_button = ctk.CTkButton(control_button_frame, text="Robot Bring Up", width=150, height=150, command=None)
        robot_bringup_button.grid(row=0, column=0, padx=10)
        robot_bringup_button.bind("<ButtonRelease-1>", self.on_robot_bringup_release)

        tracer_bringup_button = ctk.CTkButton(control_button_frame, text="Tracer Bring Up", width=150, height=150, command=None)
        tracer_bringup_button.grid(row=0, column=1, padx=10)
        tracer_bringup_button.bind("<ButtonRelease-1>", self.on_tracer_bringup_release)

        finish_button = ctk.CTkButton(control_button_frame, text="All Finish", width=150, height=150, command=None)
        finish_button.grid(row=0, column=2, padx=10)
        finish_button.bind("<ButtonRelease-1>", self.on_finish_release)

        control_slider_frame = ctk.CTkFrame(operation_frame)
        control_slider_frame.grid(row=2, column=0, sticky="")
        control_slider_frame.grid_columnconfigure(0, weight=1)
        control_slider_frame.grid_columnconfigure(1, weight=1)

        self.left_hand_current_label = ctk.CTkLabel(control_slider_frame, text=f" Left Hand Current : {self.left_current_percent} [%] ")
        self.left_hand_current_label.grid(row=0, column=0, padx=10)
        self.left_hand_current_slider = ctk.CTkSlider(control_slider_frame, from_=1, to=100, number_of_steps=99, command=self.on_left_slider_change)
        self.left_hand_current_slider.set(30)
        self.left_hand_current_slider.grid(row=1, column=0, padx=10)
        self.left_hand_current_slider.bind("<ButtonRelease-1>", self.on_left_slider_release)

        self.right_hand_current_label = ctk.CTkLabel(control_slider_frame, text=f" Right Hand Current : {self.right_current_percent} [%] ")
        self.right_hand_current_label.grid(row=0, column=1, padx=10)
        self.right_hand_current_slider = ctk.CTkSlider(control_slider_frame, from_=1, to=100, number_of_steps=99, command=self.on_right_slider_change)
        self.right_hand_current_slider.set(30)
        self.right_hand_current_slider.grid(row=1, column=1, padx=10)
        self.right_hand_current_slider.bind("<ButtonRelease-1>", self.on_right_slider_release)

        switch_frame = ctk.CTkFrame(operation_frame)
        switch_frame.grid(row=3, column=0, sticky="")
        lifter_forward_lean_label = ctk.CTkLabel(switch_frame, text="lifter forward lean")
        lifter_forward_lean_label.grid(row=0, column=0, padx=10)
        self.lifter_forward_lean_switch = ctk.CTkSwitch(switch_frame, text="OFF", command=self.on_lifter_forward_lean_toggle, onvalue=True, offvalue=False, width=80)
        self.lifter_forward_lean_switch.grid(row=1, column=0, padx=10)
        self.lifter_forward_lean_switch.deselect()

        onoff_frame = ctk.CTkFrame(operation_frame)
        onoff_frame.grid(row=4, column=0, pady=5, sticky="nsew")
        self.onoff_label = ctk.CTkLabel(onoff_frame, text="Tracer ON,OFF : ---", font=ctk.CTkFont(size=28, weight="bold"))
        self.onoff_label.pack(padx=10, pady=15)
        self.last_onoff_version = -1

        mode_frame = ctk.CTkFrame(operation_frame)
        mode_frame.grid(row=5, column=0, pady=5, sticky="nsew")
        self.mode_label = ctk.CTkLabel(mode_frame, text="Mode : ---", font=ctk.CTkFont(size=28, weight="bold"))
        self.mode_label.pack(padx=10, pady=15)
        self.last_mode_version = -1

        self.update_images()
        self.after(100,self.update_skeleton)
        self.update_onoff_label()
        self.update_mode_label()

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def get_config_path(self):
        script_dir = Path(__file__).resolve().parent
        return script_dir.parent / "config" / "ssh_config.json"

    def load_ssh_config(self):
        default_config = {
            "robot_ssh_target": "",
            "robot_ssh_password": "",
        }

        config_path = self.get_config_path()
        try:
            if config_path.exists():
                with open(config_path, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                default_config.update(loaded)
        except Exception as e:
            print(f"Failed to load ssh config: {e}")
        return default_config

    def save_ssh_config(self):
        config_path = self.get_config_path()
        config_path.parent.mkdir(parents=True, exist_ok=True)

        config = {
            "robot_ssh_target": self.robot_ssh_target_entry.get(),
            "robot_ssh_password": self.robot_ssh_password_entry.get(),
        }
        try:
            with open(config_path, "w", encoding="utf-8") as f:
                json.dump(config, f, indent=4)
        except Exception as e:
            print(f"Failed to save ssh config: {e}")

    def update_skeleton(self):
        ok = self.robot_node.compute_fk()
        if ok:
            self.skeleton_viewer.update_vbo()
            self.skeleton_viewer.redraw()
        self.after(16, self.update_skeleton)

    def make_ros_env(domain_id):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(domain_id)
        return env

    def wait_for_robot_current_service(self, timeout_sec=15):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.is_finish_requested():
                return False
            try:
                result = subprocess.run(
                    [
                        "ros2",
                        "service",
                        "list"
                    ],
                    capture_output=True,
                    text=True,
                    timeout=3,
                    env=self.make_ros_env(ROBOT_DOMAIN_ID)
                )
                if "/aero_controller/set_current" in result.stdout:
                    return True
            except Exception as e:
                print(f"Service check failed: {e}")
            time.sleep(1.0)
        return False

    def on_robot_bringup_release(self, event):
        self.on_robot_bringup_click()

    def on_robot_bringup_click(self):
        with self.process_lock:
            if self.finish_requested:
                return

        ssh_target = self.robot_ssh_target_entry.get().strip()
        ssh_password = self.robot_ssh_password_entry.get()

        threading.Thread(
            target=self.start_robot_bringup,
            args=(ssh_target, ssh_password),
            daemon=True
        ).start()
        print("Clicked Robot Bring Up Button")

    def start_robot_bringup(self, ssh_target, ssh_password):
        try:
            result = subprocess.run(
                [
                    "sshpass",
                    "-p",
                    ssh_password,
                    "ssh",
                    "-o",
                    "ConnectTimeout=1",
                    "-o",
                    "StrictHostKeyChecking=no",
                    ssh_target,
                    "echo connected"
                ],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode != 0:
                print("ERROR : SSH connection failed")
                print(result.stderr)
                return

        except Exception as e:
            print(f"ERROR : SSH connection exception : {e}")
            return

        if self.is_finish_requested():
            return

        quoted_password = shlex.quote(ssh_password)
        quoted_target = shlex.quote(ssh_target)

        self.robot_process = subprocess.Popen(
            [
                "gnome-terminal",
                "--",
                "bash",
                "-lc",
                (
                    f"sshpass -p {quoted_password} "
                    "ssh -o StrictHostKeyChecking=no "
                    f"{quoted_target} "
                    "'export ROS_DOMAIN_ID=20 && "
                    "source /opt/ros/jazzy/setup.bash && "
                    "source ~/ros2/jazzy/install/setup.bash && "
                    "ros2 launch motion_tracer_ros2 robot_bringup.launch.py simulation:=false display_rviz2:=false'"
                )
            ],
            preexec_fn=os.setsid
        )

        if not self.wait_for_robot_current_service(timeout_sec=15):
            print("ERROR : /aero_controller/set_current service is not available")
            return

        if self.is_finish_requested():
            return

        self.call_left_hand_current_service(self.left_current_percent)
        self.call_right_hand_current_service(self.right_current_percent)
        print("Robot System Bring Up")

    def on_tracer_bringup_release(self, event):
        self.on_tracer_bringup_click()

    def on_tracer_bringup_click(self):
        with self.process_lock:
            if self.finish_requested:
                return

        threading.Thread(
            target=self.start_tracer_bringup,
            daemon=True
        ).start()
        print("Clicked Tracer Bring Up Button")

    def start_tracer_bringup(self):
        if self.is_finish_requested():
            return
        self.tracer_process = subprocess.Popen(
            [
                "gnome-terminal",
                "--",
                "bash",
                "-lc",
                "export ROS_DOMAIN_ID=20 && "
                "source /opt/ros/jazzy/setup.bash && "
                "source /home/seed/ros2/jazzy/install/setup.bash && "
                "ros2 launch motion_tracer_ros2 tracer_bringup.launch.py"
            ],
            preexec_fn=os.setsid
        )
        print("Tracer System Bring Up")

    def is_finish_requested(self):
        with self.process_lock:
            return self.finish_requested

    def on_finish_release(self, event):
        self.on_finish_click()

    def on_finish_click(self):
        threading.Thread(
            target=self.finish_all,
            daemon=True
        ).start()
        print("Clicked All Finish Button")

    def finish_all(self):
        with self.process_lock:
            self.finish_requested = True

        ssh_target = self.robot_ssh_target_entry.get().strip()
        ssh_password = self.robot_ssh_password_entry.get()

        subprocess.run(
            [
                "killall",
                "-SIGINT",
                "ros2"
            ]
        )

        ok_ssh = True
        try:
            result = subprocess.run(
                [
                    "sshpass",
                    "-p",
                    ssh_password,
                    "ssh",
                    "-o",
                    "ConnectTimeout=1",
                    "-o",
                    "StrictHostKeyChecking=no",
                    ssh_target,
                    "echo connected"
                ],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode != 0:
                ok_ssh = False
                print("ERROR : SSH connection failed")
                print(result.stderr)

        except Exception as e:
            ok_ssh = False
            print(f"ERROR : SSH connection exception : {e}")

        if ok_ssh:
            try:
                subprocess.run(
                    [
                        "sshpass",
                        "-p",
                        ssh_password,
                        "ssh",
                        "-o",
                        "StrictHostKeyChecking=no",
                        ssh_target,
                        "killall -SIGINT "
                        "ros2 "
                        "ros2_control_node "
                        "robot_state_publisher "
                        "rviz2 urg_node2_node "
                        "scan_to_scan_filter_chain "
                        "init_pose_pub "
                        "static_transform_publisher "
                        "component_container "
                        "joy_linux_node "
                        "upper_controller_node "
                        "lower_controller_node "
                        "move_group"
                    ]
                )
            except Exception as e:
                print(f"ERROR : ROS process kill exception : {e}")
        time.sleep(1)

        if self.tracer_process is not None:
            try:
                self.tracer_process.terminate()
                self.tracer_process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.tracer_process.kill()
            self.tracer_process = None

        if self.robot_process is not None:
            try:
                self.robot_process.terminate()
                self.robot_process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.robot_process.kill()
            self.robot_process = None

        self.finish_requested = False
        print("All System Finished")

    def on_left_slider_change(self, slider_value):
        self.left_current_percent = round(slider_value)
        self.left_hand_current_label.configure(text=f" Left Hand Current : {self.left_current_percent} [%] ")

    def on_left_slider_release(self, event):
        threading.Thread(
            target=self.call_left_hand_current_service,
            args=(self.left_current_percent,),
            daemon=True
        ).start()

    def call_left_hand_current_service(self, current_value):
        try:
            subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    "/aero_controller/set_current",
                    "aero_controller_msgs/srv/SetCurrent",
                    (
                        "{joint_name: ['l_thumb_joint'], "
                        f"max: [{int(current_value)}], "
                        "min: [1]}"
                    )
                ],
                check=True,
                env=self.make_ros_env(ROBOT_DOMAIN_ID)
            )
        except Exception as e:
            print(f"Service call failed: {e}")

    def on_right_slider_change(self, slider_value):
        self.right_current_percent = round(slider_value)
        self.right_hand_current_label.configure(text=f" Right Hand Current : {self.right_current_percent} [%] ")

    def on_right_slider_release(self, event):
        threading.Thread(
            target=self.call_right_hand_current_service,
            args=(self.right_current_percent,),
            daemon=True
        ).start()

    def call_right_hand_current_service(self, current_value):
        try:
            subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    "/aero_controller/set_current",
                    "aero_controller_msgs/srv/SetCurrent",
                    (
                        "{joint_name: ['r_thumb_joint'], "
                        f"max: [{int(current_value)}], "
                        "min: [1]}"
                    )
                ],
                check=True,
                env=self.make_ros_env(ROBOT_DOMAIN_ID)
            )
        except Exception as e:
            print(f"Service call failed: {e}")

    def on_lifter_forward_lean_toggle(self):
        enabled = bool(self.lifter_forward_lean_switch.get())
        if enabled:
            self.lifter_forward_lean_switch.configure(text="ON")
        else:
            self.lifter_forward_lean_switch.configure(text="OFF")
        self.robot_node.notify_lifter_forward_lean(enabled)

    def cv_to_tk(self, cv_img):
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (960, 540))
        pil_img = Image.fromarray(rgb)
        return ImageTk.PhotoImage(pil_img)

    def update_images(self):
        with self.camera_node.lock:
            images = {name: image.copy()
                for name, image in self.camera_node.images.items()
                if image is not None
            }
        for cam, img in images.items():
            tk_img = self.cv_to_tk(img)
            self.labels[cam].configure(image=tk_img, text="")
            self.labels[cam].image = tk_img
        self.after(30, self.update_images)

    def update_onoff_label(self):
        with self.robot_node.notify_lock:
            cur_onoff = self.robot_node.cur_onoff
            version = self.robot_node.cur_onoff_version
        if version != self.last_onoff_version:
            if cur_onoff:
                onoff_text = "Trcaer  [ ON ]"
            else:
                onoff_text = "Tracer  [ OFF ]"
            self.onoff_label.configure(text=onoff_text)
            self.last_onoff_version = version
        self.after(50, self.update_onoff_label)

    def update_mode_label(self):
        with self.robot_node.notify_lock:
            cur_mode = self.robot_node.cur_mode
            version = self.robot_node.cur_mode_version
        if version != self.last_mode_version:
            if cur_mode:
                mode_text = "Mode  [ Mover & Lifter ]"
            else:
                mode_text = "Mode  [ Neck & Waist ]"
            self.mode_label.configure(text=mode_text)
            self.last_mode_version = version
        self.after(50, self.update_mode_label)

    def on_close(self):
        self.save_ssh_config()
        self.finish_all()
        self.skeleton_viewer.cleanup_gl()
        self.destroy()


def ros_spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

def spin_executor(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    except Exception as e:
        print(f"ROS executor error: {e}")

def main():
    camera_context = Context()
    robot_context = Context()

    camera_node = None
    robot_node = None

    camera_executor = None
    robot_executor = None

    camera_thread = None
    robot_thread = None

    try:
        rclpy.init(context=camera_context, domain_id=CAMERA_DOMAIN_ID)
        rclpy.init(context=robot_context, domain_id=ROBOT_DOMAIN_ID)

        camera_node = CameraSubscriber(camera_context)
        robot_node = RobotSubscriber(robot_context)

        camera_executor = SingleThreadedExecutor(context=camera_context)
        robot_executor = SingleThreadedExecutor(context=robot_context)

        camera_executor.add_node(camera_node)
        robot_executor.add_node(robot_node)

        camera_thread = threading.Thread(target=spin_executor, args=(camera_executor,), daemon=True)
        robot_thread = threading.Thread(target=spin_executor, args=(robot_executor,),daemon=True)

        camera_thread.start()
        robot_thread.start()

        app = MainGUI(camera_node=camera_node, robot_node=robot_node)
        app.mainloop()

    finally:
        if camera_executor is not None:
            camera_executor.shutdown(timeout_sec=2.0)
        if robot_executor is not None:
            robot_executor.shutdown(timeout_sec=2.0)
        if camera_thread is not None:
            camera_thread.join(timeout=2.0)
        if robot_thread is not None:
            robot_thread.join(timeout=2.0)
        if camera_node is not None:
            camera_node.destroy_node()
        if robot_node is not None:
            robot_node.destroy_node()
        camera_context.try_shutdown()
        robot_context.try_shutdown()

if __name__ == '__main__':
    main()
