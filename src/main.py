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

import subprocess
import os
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.lock = threading.Lock()

        self.images = {
            "camera1": None,
            "camera2": None,
            "camera3": None
        }

        self.create_subscription(
            RosImage,
            '/camera1/image_raw/compressed',
            lambda msg: self.image_callback(msg, "camera1"),
            qos_profile_sensor_data
        )

        self.create_subscription(
            RosImage,
            '/camera2/image_raw/compressed',
            lambda msg: self.image_callback(msg, "camera2"),
            qos_profile_sensor_data
        )

        self.create_subscription(
            RosImage,
            '/camera3/image_raw/compressed',
            lambda msg: self.image_callback(msg, "camera3"),
            qos_profile_sensor_data
        )

        self.get_logger().info("Subscribed to camera topics")

    def image_callback(self, msg, cam_name):
        # self.get_logger().info(f"{cam_name} frame received")
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.lock:
                self.images[cam_name] = cv_img
        except Exception as e:
            self.get_logger().error(f"{cam_name}: {e}")


class ImageGUI(ctk.CTk):
    def __init__(self, ros_node):
        super().__init__()

        self.node = ros_node

        self.robot_process = None
        self.tracer_process = None

        self.title("MotionTracerGUI ROS2")
        self.geometry("1920x1080")

        ctk.set_appearance_mode("Dark")
        ctk.set_default_color_theme("blue")

        self.labels = {}

        self.main_frame = ctk.CTkFrame(self)
        self.main_frame.pack(fill="both", expand=True)

        #-----------------------------------------------------------------------------------------

        # self.view_frame = ctk.CTkFrame(self.main_frame)
        # self.view_frame.pack(side=ctk.TOP)

        # for i, cam in enumerate(["camera1", "camera2", "camera3"]):
        #     image_frame = ctk.CTkFrame(self.view_frame)
        #     image_frame.grid(row=0, column=i)

        #     label = ctk.CTkLabel(image_frame, text=f"/{cam}/image_raw/compressed\nWaiting for image...")
        #     label.pack(padx=1)
        #     self.labels[cam] = label

        #-----------------------------------------------------------------------------------------

        # self.head_view_frame = ctk.CTkFrame(self.main_frame)
        # self.head_view_frame.pack(side=ctk.TOP)

        # head_image_frame = ctk.CTkFrame(self.head_view_frame)
        # head_image_frame.pack()

        # head_label = ctk.CTkLabel(head_image_frame, text="/camera1/image_raw/compressed\nWaiting for image...")
        # head_label.pack(padx=1)
        # self.labels["camera1"] = head_label

        # self.arm_view_frame = ctk.CTkFrame(self.main_frame)
        # self.arm_view_frame.pack(side=ctk.TOP)

        # left_image_frame = ctk.CTkFrame(self.arm_view_frame)
        # left_image_frame.pack(side=ctk.LEFT)

        # left_label = ctk.CTkLabel(left_image_frame, text="/camera2/image_raw/compressed\nWaiting for image...")
        # left_label.pack(padx=1)
        # self.labels["camera2"] = left_label

        # right_image_frame = ctk.CTkFrame(self.arm_view_frame)
        # right_image_frame.pack(side=ctk.RIGHT)

        # right_label = ctk.CTkLabel(right_image_frame, text="/camera3/image_raw/compressed\nWaiting for image...")
        # right_label.pack(padx=1)
        # self.labels["camera3"] = right_label

        #-----------------------------------------------------------------------------------------

        head_image_frame = ctk.CTkFrame(self.main_frame)
        head_image_frame.grid(row=0, column=0)

        head_label = ctk.CTkLabel(head_image_frame, width=960, height=540, text="/camera1/image_raw/compressed\nWaiting for image...")
        head_label.pack(padx=1)
        self.labels["camera1"] = head_label

        right_image_frame = ctk.CTkFrame(self.main_frame)
        right_image_frame.grid(row=0, column=1)

        right_label = ctk.CTkLabel(right_image_frame, width=960, height=540, text="/camera2/image_raw/compressed\nWaiting for image...")
        right_label.pack(padx=1)
        self.labels["camera2"] = right_label

        left_image_frame = ctk.CTkFrame(self.main_frame)
        left_image_frame.grid(row=1, column=0)

        left_label = ctk.CTkLabel(left_image_frame, width=960, height=540, text="/camera3/image_raw/compressed\nWaiting for image...")
        left_label.pack(padx=1)
        self.labels["camera3"] = left_label

        #-----------------------------------------------------------------------------------------

        control_frame = ctk.CTkFrame(self.main_frame, width=960, height=540)
        control_frame.grid(row=1, column=1)
        # control_frame.grid_propagate(False)

        control_button_frame = ctk.CTkFrame(control_frame, width=960, height=270)
        control_button_frame.pack(side=ctk.TOP, pady=(0, 50))
        # control_button_frame.pack_propagate(False)

        robot_bringup_button = ctk.CTkButton(control_button_frame, text="Robot Bring Up", width=150, height=150, command=self.on_robot_bringup_click)
        robot_bringup_button.grid(row=0, column=0, padx=10, pady=0)

        tracer_bringup_button = ctk.CTkButton(control_button_frame, text="Tracer Bring Up", width=150, height=150, command=self.on_tracer_bringup_click)
        tracer_bringup_button.grid(row=0, column=1, padx=10, pady=0)

        finish_button = ctk.CTkButton(control_button_frame, text="All Finish", width=150, height=150, command=self.on_finish_click)
        finish_button.grid(row=0, column=2, padx=10, pady=0)

        control_slider_frame = ctk.CTkFrame(control_frame, width=960, height=270)
        control_slider_frame.pack(side=ctk.BOTTOM)

        grip_current_slider = ctk.CTkSlider(control_slider_frame, from_=1, to=100, number_of_steps=99, command=self.on_slider_change)
        grip_current_slider.set(30)
        grip_current_slider.pack()

        self.after(1000, lambda: print(control_frame.winfo_height()))

        self.update_images()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def on_robot_bringup_click(self):
        print("ロボット起動ボタンがクリックされました")
        threading.Thread(
            target=self.start_robot_bringup,
            daemon=True
        ).start()

    def start_robot_bringup(self):
        print("ロボット起動しました")

        if self.robot_process is not None:
            print("Robotは既に起動中です")
            return

        #SSHパスワード暫定対応
        self.robot_process = subprocess.Popen(
            [
                "gnome-terminal",
                "--",
                "bash",
                "-lc",
                (
                    "sshpass -p 'seed' "
                    "ssh -o StrictHostKeyChecking=no "
                    "seed@192.168.0.50 "
                    "'source /opt/ros/jazzy/setup.bash && "
                    "source ~/ros2/jazzy/install/setup.bash && "
                    "ros2 launch motion_tracer_ros2 robot_bringup.launch.py simulation:=false'"
                )
            ],
            preexec_fn=os.setsid
        )

    def on_tracer_bringup_click(self):
        print("トレーサー起動ボタンがクリックされました")

        threading.Thread(
            target=self.start_tracer_bringup,
            daemon=True
        ).start()

    def start_tracer_bringup(self):
        print("トレーサー起動しました")

        if self.tracer_process is not None:
            print("Tracerは既に起動中です")
            return

        self.tracer_process = subprocess.Popen(
            [
                "gnome-terminal",
                "--",
                "bash",
                "-lc",
                "source /opt/ros/jazzy/setup.bash && "
                "source /home/seed/ros2_ws/install/setup.bash && "
                "ros2 launch motion_tracer_ros2 tracer_bringup.launch.py"
            ],
            preexec_fn=os.setsid
        )

    def on_finish_click(self):
        print("終了ボタンがクリックされました")

        threading.Thread(
            target=self.finish_all,
            daemon=True
        ).start()

    def finish_all(self):
        print("全システム停止")

        subprocess.run(
            [
                "killall",
                "-SIGINT",
                "ros2"
            ]
        )
        #SSHパスワード暫定対応
        subprocess.run(
            [
                "sshpass",
                "-p",
                "seed",
                "ssh",
                "-o",
                "StrictHostKeyChecking=no",
                "seed@192.168.0.50",
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

    def cv_to_tk(self, cv_img):
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        # rgb = cv2.resize(rgb, (638, 800))
        # rgb = cv2.resize(rgb, (588, 440))
        # rgb = cv2.resize(rgb, (960, 512))
        rgb = cv2.resize(rgb, (960, 540))

        pil_img = Image.fromarray(rgb)

        return ImageTk.PhotoImage(pil_img)

    def on_slider_change(self, value):
        print("現在のスライダーの値:", round(value, 2))

    def update_images(self):
        with self.node.lock:
            for cam, img in self.node.images.items():
                if img is not None:
                    tk_img = self.cv_to_tk(img)

                    self.labels[cam].configure(image=tk_img,text="")
                    self.labels[cam].image = tk_img

        self.after(30, self.update_images)
    
    def on_close(self):
        self.destroy()


def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()

    node = ImageSubscriber()

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    app = ImageGUI(node)

    try:
        app.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join()
        node.destroy_node()

if __name__ == '__main__':
    main()
