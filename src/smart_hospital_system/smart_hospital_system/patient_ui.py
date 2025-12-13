#!/usr/bin/env python3
import faulthandler
faulthandler.enable()

import os
import time
import threading
import queue
import json
import requests
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import qrcode
import random
import subprocess  # ★ 중요: UI 실행용

# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

DOCTOR_FORM_ID = "253293055163051"
TARGET_FIELD_NAME = "input_3"

class SmartHospitalApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node

        self.root.title("스마트 병원 환자용 키오스크")
        self.root.geometry("500x850")
        self.root.configure(bg="#f0f4f8")

        self.patient_name = ""
        self.unique_id = None
        self.qr_image = None
        self.running = True

        self.main_frame = tk.Frame(root, bg="#f0f4f8")
        self.main_frame.pack(fill="both", expand=True)

        self.event_queue = queue.Queue()
        self.show_home_screen()

        # 환자 시작 신호 발행
        self.pub_start = self.node.create_publisher(String, '/hospital_data', 10)

        # waypoint 도착 신호 구독
        self.sub_arrived = self.node.create_subscription(
            Bool, '/arrived',
            lambda m: self.event_queue.put(("arrived", m.data)),
            10
        )

        self.root.after(100, self.update_loop)
        print("환자 UI 실행됨")

    def update_loop(self):
        try:
            while True:
                msg_type, data = self.event_queue.get_nowait()

                if msg_type == "arrived" and data:
                    self.show_arrived_popup()
        except queue.Empty:
            pass

        self.root.after(100, self.update_loop)

    def clear_frame(self):
        for w in self.main_frame.winfo_children():
            w.destroy()

    def show_home_screen(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="스마트 병원 접수", font=("Arial", 24, "bold")).pack(pady=40)
        tk.Button(self.main_frame, text="수동 접수", font=("Arial", 16),
                  command=self.show_questionnaire).pack(fill="x", pady=20)

    def show_questionnaire(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="이름 입력", font=("Arial", 18)).pack(pady=20)
        self.entry_name = tk.Entry(self.main_frame, font=("Arial", 14))
        self.entry_name.pack(fill="x", padx=20)
        tk.Button(self.main_frame, text="제출", font=("Arial", 14),
                  command=self.submit_manual).pack(pady=20)

    def submit_manual(self):
        self.patient_name = self.entry_name.get()
        self.unique_id = "999"
        self.generate_qr()

    def generate_qr(self):
        self.clear_frame()
        url = f"https://form.jotform.com/{DOCTOR_FORM_ID}?{TARGET_FIELD_NAME}={self.unique_id}"

        qr = qrcode.QRCode(box_size=10, border=4)
        qr.add_data(url)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
        img = img.resize((250, 250))
        self.qr_image = ImageTk.PhotoImage(img)

        tk.Label(self.main_frame, text=f"{self.patient_name} 님 접수증",
                 font=("Arial", 18)).pack(pady=20)
        tk.Label(self.main_frame, image=self.qr_image).pack(pady=20)

        tk.Button(self.main_frame, text="로봇 연동 시작", font=("Arial", 16),
                  bg="#22c55e", fg="white",
                  command=self.start_robot).pack(fill="x", pady=20)

    def start_robot(self):
        msg = {
            "command": "start",
            "patient_name": self.patient_name
        }
        self.pub_start.publish(String(data=json.dumps(msg)))
        messagebox.showinfo("안내", "로봇이 진료실로 이동합니다.\n잠시만 기다려주세요.")

    def show_arrived_popup(self):
        messagebox.showinfo("도착", "로봇이 진료실에 도착했습니다!")

        # ★ waypoint 도착 시에만 의료진 UI 실행
        subprocess.Popen(["ros2", "run", "smart_hospital_system", "doctor_ui"])
        subprocess.Popen(["ros2", "run", "smart_hospital_system", "dashboard_ui"])

def ros_thread(node, app):
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok() and app.running:
        executor.spin_once(timeout_sec=0.1)

def main():
    rclpy.init()
    node = Node("patient_ui_node")

    if "GDK_BACKEND" not in os.environ:
        os.environ["GDK_BACKEND"] = "x11"

    root = tk.Tk()
    app = SmartHospitalApp(root, node)

    t = threading.Thread(target=ros_thread, args=(node, app), daemon=True)
    t.start()

    try:
        root.mainloop()
    finally:
        app.running = False
        t.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
