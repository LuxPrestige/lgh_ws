#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import pygame
import numpy as np
import math
import sys

from lux_msgs.msg import JointAngles

# --- 슬라이더(스크롤바) UI 클래스 ---
class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, start_val, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = start_val
        self.label = label
        self.dragging = False

    def draw(self, screen, font):
        # 슬라이더 배경 선
        pygame.draw.rect(screen, (200, 200, 200), self.rect, border_radius=5)
        # 현재 값 위치 핸들
        ratio = (self.val - self.min_val) / (self.max_val - self.min_val)
        hx = self.rect.x + int(ratio * self.rect.w)
        pygame.draw.circle(screen, (50, 150, 220), (hx, self.rect.y + self.rect.h//2), 8)
        
        # 텍스트
        txt = font.render(f"{self.label}: {self.val:.1f}", True, (50, 50, 50))
        screen.blit(txt, (self.rect.x, self.rect.y - 20))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos) or math.hypot(event.pos[0] - (self.rect.x + (self.val - self.min_val) / (self.max_val - self.min_val) * self.rect.w), event.pos[1] - (self.rect.y + self.rect.h//2)) < 15:
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                mx = max(self.rect.x, min(event.pos[0], self.rect.x + self.rect.w))
                ratio = (mx - self.rect.x) / self.rect.w
                self.val = self.min_val + ratio * (self.max_val - self.min_val)

# --- 메인 비주얼라이저 노드 ---
class Pygame3DVisualizer(Node):
    def __init__(self):
        super().__init__('pygame_visualizer')
        self.get_logger().info("Starting Pygame Visualizer with YAML Params...")

        # 1. 파라미터 선언 (기본값은 제공해주신 YAML 값으로 설정)
        self.declare_parameter('shoulder_length', 0.0905)
        self.declare_parameter('elbow_length', 0.210)
        self.declare_parameter('wrist_length', 0.210)
        self.declare_parameter('hip_x', 0.405)
        self.declare_parameter('hip_y', 0.12)

        # 2. 파라미터 값 가져오기 (YAML에서 로드됨)
        self.L1 = self.get_parameter('shoulder_length').value
        self.L2 = self.get_parameter('elbow_length').value
        self.L3 = self.get_parameter('wrist_length').value
        
        # hip_x, hip_y는 전체 길이이므로 중심(0,0) 기준 좌표를 위해 절반으로 나눔
        self.hip_x = self.get_parameter('hip_x').value / 2.0
        self.hip_y = self.get_parameter('hip_y').value / 2.0
        
        # 바닥 높이 자동 계산 (다리 길이 기준)
        self.floor_z = -(self.L2 + self.L3 - 0.05)

        self.get_logger().info(f"Robot Dimensions Loaded -> L1:{self.L1}, L2:{self.L2}, L3:{self.L3}")

        self.q = np.zeros(12) 
        self.create_subscription(JointAngles, '/spot/joints', self.joint_cb, 10)

        # 3. Pygame 설정
        pygame.init()
        self.width, self.height = 1000, 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("LUX Quadruped 3D Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 16, bold=True)

        # 4. UI 슬라이더 설정 (요청하신 범위 적용)
        # Yaw: -360 ~ 360 (회전)
        self.slider_yaw = Slider(self.width - 250, 50, 200, 10, -360, 360, 130, "Cam Yaw (Deg)")
        
        # Pitch: -360 ~ 360 (요청하신 대로 범위 확장)
        self.slider_pitch = Slider(self.width - 250, 120, 200, 10, -360, 360, -110, "Cam Pitch (Deg)")
        
        # Zoom: 10 ~ 4000
        self.slider_zoom = Slider(self.width - 250, 190, 200, 10, 10, 4000, 600, "Zoom (FOV)")
        
        self.sliders = [self.slider_yaw, self.slider_pitch, self.slider_zoom]

    def joint_cb(self, msg):
        self.q = [
            msg.fls, msg.fle, msg.flw,
            msg.frs, msg.fre, msg.frw,
            msg.bls, msg.ble, msg.blw,
            msg.brs, msg.bre, msg.brw
        ]

    def forward_kinematics(self, leg_idx):
        signs = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        sx, sy = signs[leg_idx]
        
        hip_pt = np.array([sx * self.hip_x, sy * self.hip_y, 0])
        idx = leg_idx * 3
        q1, q2, q3 = math.radians(self.q[idx]), math.radians(self.q[idx+1]), math.radians(self.q[idx+2])
        
        s_rot = np.array([0, sy * self.L1 * math.cos(q1), sy * self.L1 * math.sin(q1)])
        shoulder_pt = hip_pt + s_rot
        
        e_rot = np.array([-self.L2 * math.sin(q2), 0, -self.L2 * math.cos(q2)])
        elbow_pt = shoulder_pt + e_rot
        
        w_rot = np.array([-self.L3 * math.sin(q2+q3), 0, -self.L3 * math.cos(q2+q3)])
        foot_pt = elbow_pt + w_rot

        return [hip_pt, shoulder_pt, elbow_pt, foot_pt]

    def project_3d_to_2d(self, point3d):
        x, y, z = point3d
        
        # 슬라이더 값 적용
        yaw = math.radians(self.slider_yaw.val)
        pitch = math.radians(self.slider_pitch.val)
        fov = self.slider_zoom.val

        # Z축 회전 (Yaw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        x_rot = x * cy - y * sy
        y_rot = x * sy + y * cy
        x, y = x_rot, y_rot

        # Y축 회전 (Pitch)
        cp, sp = math.cos(pitch), math.sin(pitch)
        y_rot = y * cp - z * sp
        z_rot = y * sp + z * cp
        y, z = y_rot, z_rot

        z += 1.0 # 카메라 거리
        if z <= 0.1: z = 0.1
        
        px = int((x * fov) / z) + self.width // 2
        py = int((-y * fov) / z) + self.height // 2 + 100 
        return (px, py)

    def run_loop(self):
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                for slider in self.sliders:
                    slider.handle_event(event)

            self.screen.fill((235, 235, 235)) 

            grid_size = 5
            for i in range(-grid_size, grid_size + 1):
                p1 = self.project_3d_to_2d((i*0.2, -1.0, self.floor_z))
                p2 = self.project_3d_to_2d((i*0.2, 1.0, self.floor_z))
                pygame.draw.line(self.screen, (180, 180, 180), p1, p2, 1)
                
                p3 = self.project_3d_to_2d((-1.0, i*0.2, self.floor_z))
                p4 = self.project_3d_to_2d((1.0, i*0.2, self.floor_z))
                pygame.draw.line(self.screen, (180, 180, 180), p3, p4, 1)

            shadow_corners = [
                self.project_3d_to_2d((self.hip_x, self.hip_y, self.floor_z)),
                self.project_3d_to_2d((-self.hip_x, self.hip_y, self.floor_z)),
                self.project_3d_to_2d((-self.hip_x, -self.hip_y, self.floor_z)),
                self.project_3d_to_2d((self.hip_x, -self.hip_y, self.floor_z))
            ]
            pygame.draw.polygon(self.screen, (200, 180, 240), shadow_corners) 
            pygame.draw.polygon(self.screen, (100, 80, 140), shadow_corners, 2) 

            com_top = self.project_3d_to_2d((0, 0, 0))
            com_bottom = self.project_3d_to_2d((0, 0, self.floor_z))
            pygame.draw.line(self.screen, (50, 50, 50), com_top, com_bottom, 1)
            pygame.draw.circle(self.screen, (100, 100, 100), com_top, 8) 
            pygame.draw.circle(self.screen, (0, 150, 150), com_bottom, 5) 

            foot_positions = []
            for i in range(4):
                pts_3d = self.forward_kinematics(i)
                foot_positions.append(pts_3d[3]) 
                pts_2d = [self.project_3d_to_2d(p) for p in pts_3d]
                
                pygame.draw.lines(self.screen, (220, 20, 40), False, pts_2d, 6) 
                for p in pts_2d:
                    pygame.draw.circle(self.screen, (20, 20, 20), p, 3)

            body_corners = [
                self.project_3d_to_2d((self.hip_x, self.hip_y, 0)),
                self.project_3d_to_2d((-self.hip_x, self.hip_y, 0)),
                self.project_3d_to_2d((-self.hip_x, -self.hip_y, 0)),
                self.project_3d_to_2d((self.hip_x, -self.hip_y, 0))
            ]
            pygame.draw.polygon(self.screen, (30, 100, 220), body_corners, 12) 

            for slider in self.sliders:
                slider.draw(self.screen, self.font)

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Pygame3DVisualizer()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()