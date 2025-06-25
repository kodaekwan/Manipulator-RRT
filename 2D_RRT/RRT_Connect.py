# ================================================
# Author     : Daekwan Ko (고대관)
# Affiliation: Ph.D. Student in Robotics/AI
# E-mail     : kodaekwan@gmail.com
# Date       : 2025-06-25
# File       : rrt_connect.py
# Description: RRT-Connect 알고리즘 구현
#              - 양방향 트리 확장을 통한 빠른 연결 시도
#              - 장애물 맵 기반 실시간 시각화
#              - 탐색 결과를 GIF로 저장
# ================================================

import numpy as np
import cv2
import random
import math
import imageio

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(p1, p2):
    return math.hypot(p2.x - p1.x, p2.y - p1.y)

def steer(from_node, to_node, step_size=10):
    d = distance(from_node, to_node)
    if d < step_size:
        return Node(to_node.x, to_node.y)
    theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
    return Node(int(from_node.x + step_size * math.cos(theta)),
                int(from_node.y + step_size * math.sin(theta)))

def is_collision(p1, p2, map_img):
    steps = int(distance(p1, p2))
    for i in range(steps):
        u = i / steps
        x = int(p1.x * (1 - u) + p2.x * u)
        y = int(p1.y * (1 - u) + p2.y * u)
        if x < 0 or y < 0 or x >= map_img.shape[1] or y >= map_img.shape[0] or map_img[int(y), int(x)] == 0:
            return True
    return False

def connect(tree, target, map_img, step_size=10):
    nearest = min(tree, key=lambda n: distance(n, target))
    while True:
        new_node = steer(nearest, target, step_size)
        if is_collision(nearest, new_node, map_img):
            return None
        new_node.parent = nearest
        tree.append(new_node)
        if distance(new_node, target) < step_size:
            return new_node
        nearest = new_node

def extract_path(n1, n2):
    path = []
    node = n1
    while node:
        path.append((node.x, node.y))
        node = node.parent
    path = path[::-1]
    node = n2.parent
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path

def rrt_connect(map_img, start, goal, max_iter=5000, step_size=15, gif_path="rrt_connect.gif"):
    h, w = map_img.shape
    canvas = cv2.cvtColor(map_img.copy(), cv2.COLOR_GRAY2BGR)
    tree_a = [start]
    tree_b = [goal]
    frames = []

    for i in range(max_iter):
        rand = Node(random.randint(0, w - 1), random.randint(0, h - 1))

        # Swap trees every iteration
        if i % 2 == 0:
            Ta, Tb = tree_a, tree_b
        else:
            Ta, Tb = tree_b, tree_a

        nearest = min(Ta, key=lambda n: distance(n, rand))
        new_node = steer(nearest, rand, step_size)
        if not is_collision(nearest, new_node, map_img):
            new_node.parent = nearest
            Ta.append(new_node)

            connect_result = connect(Tb, new_node, map_img, step_size)
            if connect_result:
                path = extract_path(new_node, connect_result)

                # 시각화
                frame = canvas.copy()
                for i in range(1, len(path)):
                    cv2.line(frame, path[i - 1], path[i], (0, 0, 255), 2)
                cv2.circle(frame, (start.x, start.y), 5, (0, 255, 0), -1)
                cv2.circle(frame, (goal.x, goal.y), 5, (255, 0, 0), -1)
                frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                cv2.imshow("RRT-Connect", frame)
                cv2.waitKey(0)
                imageio.mimsave(gif_path, frames, duration=0.033)
                print(f"🎉 연결 성공! GIF 저장 완료: {gif_path}")
                return

        # 트리 시각화
        frame = canvas.copy()
        for tree in [tree_a, tree_b]:
            for node in tree:
                if node.parent:
                    cv2.line(frame, (node.x, node.y), (node.parent.x, node.parent.y), (255, 0, 0), 1)
        cv2.circle(frame, (start.x, start.y), 5, (0, 255, 0), -1)
        cv2.circle(frame, (goal.x, goal.y), 5, (255, 0, 0), -1)
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        cv2.imshow("RRT-Connect", frame)
        if cv2.waitKey(33) == ord('q'):
            break

    cv2.destroyAllWindows()
    imageio.mimsave(gif_path, frames, duration=0.033)
    print("❌ 최대 반복 도달. 경로를 찾지 못했습니다. GIF 저장됨.")

# ===== 실행 코드 =====
map_img = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)
start = Node(83, 31)
goal = Node(map_img.shape[1] - 10, map_img.shape[0] - 10)

rrt_connect(map_img, start, goal)
