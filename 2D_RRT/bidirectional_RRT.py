# ================================================
# Author     : Daekwan Ko (고대관)
# Affiliation: Ph.D. Student in Robotics/AI
# E-mail     : daekwanko@dgu.ac.kr
# Date       : 2025-06-25
# Description: Bidirectional RRT (Bi-RRT) 알고리즘
#              - 2D 맵에서 시작점과 목표점을 각각 확장
#              - 두 트리가 연결될 경우 경로 추출 및 시각화
#              - GIF로 탐색 과정을 저장
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

def get_random_point(width, height):
    return Node(random.randint(0, width - 1), random.randint(0, height - 1))

def is_collision(p1, p2, map_img):
    steps = int(distance(p1, p2))
    for i in range(steps):
        u = i / steps
        x = int(p1.x * (1 - u) + p2.x * u)
        y = int(p1.y * (1 - u) + p2.y * u)
        if x < 0 or y < 0 or x >= map_img.shape[1] or y >= map_img.shape[0] or map_img[y, x] == 0:
            return True
    return False

def nearest_node(nodes, point):
    return min(nodes, key=lambda node: distance(node, point))

def steer(from_node, to_point, step_size=10):
    d = distance(from_node, to_point)
    if d < step_size:
        return to_point
    else:
        theta = math.atan2(to_point.y - from_node.y, to_point.x - from_node.x)
        return Node(int(from_node.x + step_size * math.cos(theta)),
                    int(from_node.y + step_size * math.sin(theta)))

def extract_path(node_start, node_goal):
    # 양방향 트리 연결된 지점 기준으로 역추적
    path = []
    node = node_start
    while node:
        path.append((node.x, node.y))
        node = node.parent
    path = path[::-1]  # 시작→연결점

    node = node_goal.parent  # 연결점 중복 제외
    while node:
        path.append((node.x, node.y))
        node = node.parent

    return path

def bi_rrt(map_img, start, goal, max_iter=5000, step_size=20, connect_dist=15, gif_path="bi_rrt.gif"):
    height, width = map_img.shape
    canvas = cv2.cvtColor(map_img.copy(), cv2.COLOR_GRAY2BGR)

    tree_start = [start]
    tree_goal = [goal]
    frames = []

    for i in range(max_iter):
        # 랜덤 점 생성
        rand_point = get_random_point(width, height)

        # 트리 선택 (짝수: start 트리 확장, 홀수: goal 트리 확장)
        tree_a = tree_start if i % 2 == 0 else tree_goal
        tree_b = tree_goal if i % 2 == 0 else tree_start

        nearest_a = nearest_node(tree_a, rand_point)
        new_node_a = steer(nearest_a, rand_point, step_size)

        if not is_collision(nearest_a, new_node_a, map_img):
            new_node_a.parent = nearest_a
            tree_a.append(new_node_a)

            # tree_b에서 가장 가까운 노드로 연결 시도
            nearest_b = nearest_node(tree_b, new_node_a)
            if distance(nearest_b, new_node_a) < connect_dist and not is_collision(nearest_b, new_node_a, map_img):
                # 연결 성공
                new_node_b = Node(nearest_b.x, nearest_b.y)
                new_node_b.parent = nearest_b
                tree_b.append(new_node_b)

                path = extract_path(new_node_a, new_node_b)

                # 시각화 마지막 프레임
                frame = canvas.copy()
                for i in range(1, len(path)):
                    cv2.line(frame, path[i - 1], path[i], (0, 0, 255), 2)

                cv2.circle(frame, (start.x, start.y), 5, (0, 255, 0), -1)
                cv2.circle(frame, (goal.x, goal.y), 5, (255, 0, 0), -1)
                frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                cv2.imshow("Bi-RRT", frame)
                cv2.waitKey(0)
                imageio.mimsave(gif_path, frames, duration=0.033)
                print(f"Goal connected. GIF saved: {gif_path}")
                return

        # 트리 전체 다시 그림
        frame = canvas.copy()

        for tree in [tree_start, tree_goal]:
            for node in tree:
                if node.parent:
                    cv2.line(frame, (node.x, node.y), (node.parent.x, node.parent.y), (255, 0, 0), 2)

        cv2.circle(frame, (start.x, start.y), 5, (0, 255, 0), -1)
        cv2.circle(frame, (goal.x, goal.y), 5, (255, 0, 0), -1)
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        cv2.imshow("Bi-RRT", frame)
        key = cv2.waitKey(33)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    imageio.mimsave(gif_path, frames, duration=0.033)
    print("최대 반복 도달. 경로 미완성. GIF 저장 완료.")

# ===== 실행 =====
map_img = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)
start = Node(83, 31)
goal = Node(map_img.shape[1] - 10, map_img.shape[0] - 10)

bi_rrt(map_img, start, goal)
