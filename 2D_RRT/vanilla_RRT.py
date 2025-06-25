# ================================================
# Author     : Daekwan Ko (고대관)
# Affiliation: Ph.D. Student in Robotics/AI
# E-mail     : kodaekwan@gmail.com
# Date       : 2025-06-25
# Description: RRT (Rapidly-exploring Random Tree) 구현 예제
#              - 실시간 시각화 포함
#              - 2D 맵에서 경로 탐색
# ================================================

import numpy as np
import cv2
import random
import math
import time

# 노드 클래스: 좌표(x, y)와 부모 노드 정보 포함
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

# 두 노드 간 유클리드 거리 계산
def distance(p1, p2):
    return math.hypot(p2.x - p1.x, p2.y - p1.y)

# 이미지 크기 안에서 랜덤한 점 생성
def get_random_point(width, height):
    return Node(random.randint(0, width - 1), random.randint(0, height - 1))

# 두 점 사이에 장애물(검은색)이 있는지 확인
def is_collision(p1, p2, map_img):
    steps = int(distance(p1, p2))
    for i in range(steps):
        u = i / steps
        x = int(p1.x * (1 - u) + p2.x * u)
        y = int(p1.y * (1 - u) + p2.y * u)
        if x < 0 or y < 0 or x >= map_img.shape[1] or y >= map_img.shape[0] or map_img[y, x] == 0:
            return True  # 이미지 밖이거나 검은 픽셀(장애물)이면 충돌
    return False

# 기존 노드 중 랜덤 점과 가장 가까운 노드 찾기
def nearest_node(nodes, point):
    return min(nodes, key=lambda node: distance(node, point))

# from_node에서 to_point 방향으로 일정 거리만큼 확장
def steer(from_node, to_point, step_size=10):
    d = distance(from_node, to_point)
    if d < step_size:
        return to_point  # 이미 가까우면 바로 도달
    else:
        theta = math.atan2(to_point.y - from_node.y, to_point.x - from_node.x)
        return Node(int(from_node.x + step_size * math.cos(theta)),
                    int(from_node.y + step_size * math.sin(theta)))

# 트리 구조 시각화 (파란색 선으로 노드 연결)
def draw_tree(image, nodes):
    for node in nodes:
        if node.parent:
            cv2.line(image, (node.x, node.y), (node.parent.x, node.parent.y), (255, 0, 0), 1)

# 경로 시각화 (목표 노드에서 시작 노드까지 빨간 선으로 표시)
def draw_path(image, end_node):
    node = end_node
    while node.parent:
        cv2.line(image, (node.x, node.y), (node.parent.x, node.parent.y), (0, 0, 255), 2)
        node = node.parent

# RRT 알고리즘의 실시간 시각화 구현
def rrt_realtime(map_img, start, goal, max_iter=5000):
    height, width = map_img.shape
    canvas = cv2.cvtColor(map_img.copy(), cv2.COLOR_GRAY2BGR)  # 컬러로 변환해서 시각화용으로 사용
    nodes = [start]

    for i in range(max_iter):
        frame = canvas.copy()  # 매 반복마다 프레임 복사

        # 1. 랜덤 샘플 점 찍기 (노란 점)
        rand_point = get_random_point(width, height)
        cv2.circle(frame, (rand_point.x, rand_point.y), 2, (0, 255, 255), -1)

        # 2. 트리 내에서 가장 가까운 노드 찾기
        nearest = nearest_node(nodes, rand_point)

        # 3. 해당 방향으로 일정 거리(step_size) 확장
        step_size = 20
        new_node = steer(nearest, rand_point, step_size)

        # 4. 장애물 충돌 체크 후 확장
        if not is_collision(nearest, new_node, map_img):
            new_node.parent = nearest
            nodes.append(new_node)

            # 확장된 경로 선 그리기 (파란색)
            cv2.line(frame, (new_node.x, new_node.y), (nearest.x, nearest.y), (255, 0, 0), 1)

            # 목표에 도달하면 경로 그리고 종료
            goal_reach_dist = 30
            if distance(new_node, goal) < goal_reach_dist:
                goal.parent = new_node
                nodes.append(goal)
                draw_path(frame, goal)  # 최종 경로 빨간색으로 표시
                cv2.imshow("RRT Realtime", frame)
                cv2.waitKey(0)
                print("Goal reached!")
                break

        # 매 프레임마다 트리와 시작/목표 노드 표시
        draw_tree(frame, nodes)
        cv2.circle(frame, (start.x, start.y), 5, (0, 255, 0), -1)  # 시작점 (녹색)
        cv2.circle(frame, (goal.x, goal.y), 5, (0, 0, 255), -1)    # 목표점 (빨간색)

        # 프레임 보여주기 (33ms 대기)
        cv2.imshow("RRT Realtime", frame)
        key = cv2.waitKey(33)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

# ===== 실행 코드 =====
map_img = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)  # 맵 이미지 로드 (흑백)
start = Node(83, 31)  # 시작 지점 지정
goal = Node(map_img.shape[1] - 10, map_img.shape[0] - 10)  # 목표 지점 지정

rrt_realtime(map_img, start, goal)  # 실시간 RRT 실행
