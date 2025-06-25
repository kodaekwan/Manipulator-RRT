# ================================================
# Author     : kodaekwan
# Affiliation: Ph.D. Student in Robotics/AI
# E-mail     : kodaekwan@gmail.com
# Date       : 2025-06-25
# File       : manipulator_rrt_connect.py
    # RRT-Connect 메인 함수: 트리 번갈아 확장하며 두 트리 연결 시도
# Description: RRT-Connect 기반 다자유도 매니퓰레이터 경로 계획
#              - PyBullet 기반 조인트 공간 + 작업 공간 확장
#              - 충돌 체크 및 FK 기반 위치 추적
#              - 쌍방 트리 교차 연결 및 경로 출력
# ================================================
import numpy as np
import pybullet as p
import random
import math

# 안전하게 리스트 형태로 벡터 변환
def safe_vec3(x):
    return [float(v) for v in (x.tolist() if hasattr(x, "tolist") else x)]

# 쿼터니언 형태 보장 및 변환
def safe_quat(q):
    assert len(q) == 4, "targetOrientation must be a quaternion of length 4"
    return [float(v) for v in (q.tolist() if hasattr(q, "tolist") else q)]

# RRT 노드 클래스 (조인트 상태와 부모 연결)
class Node:
    def __init__(self, joint_pos):
        self.joint_pos = joint_pos
        self.parent = None

# 작업공간 거리 계산 (엔드 이펙터 위치 기준)
def distance(node: Node, point: np.ndarray, robot_id, joint_indices, end_effector_link_index):
    current_states = [p.getJointState(robot_id, i)[0] for i in joint_indices]
    try:
        for i, q in zip(joint_indices, node.joint_pos):
            p.resetJointState(robot_id, i, q)
        link_state = p.getLinkState(robot_id, end_effector_link_index, computeForwardKinematics=True)
        if not link_state or len(link_state) < 5:
            raise RuntimeError(f"getLinkState failed for link index {end_effector_link_index}")
        pos = link_state[4]
        return np.linalg.norm(np.array(pos) - point)
    finally:
        for i, q in zip(joint_indices, current_states):
            p.resetJointState(robot_id, i, q)

# 조인트 상태로 엔드이펙터 위치 얻기
def get_end_effector_pos(robot_id, joint_indices, joint_angles, end_effector_link_index):
    current_states = [p.getJointState(robot_id, i)[0] for i in joint_indices]
    try:
        for i, q in zip(joint_indices, joint_angles):
            p.resetJointState(robot_id, i, q)
        link_state = p.getLinkState(robot_id, end_effector_link_index, computeForwardKinematics=True)
        if not link_state or len(link_state) < 5:
            raise RuntimeError(f"getLinkState failed for link index {end_effector_link_index}")
        pos = link_state[4]
        return np.array(pos)
    finally:
        for i, q in zip(joint_indices, current_states):
            p.resetJointState(robot_id, i, q)

# 트리 내 가장 가까운 노드 탐색 (작업공간 기준)
def nearest_node(nodes, point, robot_id, joint_indices, end_effector_link_index):
    return min(nodes, key=lambda node: distance(node, point, robot_id, joint_indices, end_effector_link_index))

# 목표 노드까지 경로 생성 (역추적)
def build_path(goal_node):
    path = []
    node = goal_node
    while node:
        path.append(node.joint_pos)
        node = node.parent
    return path[::-1]

# 충돌 체크
def check_collision_stateless(robot_id, joint_indices, joint_node, obstacle_ids):
    joint_angles = joint_node.joint_pos
    current_states = [p.getJointState(robot_id, i)[0] for i in joint_indices]
    for i, q in zip(joint_indices, joint_angles):
        p.resetJointState(robot_id, i, q)
    p.performCollisionDetection()
    contacts = p.getContactPoints(bodyA=robot_id)
    for contact in contacts:
        if contact[2] in obstacle_ids or (contact[1] == contact[2] and contact[3] != contact[4]):
            for i, q in zip(joint_indices, current_states):
                p.resetJointState(robot_id, i, q)
            return True
    for i, q in zip(joint_indices, current_states):
        p.resetJointState(robot_id, i, q)
    return False

# 샘플링 기반 트리 확장
def extend_toward_sample(from_tree, sample_pos, step_size, robot_id, joint_indices, obstacle_ids, end_effector_link_index, max_extend_steps=10):
    nearest = nearest_node(from_tree, sample_pos, robot_id, joint_indices, end_effector_link_index)
    nearest_pos = get_end_effector_pos(robot_id, joint_indices, nearest.joint_pos, end_effector_link_index)

    direction = sample_pos - nearest_pos
    norm = np.linalg.norm(direction)
    if norm == 0:
        return None
    direction /= norm

    last_node = nearest
    for step in range(1, max_extend_steps + 1):
        new_pos = nearest_pos + step * step_size * direction
        joint_angles = p.calculateInverseKinematics(robot_id, end_effector_link_index, new_pos)
        new_node = Node(np.array(joint_angles[:len(joint_indices)]))
        new_node.parent = last_node

        if check_collision_stateless(robot_id, joint_indices, new_node, obstacle_ids):
            return None

        from_tree.append(new_node)
        last_node = new_node

        if np.linalg.norm(get_end_effector_pos(robot_id, joint_indices, new_node.joint_pos, end_effector_link_index) - sample_pos) < step_size:
            return new_node

    return last_node

# 작업공간 내 임의 위치 샘플링
def sample_in_workspace(center=np.array([0.4, 0, 0.3]), radius_range=(0.2, 0.6), z_range=(0.1, 0.5)):
    r = np.random.uniform(*radius_range)
    theta = np.random.uniform(-np.pi, np.pi)
    z = np.random.uniform(*z_range)
    x = center[0] + r * np.cos(theta)
    y = center[1] + r * np.sin(theta)
    return np.array([x, y, z])

# RRT-Connect 경로 계획
def manipulator_rrt_connect(robot_id, joint_indices, obstacle_ids, target_pos, target_ori,
                             end_effector_link_index=7, max_iter=2000, step_size=0.05, max_extend_steps=10):
    joint_dim = len(joint_indices)
    joint_limits = [p.getJointInfo(robot_id, i)[8:10] for i in joint_indices]
    joint_limits = np.array(joint_limits)
    low, high = joint_limits[:, 0], joint_limits[:, 1]

    start_joint = np.array([p.getJointState(robot_id, i)[0] for i in joint_indices])
    start_node = Node(start_joint)
    tree_start = [start_node]

    ik_result = p.calculateInverseKinematics(robot_id, end_effector_link_index,
                                             safe_vec3(target_pos), safe_quat(target_ori),
                                             jointDamping=[0.1]*len(joint_indices),
                                             currentPositions=start_joint.tolist())
    
    goal_joint = np.array([ik_result[i] for i in range(len(joint_indices))])
    goal_node = Node(goal_joint)
    tree_goal = [goal_node]

    for i in range(max_iter):
        rand_pos = sample_in_workspace()

        if i % 2 == 0:
            tree_a, tree_b = tree_start, tree_goal
        else:
            tree_a, tree_b = tree_goal, tree_start

        new_node_a = extend_toward_sample(tree_a, rand_pos, step_size, robot_id, joint_indices, obstacle_ids, end_effector_link_index, max_extend_steps)
        if new_node_a is None:
            continue

        new_node_b = extend_toward_sample(tree_b, get_end_effector_pos(robot_id, joint_indices, new_node_a.joint_pos, end_effector_link_index),
                                          step_size, robot_id, joint_indices, obstacle_ids, end_effector_link_index, max_extend_steps)
        if new_node_b is None:
            continue

        dist = np.linalg.norm(get_end_effector_pos(robot_id, joint_indices, new_node_a.joint_pos, end_effector_link_index) -
                              get_end_effector_pos(robot_id, joint_indices, new_node_b.joint_pos, end_effector_link_index))
        if dist < step_size:
            path_a = build_path(new_node_a)
            path_b = build_path(new_node_b)
            if tree_a is tree_start:
                return path_a + path_b[::-1][1:]
            else:
                return path_b + path_a[::-1][1:]

    return None

def draw_rrt_path(robot_id, joint_indices, path, end_effector_link_index=7, color=[0, 1, 0], life_time=0):
    """
    RRT 경로를 시각화합니다. 각 관절 구성을 따라 FK 계산하여 선을 그림.
    :param robot_id: 로봇 ID
    :param joint_indices: 관절 인덱스 리스트
    :param path: 경로 (관절 각도 리스트들의 리스트)
    :param end_effector_link_index: 엔드이펙터 링크 번호
    :param color: RGB 색
    :param life_time: 선이 사라지기까지 시간 (0이면 무한)
    """
    if len(path) < 2:
        return

    ee_positions = []

    for joint_angles in path:
        for i, q in zip(joint_indices, joint_angles):
            p.resetJointState(robot_id, i, q)
        pos, _ = p.getLinkState(robot_id, end_effector_link_index)[4:6]
        ee_positions.append(pos)

    for i in range(len(ee_positions) - 1):
        p.addUserDebugLine(ee_positions[i], ee_positions[i+1], lineColorRGB=color, lineWidth=2.0, lifeTime=life_time)
