# ================================================
# Author     : kodaekwan
# Affiliation: Ph.D. Student in Robotics/AI
# E-mail     : kodaekwan@gmail.com
# Date       : 2025-06-25
# File       : run_rrt_connect_simulation.py
# Description: UR5e 로봇에 대해 RRT-Connect 기반 경로 생성 및 시각화
#              - PyBullet 시뮬레이션 환경 구성
#              - 시작/목표 위치 시각화 및 이동
#              - 장애물 포함한 충돌 회피 경로 계획 및 재생
# ================================================
import os
import time
import pybullet as p
import pybullet_data
from manipulator_rrt_connect import manipulator_rrt_connect, draw_rrt_path
#from manipulator_rrt_connect_visual import manipulator_rrt_connect, draw_rrt_path
import numpy as np

# PyBullet 초기화
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath("robots")  # URDF 상대 경로 지원

# 바닥 추가
plane_id = p.loadURDF("plane.urdf")


# 시작지점 정의
start_pos = [0.4, 0.1, 0.5]
start_ori = list(p.getQuaternionFromEuler([np.deg2rad(0.0), 0, 0]));
start_config = list(start_pos+start_ori);

# 시작지점 시각화
start_id = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 1]),
    basePosition=start_pos
)

# 시뮬레이션 후 실제 구 위치 확인
sphere_pos = p.getBasePositionAndOrientation(start_id)[0]
print("시각화된 구 위치:", sphere_pos)

# UR5e 로드
urdf_path = os.path.join(os.getcwd(), "robots/urdf/ur5e.urdf")
ur5e_start_pos = [0.0, 0.0, 0.01]
ur5e_start_ori = p.getQuaternionFromEuler([0.0, 0, 0])
ur5e_id = p.loadURDF(
    urdf_path,
    ur5e_start_pos,
    ur5e_start_ori,
    useFixedBase=True,
    flags=p.URDF_USE_SELF_COLLISION
)

end_effector_link_index = 7


# 도착지점 정의
goal_pos = [0.5, 0.0, 0.25]
goal_ori = list(p.getQuaternionFromEuler([0.0, 0.0, 0.0]));
#goal_config = list(goal_pos+goal_ori);

# 도착지점 시각화
goal_id = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 0, 1, 1]),
    basePosition=goal_pos
)

# 도착지점 리셋위치
joint_indices =  [1,2,3,4,5,6]
joint_positions = list(p.calculateInverseKinematics(ur5e_id, end_effector_link_index, goal_pos, goal_ori,maxNumIterations=1000,residualThreshold=1e-3))
for idx, pos in zip(joint_indices, joint_positions):
    p.resetJointState(ur5e_id, idx, pos)



# 시뮬레이션 유지(시작지점 확인용)
s=time.time()
while p.isConnected():
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    if(time.time()-s>3.0):
        break

for idx, pos in zip(joint_indices, [0.0 for _ in range(6)]):
    p.resetJointState(ur5e_id, idx, pos)

# 시작지점 리셋위치
joint_indices =  [1,2,3,4,5,6]
joint_positions = list(p.calculateInverseKinematics(ur5e_id, end_effector_link_index, start_pos, start_ori,maxNumIterations=1000,residualThreshold=1e-3))
for idx, pos in zip(joint_indices, joint_positions):
    p.resetJointState(ur5e_id, idx, pos)

# 시뮬레이션 유지(시작지점 확인용)
s=time.time()
while p.isConnected():
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    if(time.time()-s>3.0):
        break


# 장애물 생성
obstacle_shape1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.001],)
obstacle_shape2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.001],)
obstacle_shape3 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.001, 0.1],)
obstacle_id1 = p.createMultiBody(0, obstacle_shape1, basePosition=[0.6, 0.2, 0.2])
obstacle_id2 = p.createMultiBody(0, obstacle_shape2, basePosition=[0.5, 0.0, 0.4])
obstacle_id3 = p.createMultiBody(0, obstacle_shape3, basePosition=[0.4, -0.3, 0.3])
obstacle_ids = [obstacle_id1, obstacle_id2,obstacle_id3,plane_id]



# 시작 상태
start_config = [p.getJointState(ur5e_id, i)[0] for i in joint_indices]




# RRT 실행
print("\n🔍 경로 탐색 중...")
path = manipulator_rrt_connect(
    robot_id=ur5e_id,
    joint_indices=[1,2,3,4,5,6],
    obstacle_ids=obstacle_ids,
    target_pos=goal_pos,
    target_ori=goal_ori,
    end_effector_link_index=7,
    max_iter = 50000,
    step_size = 0.01
     
)


if path is None:
    print("❌ 경로 탐색 실패")
else:
    print("✅ 경로 탐색 성공, 관절 경로 길이:", len(path))

    # 시작지점 리셋위치
    joint_indices =  [1,2,3,4,5,6]
    joint_positions = list(p.calculateInverseKinematics(ur5e_id, end_effector_link_index, start_pos, start_ori,maxNumIterations=1000,residualThreshold=1e-3))
    for idx, pos in zip(joint_indices, joint_positions):
        p.resetJointState(ur5e_id, idx, pos)

    draw_rrt_path(robot_id=ur5e_id, joint_indices=joint_indices, path=path, end_effector_link_index=end_effector_link_index)

    # 경로 따라가기
    for q in path:
        for i, joint_angle in enumerate(q):
 
            p.setJointMotorControl2(
                bodyIndex=ur5e_id,
                jointIndex=joint_indices[i],
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_angle,
                force=200,
                positionGain=0.1,
                velocityGain=1.0
            )
        for _ in range(30):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)


    print("🎯 목표 위치 도달 완료")

# 시뮬레이션 유지
while p.isConnected():
    joint_positions = list(p.calculateInverseKinematics(ur5e_id, end_effector_link_index, goal_pos, goal_ori,maxNumIterations=1000,residualThreshold=1e-5))

    p.stepSimulation()
    time.sleep(1.0 / 240.0)
