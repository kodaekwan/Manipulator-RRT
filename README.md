# 🤖 Manipulator-RRT with PyBullet + 2D RRT Examples

This repository contains implementations of:

- ✅ 2D RRT planners with map generation and visualization
- ✅ Manipulator-based RRT-Connect planner (UR5e in PyBullet)
- ✅ Real-time simulation and trajectory execution
- ✅ Obstacle-aware motion planning for robot arms

---

## 📂 Folder Structure

```
├── Manipulator-RRT/
├── 2D_RRT/ # 2D RRT 구현 (OpenCV 기반)
│ ├── bidirectional_RRT.py
│ ├── generate_random_map.py
│ ├── vanilla_RRT.py
│ ├── map.png # 랜덤 생성된 2D 맵 이미지
│ └── RRT_Connect.py # 2D RRT-Connect 구현
│
├── manipulator_rrt_connect.py # PyBullet용 매니퓰레이터 RRT-Connect
├── manipulator_rrt_connect_visual.py # PyBullet용 매니퓰레이터 RRT-Connect, 중간 트리 구조 시각화
├── run_rrt_connect_simulation.py # 전체 실행 스크립트 (UR5e 환경)
├── robots/
│    └── urdf/
│           └── ur5e.urdf # UR5e URDF 파일 위치
├── plane.urdf # 바닥 평면 URDF
└── README.md
```

## ⚙️ Environment Setup

### 📦 Python Dependencies

```bash
python -m pip install --upgrade pip setuptools wheel
pip install pybullet numpy opencv-python imageio
```

---

## 🚀 How to Run

### 1. Generate a 2D Obstacle Map

```bash
cd 2D_RRT
python generate_random_map.py
```

### 2. Run 2D RRT or Bi-RRT

```bash
python vanilla_RRT.py          # 단방향
python bidirectional_RRT.py    # 양방향
```

### 3. Run Manipulator RRT-Connect (UR5e + PyBullet)

```bash
cd ..
python run_rrt_connect_simulation.py
```

- Initializes PyBullet GUI with UR5e
- Visualizes sampled path using `draw_rrt_path`
- Executes planned trajectory to reach the goal
- Avoids collision with obstacles (box-shaped)

---

## 📌 Features

- Modular structure (2D and manipulator planners separated)
- Collision-aware planning using PyBullet contact checks
- Real-time trajectory execution with position control
- Easy to integrate into other PyBullet robot scenarios

---

## ✍️ Author

**Daekwan Ko (고대관)**  
Ph.D. Student in Robotics/AI  
📧 kodaekwan@gmail.com

---

## 📝 License

This project is released under the MIT License. See [`LICENSE`](LICENSE) for details.
