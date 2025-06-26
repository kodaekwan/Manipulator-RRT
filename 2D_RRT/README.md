# 🧭 RRT-Based Path Planning with Visualization

This repository provides simple yet illustrative implementations of:

- `Randomly-Exploring Random Tree (RRT)`
- `Bidirectional RRT (Bi-RRT)`
- `RRT-Connect`
- A map generator with random obstacles
- Real-time visualization and GIF export of the planning process

---

## 📁 Repository Structure

```
📂 2D_RRT/
├── generate_random_map.py   # 장애물 맵 생성기
├── vanilla_RRT.py           # 단방향 RRT with visualization
├── bidirectional_RRT.py     # 양방향 Bi-RRT with GIF 저장
├── RRT_Connect.py           # RRT-Connect 알고리즘 구현
├── map.png                  # 예시 맵 이미지 (흑백)
└── README.md                # 사용 설명서
```

---

## 🧱 Requirements

Install the required Python packages:

```bash
pip install numpy opencv-python imageio
```

---

## 🗺️ 1. Generate Obstacle Map

Run this to create a random 2D obstacle map:

```bash
python generate_random_map.py
```

- Saves a `map.png` (512×512) with random rectangles and circles as obstacles (black on white).
- Used as input for all RRT algorithms.

---

## 🚀 2. Run RRT Algorithms

### ▶️ Basic RRT (Single Tree)

```bash
python vanilla_RRT.py
```

- Expands tree from `start` toward `goal` using random sampling
- Visualized in real-time using OpenCV

---

### 🔁 Bi-RRT (Bidirectional RRT)

```bash
python bidirectional_RRT.py
```

- Alternates expansion from both `start` and `goal` trees
- Connects trees when close enough
- Saves result as animated GIF (`bi_rrt.gif`)

---

### 🔀 RRT-Connect

```bash
python RRT_Connect.py
```

- Aggressive connection strategy: one tree extends toward the other repeatedly until blocked
- Efficient for narrow passage environments
- Saves animation as `rrt_connect.gif`

---

## 🖼️ Output Example

> To be added:
> - `bi_rrt.gif`
> - `rrt_connect.gif`

You can replace this section with actual example GIFs (using `![caption](path)` syntax).

---

## ✍️ Author

**Daekwan Ko (고대관)**  
Ph.D. Student in Robotics/AI  
📧 daekwanko@dgu.ac.kr

---

## 📝 License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE) for details.
