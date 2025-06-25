# ================================================
# Author     : Daekwan Ko (고대관)
# Affiliation: Ph.D. Student in Robotics/AI
# E-mail     : kodaekwan@gmail.com
# Date       : 2025-06-25
# Description: RRT 테스트용 랜덤 장애물 맵 생성기
#              - 흰 배경에 사각형 및 원형 장애물 삽입
#              - 결과 이미지를 PNG로 저장
# ================================================

import cv2
import numpy as np
import random

def generate_rrt_map(width=512, height=512, num_rects=10, num_circles=10, save_path='map.png'):
    # 흰 배경 초기화
    map_img = np.ones((height, width), dtype=np.uint8) * 255

    # 랜덤 사각형 장애물
    for _ in range(num_rects):
        x1 = random.randint(0, width - 50)
        y1 = random.randint(0, height - 50)
        x2 = x1 + random.randint(20, 80)
        y2 = y1 + random.randint(20, 80)
        cv2.rectangle(map_img, (x1, y1), (min(x2, width - 1), min(y2, height - 1)), 0, -1)

    # 랜덤 원형 장애물
    for _ in range(num_circles):
        center = (random.randint(0, width - 1), random.randint(0, height - 1))
        radius = random.randint(10, 40)
        cv2.circle(map_img, center, radius, 0, -1)

    # 저장
    cv2.imwrite(save_path, map_img)
    print(f"Map saved to {save_path}")

# 실행
if __name__ == "__main__":
    generate_rrt_map(save_path="map.png")
