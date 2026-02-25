import numpy as np
import cv2

# 체커보드 설정 (calibrate_ir.py와 동일하게 내부 코너 9x6)
INNER_COLS = 9
INNER_ROWS = 6
SQUARE_SIZE_PX = 80  # 각 사각형 크기 (픽셀)

# 외곽 포함 실제 사각형 수
cols = INNER_COLS + 1  # 10
rows = INNER_ROWS + 1  # 7

img_w = cols * SQUARE_SIZE_PX
img_h = rows * SQUARE_SIZE_PX
board = np.zeros((img_h, img_w), dtype=np.uint8)

for r in range(rows):
    for c in range(cols):
        if (r + c) % 2 == 0:
            y1 = r * SQUARE_SIZE_PX
            y2 = y1 + SQUARE_SIZE_PX
            x1 = c * SQUARE_SIZE_PX
            x2 = x1 + SQUARE_SIZE_PX
            board[y1:y2, x1:x2] = 255

# 여백 추가
MARGIN = 40
result = np.full((img_h + MARGIN * 2, img_w + MARGIN * 2), 255, dtype=np.uint8)
result[MARGIN:MARGIN + img_h, MARGIN:MARGIN + img_w] = board

save_path = r"c:\0.shinhyoung\Project\realsenseTest\checkerboard_9x6.png"
cv2.imwrite(save_path, result)
print(f"Checkerboard saved: {save_path}")
print(f"  Inner corners: {INNER_COLS}x{INNER_ROWS}")
print(f"  Image size: {result.shape[1]}x{result.shape[0]} px")
print(f"\nA4 용지에 인쇄하거나 모니터에 띄워서 사용하세요.")
