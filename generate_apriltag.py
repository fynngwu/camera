import cv2
import os

# 生成一个 tag36h11 家族中的 AprilTag
tag_id = 0                 # 标签编号，按需改
side_pixels = 400          # 输出图像大小（像素）
border_bits = 1            # 黑边宽度，通常用 1
quiet_zone_pixels = 100    # 外侧白边，pupil_apriltags 检测更稳定
save_path = f"tag36h11_{tag_id}.png"

# 获取预定义字典：AprilTag 36h11
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# 生成 marker 图像（旧版OpenCV使用drawMarker，位置参数）
img = cv2.aruco.drawMarker(dictionary, tag_id, side_pixels, border_bits)
img = cv2.copyMakeBorder(
    img,
    quiet_zone_pixels,
    quiet_zone_pixels,
    quiet_zone_pixels,
    quiet_zone_pixels,
    cv2.BORDER_CONSTANT,
    value=255,
)

# 保存
cv2.imwrite(save_path, img)

print(f"Saved to {os.path.abspath(save_path)}")
