import cv2
import numpy as np
import time
import serial
import threading

serial_lock = threading.Lock()

show_text_until = 0
text_to_show = ''
show_text_until = 0

# 串口初始化
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
listener1_enabled = threading.Event()
listener1_enabled.set()
listener2_enabled = threading.Event()
listener2_enabled.set()

# 初始化摄像头（树莓派摄像头通常索引为0或1）
cap = cv2.VideoCapture(0)
# 设置摄像头分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# 设置帧率
cap.set(cv2.CAP_PROP_FPS, 30)

# 矩形检测参数
MIN_AREA = 9000
MAX_AREA = 80000
pause_rect_detect = False  # 是否暂停矩形检测
Shape_Detection = False  # 是否检测形状
K = 253000  # 距离计算常数

# 面积采样参数
mianji_samples = []  # 存储面积采样数据
sample_count = 0  # 当前采样计数
TOTAL_SAMPLES = 3  # 总采样次数
TRIM_SAMPLES = 2  # 需要去除的最大最小值数量

distance_display_start = 0


def sort_rect_points(pts):
    """对矩形四个顶点进行排序：左上、右上、左下、右下"""
    pts = sorted(pts, key=lambda p: (p[1], p[0]))
    top = sorted(pts[:2], key=lambda p: p[0])
    bottom = sorted(pts[2:], key=lambda p: p[0], reverse=True)
    return [top[0], top[1], bottom[0], bottom[1]]


def match_corners_by_distance(ref_pts, target_pts):
    """通过距离匹配矩形角点"""
    matched = [None] * 4
    used = [False] * 4
    for i, p1 in enumerate(ref_pts):
        min_dist = float("inf")
        min_j = -1
        for j, p2 in enumerate(target_pts):
            if used[j]:
                continue
            dist = np.linalg.norm(np.array(p1) - np.array(p2))
            if dist < min_dist:
                min_dist = dist
                min_j = j
        matched[i] = target_pts[min_j]
        used[min_j] = True
    return matched


def is_similar_rect(rect1, rect2, threshold=8, area_thresh=0.05):
    """判断两个矩形是否相似"""
    try:
        rect1 = sort_rect_points(rect1)
        rect2 = sort_rect_points(rect2)
        avg_dist = np.mean([np.linalg.norm(np.array(p1) - np.array(p2)) for p1, p2 in zip(rect1, rect2)])
        area1 = cv2.contourArea(np.array(rect1, dtype=np.int32))
        area2 = cv2.contourArea(np.array(rect2, dtype=np.int32))
        area_diff_ratio = abs(area1 - area2) / max(area1, area2)
        return avg_dist < threshold and area_diff_ratio < area_thresh
    except Exception as e:
        print("矩形比较异常:", e)
        return False


def is_rectangle(approx):
    """判断轮廓是否为矩形"""
    if approx is None or len(approx) != 4 or not cv2.isContourConvex(approx):
        return False
    pts = [point[0] for point in approx]

    def angle(p1, p2, p3):
        v1 = np.array(p1) - np.array(p2)
        v2 = np.array(p3) - np.array(p2)
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        if norm1 == 0 or norm2 == 0:
            return 0
        cos_angle = np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0)
        return np.arccos(cos_angle) * 180 / np.pi

    angles = [angle(pts[i - 1], pts[i], pts[(i + 1) % 4]) for i in range(4)]
    return all(80 < ang < 100 for ang in angles)


def calculate_trimmed_mean(data):
    """计算去掉TRIM_SAMPLES个最小和最大值后的平均值"""
    if len(data) <= 2 * TRIM_SAMPLES:
        return np.mean(data) if data else 0

    sorted_data = sorted(data)
    trimmed_data = sorted_data[TRIM_SAMPLES:-TRIM_SAMPLES]
    return np.mean(trimmed_data)


def serial_listener():
    global recognition_active, mianji_samples, sample_count, pause_rect_detect, Shape_Detection, print_D, distance_display_start, listener1_enabled
    while True:
        try:
            with serial_lock:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    if b'\x01' in data:
                        recognition_active = True
                        pause_rect_detect = False
                        Shape_Detection = True
                        print_D = False
                        mianji_samples = []
                        sample_count = 0
                        # print("收到0x01")
                    if b'\x02' in data:
                        show_text_until = time.time() + 5
                        listener1_enabled.set()
                        recognition_active = True
                        pause_rect_detect = False
                        Shape_Detection = False
                        print_D = True
                        # print("串口触发0x02 - 开始识别")
        except Exception as e:
            print(f"1")

        # 创建窗口


cv2.namedWindow("Detection Result", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Detection Result", 800, 600)

# 启动串口监听线程
threading.Thread(target=serial_listener, daemon=True).start()

# 主循环
recognition_active = False
while True:
    try:
        ret, img_raw = cap.read()
        if not ret:
            print("无法获取图像")
            time.sleep(1)
            continue

        midpoints = [(-1, -1)] * 4
        inner_rect_width = 0
        inner_rect_height = 0
        start = time.time()
        # 矩形检测与中点计算
        if recognition_active and not pause_rect_detect:
            try:
                # 图像处理
                gray = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edged = cv2.Canny(blurred, 50, 150)

                # 形态学操作
                kernel = np.ones((5, 5), np.uint8)
                dilated = cv2.dilate(edged, kernel, iterations=1)
                eroded = cv2.erode(dilated, kernel, iterations=1)

                # 轮廓检测
                contours, _ = cv2.findContours(eroded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                rectangles = []

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if not (MIN_AREA <= area <= MAX_AREA):
                        continue
                    x, y, w, h = cv2.boundingRect(contour)
                    margin = 1  # 边缘安全距离
                    if x < margin or y < margin or x + w > img_raw.shape[1] - margin or y + h > img_raw.shape[
                        0] - margin:
                        continue
                    approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
                    if is_rectangle(approx):
                        rect = [tuple(pt[0]) for pt in approx]
                        # 排除正方形
                        rect_points = np.array(rect, dtype=np.float32)
                        rect_box = cv2.minAreaRect(rect_points)
                        width, height = rect_box[1]
                        max_side = max(width, height)
                        min_side = min(width, height)
                        aspect_ratio = min_side / (max_side + 1e-5)
                        # 长宽比>0.85视为正方形，跳过
                        if aspect_ratio > 0.85:
                            continue
                        if not any(is_similar_rect(rect, r) for r in rectangles):
                            rectangles.append(rect)
                            cv2.drawContours(img_raw, [np.array(rect, dtype=np.int32)], -1, (0, 255, 0), 2)
                            for x, y in rect:
                                cv2.circle(img_raw, (x, y), 5, (0, 0, 255), -1)

                if len(rectangles) == 2:
                    r1 = sort_rect_points(rectangles[0])
                    r2 = sort_rect_points(rectangles[1])

                    # 计算两个矩形的面积
                    area1 = cv2.contourArea(np.array(r1, dtype=np.int32))
                    area2 = cv2.contourArea(np.array(r2, dtype=np.int32))

                    # 选择面积较小的矩形
                    smaller_rect = r1 if area1 < area2 else r2

                    # 计算较小矩形的宽高
                    rect_points = np.array(smaller_rect, dtype=np.float32)
                    rect = cv2.minAreaRect(rect_points)
                    width, height = rect[1]
                    min_rect_width = max(width, height)
                    # print(f"{min_rect_width}       shishishishishis")
                    min_rect_height = min(width, height)
                    listener1_enabled.set()
                    listener2_enabled.set()

                    # 计算距离
                    juli = K / min_rect_width
                    if print_D:
                        text_to_show = f'D: {juli / 10 - 13:.1f} cm\n'
                        print_D = False
                        # print(f"{(juli / 10 - 13)*10}mm")
                        juli = (juli / 10 - 13) * 10
                        if min_rect_width >= 156.0:
                            juli = juli + 30
                            print(f"D = {juli:.1f}mm\n")
                        if min_rect_width < 146.3:
                            juli = juli - 30
                            print(f"D = {juli:.1f}mm\n")

                    # ------------------------------------------------------------------------------------------------------------------------------
                    # 形状识别部分
                    if Shape_Detection:
                        pts = np.array(smaller_rect, dtype=np.int32)
                        x, y, w, h = cv2.boundingRect(pts)

                        # 确保ROI在图像范围内
                        if x < 0: x = 0
                        if y < 0: y = 0
                        if x + w > img_raw.shape[1]: w = img_raw.shape[1] - x
                        if y + h > img_raw.shape[0]: h = img_raw.shape[0] - y

                        if w > 0 and h > 0:
                            roi = img_raw[y:y + h, x:x + w]
                            # 形状识别处理
                            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                            _, binary_roi = cv2.threshold(gray_roi, 100, 255, cv2.THRESH_BINARY_INV)
                            contours_roi, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                            shape_info = ""
                            size_value = 0

                            if contours_roi:
                                max_contour = max(contours_roi, key=cv2.contourArea)
                                perimeter = cv2.arcLength(max_contour, True)
                                epsilon = 0.04 * perimeter
                                approx = cv2.approxPolyDP(max_contour, epsilon, True)
                                num_vertices = len(approx)
                                area = cv2.contourArea(max_contour)

                                # 形状识别
                                if num_vertices == 3:  # 三角形
                                    side1 = np.linalg.norm(approx[0][0] - approx[1][0])
                                    side2 = np.linalg.norm(approx[1][0] - approx[2][0])
                                    side3 = np.linalg.norm(approx[2][0] - approx[0][0])
                                    avg_side = (side1 + side2 + side3) / 3.0
                                    shape_info = "X"
                                    size_value = avg_side
                                    cv2.drawContours(roi, [approx], -1, (255, 0, 255), 2)

                                elif num_vertices == 4:  # 四边形
                                    side1 = np.linalg.norm(approx[0][0] - approx[1][0])
                                    side2 = np.linalg.norm(approx[1][0] - approx[2][0])
                                    side3 = np.linalg.norm(approx[2][0] - approx[3][0])
                                    side4 = np.linalg.norm(approx[3][0] - approx[0][0])
                                    avg_side = (side1 + side2 + side3 + side4) / 4.0
                                    min_side = min(side1, side2, side3, side4)
                                    max_side = max(side1, side2, side3, side4)
                                    side_ratio = min_side / max_side

                                    if side_ratio > 0.85:  # 正方形
                                        shape_info = "X"
                                        size_value = avg_side
                                        cv2.drawContours(roi, [approx], -1, (255, 0, 255), 2)


                                else:  # 圆形
                                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                                    if circularity > 0.7:
                                        (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour)
                                        diameter = radius * 2
                                        shape_info = "X"
                                        size_value = diameter
                                        center = (int(center_x), int(center_y))
                                        radius = int(radius)
                                        cv2.circle(roi, center, radius, (255, 0, 255), 2)

                                # 显示形状信息
                                if shape_info:
                                    # 计算并采集面积数据
                                    mianji_size = (size_value / 88) * (juli / 132) * 112

                                    # 采样逻辑
                                    if sample_count < TOTAL_SAMPLES:
                                        mianji_samples.append(mianji_size)
                                        sample_count += 1
                                        # print(f"采样 {sample_count}/{TOTAL_SAMPLES}: {mianji_size:.2f}")
                                    elif sample_count == TOTAL_SAMPLES:
                                        avg_mianji = calculate_trimmed_mean(mianji_samples)
                                        juli = (juli / 10 - 13) * 10
                                        if min_rect_width >= 156.0:
                                            juli = juli + 30
                                            # print(f"juli = {juli}mm")
                                        if min_rect_width < 146.3:
                                            juli = juli - 30
                                            # print(f"juli = {juli}mm")

                                        info_text = f"{shape_info}: {(mianji_size) / 10:.1f}mm"

                                        # print("\n最终平均值(去掉3个最小和最大值): {:.2f}".format(avg_mianji - 200))
                                        # print("所有采样数据:", ["{:.2f}".format(x) for x in mianji_samples])
                                        cv2.imshow("Detection Result", img_raw)
                                        # cv2.waitKey(5000)  # 显示3秒
                                        print(f"D = {juli:.1f}mm")
                                        if num_vertices == 3:  # 三角形
                                            print(f"X = {(mianji_size) / 10 + 10:.1f}mm\n")
                                        elif num_vertices == 4:  # 四边形
                                            print(f"X = {(mianji_size) / 10 + 6:.1f}mm\n")
                                        else:  # 圆形
                                            print(f"X = {(mianji_size) / 10:.1f}mm\n")
                                        sample_count += 1  # 避免重复计算

                    # 绘制选中的最小矩形
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(img_raw, [box], 0, (255, 0, 0), 2)

            except Exception as e:
                print("矩形检测异常:", e)

        # 显示图像
        cv2.imshow("Detection Result", img_raw)

        # 处理按键事件
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # 按q退出
            break
        elif key == ord('p'):  # 按p暂停/继续检测
            pause_rect_detect = not pause_rect_detect
            print("矩形检测已" + ("暂停" if pause_rect_detect else "继续"))
        elif key == ord('r'):  # 按r重置采样
            mianji_samples = []
            sample_count = 0
            print("采样已重置")

    except Exception as e:
        print("主循环异常:", e)
        time.sleep(1)

# 清理资源
cap.release()
cv2.destroyAllWindows()
ser.close()
