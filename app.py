import os
import base64
import paho.mqtt.client as mqtt
import cv2
import numpy as np
from datetime import datetime
from flask import Flask, Response, render_template, jsonify

# ================= MQTT CONFIG =================
MQTT_BROKER = "192.168.1.41"
MQTT_PORT = 1883
MQTT_TOPIC = "esp32"

# ================= FILE CONFIG =================
UPLOAD_FOLDER = "static/uploads"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

# ================= GLOBAL VARIABLES =================
image_parts = []
total_parts = 0
previous_frame = None
latest_frame = None
last_motion_image = None  # Đường dẫn ảnh phát hiện chuyển động

# ================= MQTT CALLBACK FUNCTIONS =================
def on_connect(client, userdata, flags, rc):
    """Gọi khi kết nối MQTT thành công"""
    print(f"✅ Kết nối MQTT thành công (code: {rc})")
    client.subscribe(MQTT_TOPIC)


def on_message(client, userdata, msg):
    """Nhận dữ liệu ảnh từ MQTT và xử lý"""
    global image_parts, total_parts, previous_frame, latest_frame, last_motion_image

    message = msg.payload.decode()

    if message == "end":
        if not image_parts:
            print("❌ Không có dữ liệu ảnh!")
            return

        try:
            print("📥 Nhận ảnh xong. Đang giải mã...")

            # Ghép tất cả các phần lại thành Base64 đầy đủ
            full_image_data = "".join(image_parts)

            # Kiểm tra lỗi padding Base64
            if len(full_image_data) % 4 != 0:
                full_image_data += "=" * (4 - len(full_image_data) % 4)

            # Giải mã Base64
            image_bytes = base64.b64decode(full_image_data)

            # Chuyển thành mảng numpy
            np_arr = np.frombuffer(image_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                print("❌ Lỗi giải mã ảnh!")
                return

            # Cập nhật ảnh mới nhất
            latest_frame = frame

            # Phát hiện chuyển động
            if detect_motion(frame):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                file_name = f"motion_{timestamp}.jpg"
                file_path = os.path.join(UPLOAD_FOLDER, file_name)
                cv2.imwrite(file_path, frame)
                print(f"📸 Phát hiện chuyển động! Lưu ảnh: {file_path}")

                # Cập nhật ảnh phát hiện chuyển động
                last_motion_image = file_name

        except Exception as e:
            print(f"❌ Lỗi xử lý ảnh: {e}")

        # Reset buffer
        image_parts = []
        total_parts = 0

    else:
        try:
            # Nhận dữ liệu ảnh từ MQTT
            part_info, part_data = message.split(":", 1)
            part_index, total_parts = map(int, part_info.split("/"))

            if len(image_parts) == 0:
                image_parts = [""] * total_parts  # Khởi tạo danh sách rỗng

            image_parts[part_index - 1] = part_data

        except Exception as e:
            print(f"❌ Lỗi nhận phần ảnh: {e}")


def detect_motion(frame):
    """Phát hiện chuyển động bằng cách so sánh với frame trước"""
    global previous_frame

    if previous_frame is None:
        previous_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return False

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(previous_frame, gray)
    thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)[1]
    non_zero_count = cv2.countNonZero(thresh)

    previous_frame = gray  # Cập nhật frame trước

    return non_zero_count > 5000  # Ngưỡng phát hiện chuyển động


# ================= FLASK WEB SERVER =================
app = Flask(__name__)

@app.route("/")
def index():
    """Giao diện chính"""
    return render_template("index.html")


def generate_video():
    """Luồng video từ ESP32-CAM"""
    global latest_frame
    while True:
        if latest_frame is not None:
            _, buffer = cv2.imencode(".jpg", latest_frame)
            frame_bytes = buffer.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   frame_bytes + b"\r\n")


@app.route("/video_feed")
def video_feed():
    """API stream video"""
    return Response(generate_video(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/latest_motion")
def latest_motion():
    """API trả về ảnh phát hiện chuyển động mới nhất"""
    global last_motion_image
    if last_motion_image:
        return jsonify({"image": f"/static/uploads/{last_motion_image}", "message": "Chuyển động được phát hiện!"})
    else:
        return jsonify({"image": "", "message": "Không có chuyển động nào được phát hiện."})



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()  # Chạy MQTT trong luồng riêng

# ================= RUN WEB SERVER =================
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
