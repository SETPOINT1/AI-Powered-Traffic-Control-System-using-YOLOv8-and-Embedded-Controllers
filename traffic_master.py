import cv2
import time
import serial
import math
from ultralytics import YOLO

# ==========================================
# ⚙️ SYSTEM CONFIGURATION (ตั้งค่าระบบ)
# ==========================================

# 1. การเชื่อมต่อ Arduino
SERIAL_PORT = '/dev/ttyACM0'  # เช็คด้วยคำสั่ง ls /dev/tty* ใน Terminal
BAUD_RATE = 115200

# 2. การตั้งค่ากล้อง (Map ทิศทาง กับ Index ของกล้อง)
# หมายเหตุ: ถ้าใช้ Video File ทดสอบ ให้ใส่ชื่อไฟล์แทนตัวเลข เช่น "north": "test_n.mp4"
CAMERAS = {
    "NORTH": 0,  # ทิศเหนือ
    "EAST":  2,  # ทิศตะวันออก
    "SOUTH": 4,  # ทิศใต้
    "WEST":  6   # ทิศตะวันตก
}

# 3. การตั้งค่า AI & Traffic Logic
MODEL_PATH = 'yolov8n.pt'     # ใช้ Nano model เพื่อความเร็วบน Pi
CONF_THRESHOLD = 0.45         # ความมั่นใจขั้นต่ำ (45%)
MIN_GREEN_TIME = 10           # ไฟเขียวขั้นต่ำ (วินาที)
MAX_GREEN_TIME = 60           # ไฟเขียวสูงสุด (วินาที)
TIME_PER_UNIT = 2.0           # 1 หน่วยความหนาแน่น = เพิ่มเวลา 2 วินาที

# น้ำหนักคะแนนยานพาหนะ (Priority Weights)
VEHICLE_WEIGHTS = {
    2: 1.0,   # Car (รถเก๋ง)
    3: 0.5,   # Motorcycle (มอเตอร์ไซค์)
    5: 2.5,   # Bus (รถเมล์)
    7: 3.0    # Truck (รถบรรทุก)
}
# Class ID ที่ต้องการตรวจจับ
TARGET_CLASSES = list(VEHICLE_WEIGHTS.keys())

# ==========================================
# 🚀 INITIALIZATION (เริ่มระบบ)
# ==========================================

print("🔵 Initializing Smart Traffic System...")

# โหลดโมเดล YOLO
print(f"   - Loading AI Model ({MODEL_PATH})...")
model = YOLO(MODEL_PATH)

# เชื่อมต่อ Arduino
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # รอ Arduino รีเซ็ต
    print(f"   - Connected to Arduino at {SERIAL_PORT}")
except Exception as e:
    print(f"🔴 Error connecting to Arduino: {e}")
    print("   (System will run in Simulation Mode)")

# ==========================================
# 🛠️ CORE FUNCTIONS (ฟังก์ชันหลัก)
# ==========================================

def get_traffic_density(cam_source, direction_name):
    """
    เปิดกล้อง -> ถ่ายภาพ 1 เฟรม -> วิเคราะห์ -> คืนค่าคะแนน
    """
    print(f"   📷 Scanning {direction_name}...", end="\r")
    
    cap = cv2.VideoCapture(cam_source)
    
    # ตั้งค่าความละเอียดต่ำลงเพื่อความเร็ว (640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"\n   ⚠️ Warning: Camera {direction_name} not found!")
        return 0.0

    # อ่านภาพ (อ่านทิ้ง 2-3 เฟรมเพื่อให้กล้องปรับแสง Auto Focus)
    for _ in range(5):
        ret, frame = cap.read()
    
    if not ret:
        print(f"\n   ⚠️ Warning: Failed to grab frame from {direction_name}")
        cap.release()
        return 0.0

    # --- AI Analysis ---
    results = model(frame, verbose=False)
    
    density_score = 0.0
    vehicle_count = 0

    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])

            if cls in TARGET_CLASSES and conf >= CONF_THRESHOLD:
                weight = VEHICLE_WEIGHTS.get(cls, 1.0)
                density_score += weight
                vehicle_count += 1

    cap.release() # ปิดกล้องทันทีเพื่อประหยัด USB Bandwidth
    print(f"   ✅ {direction_name}: {vehicle_count} vehicles (Score: {density_score:.1f})")
    return density_score

def send_command_to_arduino(lane_id, duration):
    """
    ส่งคำสั่ง Serial ไปหา Arduino: <LANE, TIME>
    """
    if ser is None:
        return

    command = f"<{lane_id},{duration}>\n"
    ser.write(command.encode('utf-8'))
    print(f"   📡 Sent Command: {command.strip()}")

    # --- รอ Feedback "DONE" จาก Arduino ---
    print("   ⏳ Waiting for traffic cycle to finish...")
    start_wait = time.time()
    
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            if line == "DONE":
                print("   ✅ Cycle Complete. Moving to next phase.")
                break
        
        # Safety Timeout: ถ้ารอเกิน (เวลาไฟเขียว + 10วิ) แล้ว Arduino เงียบ ให้ข้ามเลย
        if time.time() - start_wait > (duration + 10):
            print("   ⚠️ Timeout: Arduino did not respond. Forcing next cycle.")
            break
        
        time.sleep(0.1)

def calculate_time(score):
    """แปลงคะแนนเป็นวินาที"""
    if score == 0:
        return MIN_GREEN_TIME
    
    calc_time = MIN_GREEN_TIME + (score * TIME_PER_UNIT)
    
    # ปัดเศษและจำกัดค่าไม่ให้เกิน Max
    final_time = int(min(calc_time, MAX_GREEN_TIME))
    return final_time

# ==========================================
# 🔄 MAIN LOOP (ลูปทำงานหลัก)
# ==========================================
def main():
    print("\n🟢 System Started. Running Traffic Control Loop...\n")
    
    # Mapping ชื่อทิศ กับ ID ที่ Arduino เข้าใจ (1,2,3,4)
    LANE_MAPPING = {
        "NORTH": 1,
        "EAST":  2,
        "SOUTH": 3,
        "WEST":  4
    }

    try:
        while True:
            print("-" * 40)
            print(f"🕒 Cycle Start: {time.strftime('%H:%M:%S')}")
            
            # 1. เก็บข้อมูลทุกแยก (Data Acquisition Phase)
            scores = {}
            for direction, cam_idx in CAMERAS.items():
                scores[direction] = get_traffic_density(cam_idx, direction)
            
            # 2. ตัดสินใจ (Decision Phase)
            # หาแยกที่มีคะแนนสูงสุด
            best_lane = max(scores, key=scores.get)
            max_score = scores[best_lane]
            
            # คำนวณเวลา
            green_time = calculate_time(max_score)
            arduino_lane_id = LANE_MAPPING[best_lane]

            print(f"\n🧠 AI Decision: GREEN LIGHT for {best_lane}")
            print(f"   - Score: {max_score:.1f}")
            print(f"   - Duration: {green_time} seconds")

            # 3. สั่งการ (Action Phase)
            send_command_to_arduino(arduino_lane_id, green_time)
            
            # (Optional) หน่วงเวลาเล็กน้อยก่อนเริ่มรอบใหม่
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n🔴 System Stopping...")
        if ser: ser.close()
        print("Bye!")

if __name__ == "__main__":
    main()