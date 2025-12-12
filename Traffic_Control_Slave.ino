// --- CONFIGURATION ---
// ตั้งค่า Relay: ถ้าใช้ Relay Module ทั่วไปจะเป็น Active LOW (LOW=ติด, HIGH=ดับ)
#define RELAY_ON  LOW
#define RELAY_OFF HIGH

const int YELLOW_TIME = 3000; // เวลาไฟเหลือง (3 วินาที)

// กำหนดขา Pin (Red, Yellow, Green)
// Lane 1: เหนือ, Lane 2: ออก, Lane 3: ใต้, Lane 4: ตก
const int L1_R = 22, L1_Y = 23, L1_G = 24;
const int L2_R = 25, L2_Y = 26, L2_G = 27;
const int L3_R = 28, L3_Y = 29, L3_G = 30;
const int L4_R = 31, L4_Y = 32, L4_G = 33;

// ตัวแปรสำหรับรับข้อมูล Serial
String inputString = "";
boolean stringComplete = false;

void setup() {
  // 1. เริ่มต้น Serial
  Serial.begin(115200);
  inputString.reserve(200);

  // 2. กำหนดโหมดขา Pin ทั้งหมดเป็น OUTPUT
  int pins[] = {L1_R, L1_Y, L1_G, L2_R, L2_Y, L2_G, L3_R, L3_Y, L3_G, L4_R, L4_Y, L4_G};
  for (int i = 0; i < 12; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], RELAY_OFF); // ปิดไฟทุกดวงก่อน
  }

  // 3. เริ่มต้นด้วยไฟแดงทุกด้าน (Safety Mode)
  setAllRed();
  
  Serial.println("SYSTEM READY: Waiting for commands...");
}

void loop() {
  // ตรวจสอบว่ามีคำสั่งมาครบหรือยัง
  if (stringComplete) {
    parseCommand(inputString);
    
    // เคลียร์ค่าเพื่อรอคำสั่งใหม่
    inputString = "";
    stringComplete = false;
  }
}

// --- ฟังก์ชันอ่าน Serial แบบ Interrupt (ทำงานอัตโนมัติเมื่อมีข้อมูลมา) ---
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    // ถ้าเจอตัวปิดท้าย '>' แสดงว่าจบคำสั่ง
    if (inChar == '>') {
      stringComplete = true;
    }
  }
}

// --- ฟังก์ชันแกะคำสั่ง (Parser) ---
void parseCommand(String data) {
  // รูปแบบคำสั่ง: <LANE, TIME>  เช่น <1, 30>
  
  int startIndex = data.indexOf('<');
  int endIndex = data.indexOf('>');
  int commaIndex = data.indexOf(',');

  if (startIndex != -1 && endIndex != -1 && commaIndex != -1) {
    // ตัดคำเพื่อเอาตัวเลขออกมา
    String laneStr = data.substring(startIndex + 1, commaIndex);
    String timeStr = data.substring(commaIndex + 1, endIndex);

    int laneID = laneStr.toInt();
    int duration = timeStr.toInt();

    Serial.print("EXECUTING: Lane ");
    Serial.print(laneID);
    Serial.print(" for ");
    Serial.print(duration);
    Serial.println(" sec.");

    // สั่งทำงาน
    runTrafficSequence(laneID, duration);
    
    // ส่งงานเสร็จ บอก Pi
    Serial.println("DONE"); 
  }
}

// --- ฟังก์ชันควบคุมไฟจราจร (Core Logic) ---
void runTrafficSequence(int lane, int durationSec) {
  // 1. เพื่อความปลอดภัย: สั่งแดงทุกแยกก่อนเสมอ
  setAllRed();
  delay(1000); // รอ 1 วิกันรถฝ่าไฟแดง

  // กำหนดขาที่จะใช้ตาม Lane ID
  int r_pin, y_pin, g_pin;
  
  switch(lane) {
    case 1: r_pin = L1_R; y_pin = L1_Y; g_pin = L1_G; break;
    case 2: r_pin = L2_R; y_pin = L2_Y; g_pin = L2_G; break;
    case 3: r_pin = L3_R; y_pin = L3_Y; g_pin = L3_G; break;
    case 4: r_pin = L4_R; y_pin = L4_Y; g_pin = L4_G; break;
    default: return; // ถ้าส่งเลขมั่วมา ไม่ทำอะไร
  }

  // 2. ไฟเขียว (Green)
  digitalWrite(r_pin, RELAY_OFF); // ดับแดง
  digitalWrite(g_pin, RELAY_ON);  // ติดเขียว
  
  // นับถอยหลัง (ใช้ Loop เพื่อให้สามารถ Break ได้ถ้าจำเป็นในอนาคต)
  for(int i=0; i<durationSec; i++) {
    delay(1000); 
  }

  // 3. ไฟเหลือง (Yellow)
  digitalWrite(g_pin, RELAY_OFF); // ดับเขียว
  digitalWrite(y_pin, RELAY_ON);  // ติดเหลือง
  delay(YELLOW_TIME);

  // 4. ไฟแดง (Red)
  digitalWrite(y_pin, RELAY_OFF); // ดับเหลือง
  digitalWrite(r_pin, RELAY_ON);  // ติดแดง
}

// --- ฟังก์ชัน Safety: เปิดไฟแดงทุกด้าน ---
void setAllRed() {
  digitalWrite(L1_G, RELAY_OFF); digitalWrite(L1_Y, RELAY_OFF); digitalWrite(L1_R, RELAY_ON);
  digitalWrite(L2_G, RELAY_OFF); digitalWrite(L2_Y, RELAY_OFF); digitalWrite(L2_R, RELAY_ON);
  digitalWrite(L3_G, RELAY_OFF); digitalWrite(L3_Y, RELAY_OFF); digitalWrite(L3_R, RELAY_ON);
  digitalWrite(L4_G, RELAY_OFF); digitalWrite(L4_Y, RELAY_OFF); digitalWrite(L4_R, RELAY_ON);
}