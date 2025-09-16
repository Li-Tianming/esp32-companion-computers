#include <MAVLink.h>

// ESP32 串口通信示例
// 使用Serial(USB)和Serial1(硬件串口)进行数据收发

// 定义引脚 - ESP32的默认Serial1引脚是GPIO9(RX)和GPIO10(TX)
#define SERIAL1_RX_PIN 19
#define SERIAL1_TX_PIN 18

// 设置函数
void setup() {
  // 初始化USB串口，用于与计算机通信
  Serial.begin(115200);
  
  // 初始化硬件串口1，用于与其他设备通信
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  
  // 等待串口初始化完成
  delay(1000);
  
  // 打印欢迎信息
  // Serial.println("Press Help to show how to do");
  Serial.println("------------------------------");
}

// 主循环
void loop() {

  processCommand();

  receiveMAVLink();
  // sendMAVLink();
  
  // 短暂延迟以减少CPU占用
  delay(10);
}

void compareStrings(String input) {
  Serial.print("输入: '");
  Serial.print(input);
  Serial.println("'");
  
 
  // 不区分大小写匹配
  if (input.equalsIgnoreCase("land")) {
    Serial.println("LAND");
      sendMAVLink();
  }else if(input.equalsIgnoreCase("set")) {
    Serial.println("SET PARAM");
    sendMAVLinkSetAngleMax();
  }
    
}

void processCommand() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // 读取直到换行符
    input.trim();
    
    compareStrings(input);
  }
}

void sendMAVLinkSetAngleMax() {

  // Generate HEARTBEAT message buffer
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // 从原PARAM_VALUE消息中提取的参数信息
  const char *param_id = "ANGLE_MAX";      // 参数名
  float param_value = 1500.0f;             // 参数值（15.0度，单位为厘度）
  uint8_t param_type = MAV_PARAM_TYPE_REAL32; // 参数类型（浮点数）

  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,  
  //uint8_t target_system, uint8_t target_component, const char *param_id, float param_value, uint8_t param_type)
  mavlink_msg_param_set_pack(0xFF, 
                            MAV_COMP_ID_MISSIONPLANNER, 
                            &msg, 
                            1, 
                            1, 
                            param_id,
                            param_value,
                            param_type);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len); // 发送原始字节
  Serial.println("Send Mavlink SET_PARAM ANGLE_MAX");

}


void sendMAVLink() {
  static uint32_t lastSent = 0;
  if (millis() - lastSent < 100) return; // Send every second

  // Generate HEARTBEAT message buffer
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t base_mode, uint32_t custom_mode
  mavlink_msg_set_mode_pack(0xFF, MAV_COMP_ID_MISSIONPLANNER, &msg, MAV_TYPE_FIXED_WING, 1, 9);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send buffer over UDP
	// udp.beginPacket("255.255.255.255", 14550);
	// udp.write(buf, len);
	// udp.endPacket();
  Serial1.write(buf, len); // 发送原始字节
  Serial.println("Send Mavlink SET_MODE");

  lastSent = millis();
}

void receiveMAVLink() {

    // Read UDP packet
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int bytesRead = 0;

  if (Serial1.available()) {

    bytesRead = Serial1.readBytes(buf, MAVLINK_MAX_PACKET_LEN);
  
    Serial.printf("Received %d: ", bytesRead);

    for (size_t i = 0; i < bytesRead; i++) {
      Serial.printf("%02X ", buf[i]);
    }
    Serial.println();

  }

  // Parse MAVLink message
  mavlink_message_t msg;
  mavlink_status_t status;
  for (int i = 0; i < bytesRead; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          Serial.println("Received HEARTBEAT");
          break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
          // Can be used to control the vehicle
          mavlink_manual_control_t manualControl;
          mavlink_msg_manual_control_decode(&msg, &manualControl);
          Serial.print("Received MANUAL_CONTROL:");
          Serial.print(" x=");
          Serial.print(manualControl.x);
          Serial.print(" y=");
          Serial.print(manualControl.y);
          Serial.print(" z=");
          Serial.print(manualControl.z);
          Serial.print(" r=");
          Serial.println(manualControl.r);
          break;
        default:
          Serial.print("Received message with ID ");
          Serial.println(msg.msgid);
          break;
      }
    }
  }
}