#include <M5Unified.h>
#include <mcp_can.h>
#include <C610.hpp>

MCP_CAN CAN0(27);    // Set CS to pin 10

C610Array c610;

void can_init();
void can_read();
void can_write();

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.Power.begin();
  M5.Lcd.begin();
  can_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  auto now = millis();

  auto imu_update = M5.Imu.update();
  if(imu_update) {
    float ax, ay, az;
    float gx, gy, gz;

    M5.Imu.getAccel(&ax, &ay, &az);
    M5.Imu.getGyro(&gx, &gy, &gz);

    static auto last_display = now;
    if(now - last_display > 100) {
      M5.Display.startWrite();
      M5.Display.setCursor(0, 0);

      M5.Display.printf("Accel X = %12.6f\n", ax);
      M5.Display.printf("Accel Y = %12.6f\n", ay);
      M5.Display.printf("Accel Z = %12.6f\n", az);
      M5.Display.printf("Gyro  X = %12.6f\n", gx);
      M5.Display.printf("Gyro  Y = %12.6f\n", gy);
      M5.Display.printf("Gyro  Z = %12.6f\n", gz);

      M5.Display.endWrite();
      last_display = now;
    }
  }

  c610[0].set_current(1.0);
  can_read();

  static auto last_can_send = now;
  if(now - last_can_send > 10) {
    can_write();
    last_can_send = now;
  }
}

void can_init() {
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the
  // masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK){
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
  }

  CAN0.setMode(MCP_NORMAL);  // Change to normal mode to allow messages to be
                            // transmitted
}

byte can_read_msg(CANMessage &msg) {
  return CAN0.readMsgBuf(&msg.id, &msg.len, msg.data);
}

void can_read() {
  if(CANMessage msg; can_read_msg(msg) == CAN_OK) {
    c610.parse_packet(msg);
  }
}

byte can_write_msg(CANMessage &msg) {
  auto rtr = msg.type == CANData ? 0 : 1;
  auto ext = msg.format == CANStandard ? 0 : 1;
  auto id = msg.id | (ext << 31) | (rtr << 30);
  return CAN0.sendMsgBuf(id, msg.len, msg.data);
}

void can_write() {
  auto msgs = c610.to_msgs();
  for(auto& msg : msgs) {
    byte sndStat = can_write_msg(msg);
    if (sndStat == CAN_OK) {
      Serial.printf("Message Sent Successfully! ID: %d\n", msg.id);
    } else {
      Serial.println("Error Sending Message...");
    }
  }
}
