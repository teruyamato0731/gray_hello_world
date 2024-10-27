#include <M5Unified.h>
#include <mcp_can.h>
#include <C610.hpp>
#include <packet.hpp>

MCP_CAN CAN0(27);    // Set CS to pin 10

C610Array c610;

void can_init();
void can_read();
void can_write();
bool serial_read(Control& c);
void serial_write(const Sensor& s);
void display_update();

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Serial.begin(115200);
  M5.Power.begin();
  M5.Lcd.begin();
  M5.Lcd.setTextFont(4);
  can_init();
  Serial.println("M5Stack Initialized");
}

void loop() {
  // put your main code here, to run repeatedly:
  auto now = millis();

  M5.Imu.update();
  display_update();

  Sensor s{1.2, 3.4};
  serial_write(s);

  if (Control c; serial_read(c)) {
    c610[0].set_raw_current(c.current);
  }

  c610[0].set_current(1.0);
  can_read();

  can_write();
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
  auto now = millis();
  static auto last_can_send = now;
  if(now - last_can_send > 10) {
    auto msgs = c610.to_msgs();
    for(auto& msg : msgs) {
      byte sndStat = can_write_msg(msg);
      if (sndStat == CAN_OK) {
        // Serial.printf("Message Sent Successfully! ID: %d\n", msg.id);
      } else {
        Serial.println("Error Sending Message...");
      }
    }
    last_can_send = now;
  }
}

constexpr size_t buf_size = 8;
uint8_t buf[buf_size] = {0xde, 0xad, 0xbe, 0xef};
bool serial_read(Control& c) {
  if(Serial.available() > 4) {
    size_t len = Serial.readBytesUntil(0x00, buf, buf_size);
    if (len == SIZE_OF_CONTROL + 1) {
      c.decode(buf);
      return true;
    }
  }
  return false;
}

void serial_write(const Sensor& s) {
  auto now = millis();
  static auto last_serial_send = now;
  if(now - last_serial_send > 3) {
    uint8_t buf[SIZE_OF_SENSOR + 2];
    s.encode(buf);
    Serial.write(buf, sizeof(buf));
    last_serial_send = now;
  }
}

void display_update() {
  auto now = millis();
  static auto last_display = now;
  if(now - last_display > 100) {
    float ax, ay, az;
    float gx, gy, gz;

    M5.Imu.getAccel(&ax, &ay, &az);
    M5.Imu.getGyro(&gx, &gy, &gz);

    M5.Display.startWrite();
    M5.Display.setCursor(0, 0);

    M5.Display.printf("Accel X = % 12.3f  \n", ax);
    M5.Display.printf("Accel Y = % 12.3f  \n", ay);
    M5.Display.printf("Accel Z = % 12.3f  \n", az);
    M5.Display.printf("Gyro  X = % 12.3f  \n", gx);
    M5.Display.printf("Gyro  Y = % 12.3f  \n", gy);
    M5.Display.printf("Gyro  Z = % 12.3f  \n", gz);
    M5.Display.printf("\n");
    M5.Display.printf("Control = % 4d\n", c610[0].get_raw_current());
    M5.Display.printf("buf: [%2x, %2x, %2x, %2x]     \n", buf[0], buf[1], buf[2], buf[3]);

    M5.Display.endWrite();
    last_display = now;
  }
}
