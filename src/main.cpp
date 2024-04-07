#include <M5Unified.h>

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Serial.println("=== M5Stack initialized ===");
}

void loop() {
  // put your main code here, to run repeatedly:
  auto imu_update = M5.Imu.update();
  if(imu_update) {
    float ax, ay, az;
    float gx, gy, gz;

    M5.Imu.getAccel(&ax, &ay, &az);
    M5.Imu.getGyro(&gx, &gy, &gz);

    M5.Display.startWrite();
    M5.Display.setCursor(0, 0);

    M5.Display.printf("Accel X = %12.6f\n", ax);
    M5.Display.printf("Accel Y = %12.6f\n", ay);
    M5.Display.printf("Accel Z = %12.6f\n", az);
    M5.Display.printf("Gyro  X = %12.6f\n", gx);
    M5.Display.printf("Gyro  Y = %12.6f\n", gy);
    M5.Display.printf("Gyro  Z = %12.6f\n", gz);

    M5.Display.endWrite();

    delay(100);
  }
}
