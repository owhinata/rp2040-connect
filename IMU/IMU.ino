#include <Arduino_LSM6DSOX.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {}
  }
  Serial.println("LSM6DSOX ready.");
  Serial.println("ax[g], ay[g], az[g], gx[dps], gy[dps], gz[dps], temp[C]");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  int t;

  bool got = false;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    if (IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {
      got = true;
    }
  }
  if (IMU.temperatureAvailable()) {
    IMU.readTemperature(t);
  }

  if (got) {
    Serial.print(ax, 6); Serial.print(", ");
    Serial.print(ay, 6); Serial.print(", ");
    Serial.print(az, 6); Serial.print(", ");
    Serial.print(gx, 6); Serial.print(", ");
    Serial.print(gy, 6); Serial.print(", ");
    Serial.print(gz, 6); Serial.print(", ");
    Serial.println(t, 2);
  }

  delay(10);
}
