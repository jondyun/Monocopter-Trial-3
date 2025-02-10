#include <MPU6050.h>

MPU6050 mpu;

// Constants for complementary filter
const float alpha = 0.98;  // Filter coefficient for complementary filter
float dt = 0.005;           // Loop period in seconds

// Angle variables
float pitch = 0;
float roll = 0;
float yaw = 0;

// Use gyro directly for yaw
float previousGyroZ = 0;

void setup() {
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU connection failed!");
    while (1);
  }

  Serial.begin(115200);

  // Get initial raw data to set initial pitch, roll, and yaw
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert accelerometer data to g's using MPU6050 sensitivity value
  float axG = ax / 16384.0;
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;

  // Calculate initial pitch and roll from accelerometer, in degrees
  pitch = atan2(ayG, azG) * 180 / M_PI;
  roll = atan2(-axG, sqrt(ayG * ayG + azG * azG)) * 180 / M_PI;

  // No accelerometer yaw data
  yaw = 0;
}

void loop() {
  // Get raw data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer data to g's using MPU6050 sensitivity value
  float axG = ax / 16384.0;
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;

  // Convert gyroscope data to degrees per second using LSBs
  float gxDPS = gx / 131.0; // Gyro gives a speed in degrees/second
  float gyDPS = gy / 131.0;
  float gzDPS = gz / 131.0;

  // Calculate pitch and roll from accelerometer
  float pitchAcc = atan2(ayG, azG) * 180 / M_PI;
  float rollAcc = atan2(-axG, sqrt(ayG * ayG + azG * azG)) * 180 / M_PI;

  // Complementary filter to combine accelerometer and gyroscope data for pitch and roll
  pitch = alpha * (pitch + gxDPS * dt) + (1 - alpha) * pitchAcc;
  roll = alpha * (roll + gyDPS * dt) + (1 - alpha) * rollAcc;

  // Integrating z-axis gyro data over time
  yaw += gzDPS * dt;
  
  // Output test results
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.println(yaw);

  delay(dt * 1000);  // Delay for dt seconds
}
