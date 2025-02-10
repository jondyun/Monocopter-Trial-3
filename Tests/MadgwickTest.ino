#include <MPU6050.h>
#include <MadgwickAHRS.h>

MPU6050 mpu;
Madgwick filter;  // Madgwick filter instance

// Raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Tuneable
const float beta = 200;  // Tune for Madgwick filter 
const float dt = 0.005;  // Loop period (s)

// Bias variables
float gyroBias[3] = {0, 0, 0}; // Array to store gyroscope bias values

// Angle variables
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup() {
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU connection failed!");
    while (1);
  }

  Serial.begin(115200);

  // Calibration function 
  calibrateGyroscope();

  // Initialize Madgwick filter with beta
  filter.begin(beta);
}

void calibrateGyroscope() {
  const int numSamples = 1000; // Number of samples for averaging
  long gxSum = 0, gySum = 0, gzSum = 0;

  Serial.println("Calibrating gyroscope... Do not move the controller.");
  for (int i = 0; i < numSamples; i++) {
    mpu.getRotation(&gx, &gy, &gz);

    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    delay(2); // Very slight delay between samples
  }

  // Calculate average bias
  gyroBias[0] = gxSum / (float)numSamples / 131.0; // Convert to deg/sec
  gyroBias[1] = gySum / (float)numSamples / 131.0;
  gyroBias[2] = gzSum / (float)numSamples / 131.0;

  // // Print for debugging
  // Serial.print("Gyro Bias X: "); Serial.println(gyroBias[0]);
  // Serial.print("Gyro Bias Y: "); Serial.println(gyroBias[1]);
  // Serial.print("Gyro Bias Z: "); Serial.println(gyroBias[2]);
}

void loop() {
  // Get raw data from MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer data to g's
  float axG = ax / 16384.0;
  float ayG = ay / 16384.0;
  float azG = az / 16384.0;

  // Convert gyroscope data to deg/s and apply bias correction
  float gxDPS = (gx / 131.0) - gyroBias[0];
  float gyDPS = (gy / 131.0) - gyroBias[1];
  float gzDPS = (gz / 131.0) - gyroBias[2];

  // Update Madgwick filter
  filter.updateIMU(gxDPS, gyDPS, gzDPS, axG, ayG, azG);

  // Retrieve the calculated orientation angles
  pitch = filter.getPitch();
  roll = filter.getRoll();
  yaw = filter.getYaw();

  // // Print for debugging
  // Serial.print(pitch);
  // Serial.print(", ");
  // Serial.print(roll);
  // Serial.print(", ");
  // Serial.println(yaw);

  // Delay to match the loop duration
  delay(dt * 1000);
}
