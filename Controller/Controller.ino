#include <MPU6050.h>
#include <Servo.h>
#include <MadgwickAHRS.h>

MPU6050 mpu;
Madgwick filter; // Madgwick instance

Servo servo1, servo2, servo3, servo4;

// MPU raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Complementary filter for roll and pitch
float alpha = 0.98; // Tune for complementary filter
float rollComp = 0, pitchComp = 0;
float prevRoll = 0, prevPitch = 0;

// Madwick filter for yaw
const float beta = 600;  // Tune for Madgwick filter 
float yawMadg = 0;

// Sampling, 200 Hz
const float dt = 0.005;  // Loop period (s)

// PID variables
float Kp = 1.2, Ki = 0.007, Kd = 0.008;

// Other variables  
float servo1Angle = 90, servo2Angle = 90, servo3Angle = 90, servo4Angle = 90;
float gyroBias[3] = {0, 0, 0};
float targetPitch = 0, targetRoll = 0, targetYaw = 180;
float errorPitch = 0, prevErrorPitch = 0, integralPitch = 0, derivativePitch = 0;
float errorRoll = 0, prevErrorRoll = 0, integralRoll = 0, derivativeRoll = 0;
float errorYaw = 0, prevErrorYaw = 0, integralYaw = 0, derivativeYaw = 0;

float rollOffset = 0, pitchOffset = 0, yawOffset = 0;

void setup() {
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU connection failed!");
    while (1);
  }

  Serial.begin(115200);

  // Calibrate Gyroscope
  calibrateGyroscope();

  // Initialize Madgwick filter
  filter.begin(beta);

  // Capture initial orientation for offsets
  for (int i = 0; i < 100; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert accelerometer data to g's
    float axG = ax / 16384.0, ayG = ay / 16384.0, azG = az / 16384.0;

    // Convert gyroscope data to deg/s
    float gxDPS = (gx / 131.0) - gyroBias[0];
    float gyDPS = (gy / 131.0) - gyroBias[1];
    float gzDPS = (gz / 131.0) - gyroBias[2];

    // Update filter
    filter.updateIMU(gxDPS, gyDPS, gzDPS, axG, ayG, azG);

    // Accumulate offsets
    rollOffset += atan2(ayG, azG) * 180.0 / PI;
    pitchOffset += atan2(-axG, sqrt(ayG * ayG + azG * azG)) * 180.0 / PI;
    yawOffset += filter.getYaw();

    delay(5); // Small delay for sampling
  }

  // Average offsets
  rollOffset /= 100.0;
  pitchOffset /= 100.0;
  yawOffset /= 100.0;

  Serial.println("Offsets calculated:");
  Serial.print("Roll Offset: "); Serial.println(rollOffset);
  Serial.print("Pitch Offset: "); Serial.println(pitchOffset);
  Serial.print("Yaw Offset: "); Serial.println(yawOffset);

  // Initialize servos
  servo1.attach(6);
  servo2.attach(7);
  servo3.attach(8);
  servo4.attach(9);

  // Set to neutral
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);

  delay(1000);
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
  gyroBias[0] = gxSum / (float)numSamples / 131.0; // Convert to deg/s
  gyroBias[1] = gySum / (float)numSamples / 131.0;
  gyroBias[2] = gzSum / (float)numSamples / 131.0;
  Serial.println("All set!");
}

float normalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer data to g's
  float axG = ax / 16384.0, ayG = ay / 16384.0, azG = az / 16384.0;

  // Convert gyroscope data to deg/s and apply bias correction
  float gxDPS = (gx / 131.0) - gyroBias[0], gyDPS = (gy / 131.0) - gyroBias[1], gzDPS = (gz / 131.0) - gyroBias[2];

  // Calculate roll and pitch from accelerometer data
  float rollAccel = atan2(ayG, azG) * 180.0 / PI - rollOffset;
  float pitchAccel = atan2(-axG, sqrt(ayG * ayG + azG * azG)) * 180.0 / PI - pitchOffset;

  // Update Madgwick filter
  filter.updateIMU(gxDPS, gyDPS, gzDPS, axG, ayG, azG);
  yawMadg = filter.getYaw() - yawOffset;
  yawMadg = normalizeAngle(yawMadg);

  // Apply complementary filter for roll and pitch
  rollComp = alpha * (prevRoll + gxDPS * dt) + (1 - alpha) * rollAccel;
  pitchComp = alpha * (prevPitch + gyDPS * dt) + (1 - alpha) * pitchAccel;

  // Update previous values for the next loop
  prevRoll = rollComp;
  prevPitch = pitchComp;

  // Retrieve the calculated orientation angles from Madgwick
  yawMadg = filter.getYaw();

  // Use the filtered roll and pitch for PID calculations
  errorPitch = targetPitch - pitchComp;
  integralPitch += errorPitch * dt;
  derivativePitch = (errorPitch - prevErrorPitch) / dt;
  prevErrorPitch = errorPitch;
  float pitchOutput = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;

  errorRoll = targetRoll - rollComp;
  integralRoll += errorRoll * dt;
  derivativeRoll = (errorRoll - prevErrorRoll) / dt;
  prevErrorRoll = errorRoll;
  float rollOutput = Kp * errorRoll + Ki * integralRoll + Kd * derivativeRoll;

  errorYaw = targetYaw - yawMadg;
  integralYaw += errorYaw * dt;
  derivativeYaw = (errorYaw - prevErrorYaw) / dt;
  prevErrorYaw = errorYaw;
  float yawOutput = Kp * errorYaw + Ki * integralYaw + Kd * derivativeYaw;

  // Anti-windup
  integralYaw = constrain(integralYaw, -50, 50);

  // Map PID outputs to servo angles
  servo1Angle = constrain(90 + pitchOutput + yawOutput, 30, 150);
  servo2Angle = constrain(90 + rollOutput - yawOutput, 30, 150);
  servo3Angle = constrain(90 - pitchOutput + yawOutput, 30, 150);
  servo4Angle = constrain(90 - rollOutput - yawOutput, 30, 150);

  // Update servos at a fixed rate
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 20) { // 50 Hz update rate
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
    servo4.write(servo4Angle);

    // Debug
    // Serial.print(servo1Angle);
    // Serial.print(", "); Serial.print(servo2Angle);
    // Serial.print(", "); Serial.print(servo3Angle);
    // Serial.print(", "); Serial.println(servo4Angle);

    Serial.print(pitchOutput); Serial.print(", ");
    Serial.print(rollOutput); Serial.print(", ");
    Serial.println(yawOutput);

    // Serial.print("Error Yaw: "); Serial.println(errorYaw);
    // Serial.print("Integral Yaw: "); Serial.println(integralYaw);
    // Serial.print("Derivative Yaw: "); Serial.println(derivativeYaw);
    // Serial.print("Yaw Output: "); Serial.println(yawOutput);
  }

  // Serial input for PID tuning
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'P') Kp += 0.1; // Increase Kp
    else if (command == 'p') Kp -= 0.1; // Decrease Kp
    else if (command == 'I') Ki += 0.001; // Increase Ki
    else if (command == 'i') Ki -= 0.001; // Decrease Ki
    else if (command == 'D') Kd += 0.01; // Increase Kd
    else if (command == 'd') Kd -= 0.01; // Decrease Kd
    else if (command == '?') printPIDValues(); // Print PID values

    // Clamp
    Kp = constrain(Kp, 0, 10);
    Ki = constrain(Ki, 0, 1);
    Kd = constrain(Kd, 0, 1);
  }
}

// Function to print PID values
void printPIDValues() {
  Serial.print("Kp: "); Serial.print(Kp, 3);
  Serial.print(", Ki: "); Serial.print(Ki, 5);
  Serial.print(", Kd: "); Serial.println(Kd, 5);
}
