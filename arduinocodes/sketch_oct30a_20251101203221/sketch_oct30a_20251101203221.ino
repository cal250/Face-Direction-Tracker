#include <Stepper.h>

const int STEPS_PER_REV = 2048;
Stepper panMotor(STEPS_PER_REV / 4, 8, 10, 9, 11);
Stepper tiltMotor(STEPS_PER_REV / 4, 3, 5, 4, 6);

int targetPan = 0;
int targetTilt = 0;
int currentPan = 0;
int currentTilt = 0;

void setup() {
  Serial.begin(115200);  // MAX SPEED!
  panMotor.setSpeed(30);
  tiltMotor.setSpeed(30);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    int p = cmd.substring(1, cmd.indexOf(',')).toInt();
    int t = cmd.substring(cmd.indexOf('T') + 1).toInt();

    if (cmd.startsWith("P")) {
      targetPan += p;
      targetTilt += t;
    }
  }

  // INSTANT MICRO-STEPPING
  if (currentPan != targetPan) {
    int step = targetPan > currentPan ? 1 : -1;
    panMotor.step(step);
    currentPan += step;
  }
  if (currentTilt != targetTilt) {
    int step = targetTilt > currentTilt ? 1 : -1;
    tiltMotor.step(step);
    currentTilt += step;
  }
}