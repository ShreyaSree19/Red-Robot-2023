// Replace 12345 with the correct team number and then uncomment the line below.
#define TEAM_NUMBER 21

#ifndef TEAM_NUMBER
#error "Define your team number with `#define TEAM_NUMBER 12345` at the top of the file."
#elif TEAM_NUMBER < 1 || 40 < TEAM_NUMBER
#error "Team number must be within 1 and 40"
#endif

void setup()
{
  Serial.begin(115200);
}

int temp = 0;
bool autonomous = false;

void loop()
{
  // Read the four joystick axes
  // These will be in the range [-1.0, 1.0]
  float leftX = RR_axisRY();
  float leftY = RR_axisRX();
  float rightX = RR_axisLY();
  float rightY = RR_axisLX();

  // Arcade-drive scheme
  // Left Y-axis = throttle
  // Right X-axis = steering
  RR_setMotor1(leftY + rightX);
  RR_setMotor2(leftY - rightX);

  // Get the button states
  bool btnA = RR_buttonA();
  bool btnB = RR_buttonB();
  bool btnX = RR_buttonX();
  bool btnY = RR_buttonY();
  bool btnRB = RR_buttonRB();
  bool btnLB = RR_buttonLB();

  // Control motor3 port (unused on base robot) using A/B buttons
  if (btnA)
  {
    autonomous = true;
    RR_setMotor3(1.0);
  }
  else if (btnB)
  {
    autonomous = false;
    RR_setMotor3(-1.0);
  }
  else
  {
    RR_setMotor3(0.0);
  }

  // Control motor4 port (unused on base robot) using X/Y buttons
  if (btnX)
  {
    RR_setMotor4(1.0);
  }
  else if (btnY)
  {
    RR_setMotor4(-1.0);
  }
  else
  {
    RR_setMotor4(0.0);
  }

  // Control servo 1 using the dpad
  // 6 = left, 2 = right, 0 = up, 4 = down, 8 = center
  if (RR_dpad() == 6)
  { // left

    // we can't move a servo less than 0 degrees
    if (temp > 0)
      temp -= 10;
  }
  else if (RR_dpad() == 2)
  { // right

    // we can't move a servo past 180 degrees
    // for continuous rotation, try using a DC motor
    if (temp < 180)
      temp += 10;
    //FAIL SAFE WHILE TESTING
    RR_setMotor1(0);
    RR_setMotor2(0);
  }
  RR_setServo1(temp);

  // Control servo 2 using the shoulder buttons
  // This example moves the servo to fixed points
  // You can change the angles based on your mechanism
  // (this is great for a mechanism that only has 2 states,
  //  such as a grabber or hook)
  if (btnRB)
  {
    RR_setServo2(180);
  }
  else if (btnLB)
  {
    RR_setServo2(0);
  }

  // we also have RR_setServo3 and RR_setServo4 available

  // read the ultrasonic sensors

  Serial.print("Ultrasonic=");
  Serial.print(RR_getUltrasonic());
  Serial.print(" ;; ");
  int sensors[6];

  Serial.print("Line sensors=");
  RR_getLineSensors(sensors);
  long numerator = 0;
  long denominator = 0;
  for (int i = 0; i < 6; ++i)
  {
    Serial.print(sensors[i]);
    Serial.print(" ");
    numerator += (sensors[i] * i*1000);
    denominator += (sensors[i]);
  }
  Serial.print("Average=");
  long avg = numerator / (denominator);
  Serial.print(avg);
  // aim for 26000 - 29000
  //left: 29000-36000
  //right: 19000-21000
  if (autonomous) {
      // --- Line following control ---
    const long target = 27500000; // center value
    float error = (avg - target)/1000000; // normalize
    Serial.print("Error = ");
    Serial.println(error);
    float Kp = 0.05; // tune this gain experimentally

    // Base speed of motors
    float baseSpeed = 0.8;

    // Compute turn correction
    float turn = Kp * error;
    Serial.print("Turn = ");
    Serial.println(turn);
    float leftMotor = baseSpeed - turn;
    Serial.print("Left speed: ");
    Serial.println(leftMotor);
    float rightMotor = baseSpeed + turn;
    Serial.print("Right speed: ");
    Serial.println(rightMotor);
    // Clamp motor values to [-1, 1]
    leftMotor = constrain(leftMotor, -1, 1);
    rightMotor = constrain(rightMotor, -1, 1);

    RR_setMotor1(leftMotor);
    RR_setMotor2(rightMotor);
  }
  Serial.print(btnA ? 1 : 0);
  Serial.print(btnB ? 1 : 0);
  Serial.print(btnX ? 1 : 0);
  Serial.print(btnY ? 1 : 0);
  Serial.println();

  // This is important - it sleeps for 0.02 seconds (= 50 times / second)
  // Running the code too fast will overwhelm the microcontroller and peripherals
  delay(20);
}

// vim: tabstop=2 shiftwidth=2 expandtab
