//////////////////////////// Pins /////////////////////////////
// Motor control with L298N motor driver
int motor_a_value_pin = 5;        // ENA -> (digital PWM)
int motor_a_direction_1_pin = 7;  // IN1 -> (digital)
int motor_a_direction_2_pin = 8;  // IN2 -> (digital)
int motor_b_value_pin = 3;        // ENB -> (figital PWM)
int motor_b_direction_1_pin = 4;  // IN3 -> (digital)
int motor_b_direction_2_pin = 2;  // IN4 -> (digital)
// RC channels
// 1: right left/right, 2: left up/down, 3: right up/down, 4: left left/right,
// 5: ?, 6: ?
int rc_channel_pins[] = {-1, 11, -1, 10, 9, 6};  // channel x -> (digital PWM)
///////////////////////////////////////////////////////////////

///////////////////////// Constants ///////////////////////////
// Motor aliases
int motor_a = 0;
int motor_b = 1;
// RC control
int rc_padding = 50;
int rc_min_value = 985 + rc_padding;
int rc_max_value = 1985 - rc_padding;
int rc_off_threshold = 100;
int rc_no_power_threshold = 5000;
///////////////////////////////////////////////////////////////

float read_rc_value(int channel) {
  int pin = rc_channel_pins[channel - 1];

  int rc_value = pulseIn(pin, HIGH);

  if (rc_value < rc_off_threshold || rc_value > rc_no_power_threshold) {
    return 0.0;
  }

  rc_value = constrain(rc_value, rc_min_value, rc_max_value);

  float value = map(rc_value, rc_min_value, rc_max_value, -512, 512) / 512.0;

  return value;
}

// motor_a or motor_b, direction -1 or 1
void set_direction(int motor, int direction) {
  int direction_1_pin;
  int direction_2_pin;
  if (motor == motor_a) {
    direction_1_pin = motor_a_direction_1_pin;
    direction_2_pin = motor_a_direction_2_pin;
  } else {
    direction_1_pin = motor_b_direction_1_pin;
    direction_2_pin = motor_b_direction_2_pin;
  }

  if (direction == 1) {
    digitalWrite(direction_1_pin, HIGH);
    digitalWrite(direction_2_pin, LOW);
  } else if (direction == -1) {
    digitalWrite(direction_1_pin, LOW);
    digitalWrite(direction_2_pin, HIGH);
  } else {
    digitalWrite(direction_1_pin, LOW);
    digitalWrite(direction_2_pin, LOW);
  }
}

// motor_a or motor_b, speed from -1 to 1
void set_speed(int motor, float speed) {
  speed = min(1, max(-1, speed));

  int direction = 0;
  if (abs(speed) < 0.01) {
    set_direction(motor, 0);
  } else if (speed > 0) {
    set_direction(motor, 1);
  } else {
    set_direction(motor, -1);
  }

  int value_pin;
  if (motor == motor_a) {
    value_pin = motor_a_value_pin;
  } else {
    value_pin = motor_b_value_pin;
  }
  int analog_speed = map((int)(abs(speed) * 1000), 0, 1000, 0, 255);
  analogWrite(value_pin, analog_speed);
}

void setup() {
  Serial.begin(9600);

  // rc_channel_2.setRange(rc_min_value, rc_max_value);

  // Motor A
  pinMode(motor_a_value_pin, OUTPUT);
  pinMode(motor_a_direction_1_pin, OUTPUT);
  pinMode(motor_a_direction_2_pin, OUTPUT);
  // Motor B
  pinMode(motor_b_value_pin, OUTPUT);
  pinMode(motor_b_direction_1_pin, OUTPUT);
  pinMode(motor_b_direction_2_pin, OUTPUT);
  // RC channels
  for (int channel = 1; channel < 7; channel++) {
    int pin = rc_channel_pins[channel - 1];
    if (pin != -1) {
      pinMode(pin, INPUT);
    }
  }
  pinMode(rc_channel_pins[0], INPUT);
  pinMode(rc_channel_pins[1], INPUT);
  pinMode(rc_channel_pins[2], INPUT);
  pinMode(rc_channel_pins[3], INPUT);
}

void loop() {
  float x = read_rc_value(2);
  float y = read_rc_value(4);

  float left_wheel_speed = constrain(x + y, -1, 1);
  float right_wheel_speed = constrain(y - x, -1, 1); 

  set_speed(motor_a, -left_wheel_speed);
  set_speed(motor_b, right_wheel_speed);
}