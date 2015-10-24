


#define MINIMUMULTRA 300
#define MINIMUMIR 300
#define SAMPLERATE 100
#define TIMEVIBRATING 10

int accelerometer_previousX = 0;
int accelerometer_previousY = 0;
int accelerometer_previousZ = 0;
int led_Pin = 13;

long sample_time = 0;
long time_already_vibrating = 0;

const int accelerometer_pinX = A0;
const int accelerometer_pinY = A1;
const int accelerometer_pinZ = A2;
const int r_i_pin = A3;
const int ultrasonic_pin = A4;
const int motor_pin = 2;

long timer = 0;

void foot_on_ground();
void (*state)() = foot_in_air;

void turn_on_motor(){
  digitalWrite(motor_pin, HIGH);;
}


void turn_off_motor(){
  digitalWrite(motor_pin, LOW);;
}

void check_sensors() {
  int ultrasonic_reading = analogRead(ultrasonic_pin);
  int infrared_reading = analogRead(r_i_pin);
  if(ultrasonic_reading < MINIMUMULTRA || infrared_reading < MINIMUMIR)
    turn_on_motor();
    time_already_vibrating = 0;
  state = foot_in_air;
}

void foot_in_air() {
  if(millis() - sample_time > SAMPLERATE){
    int accelerometer_signalX = analogRead(accelerometer_pinX);
    int accelerometer_signalY = analogRead(accelerometer_pinY);
    int accelerometer_signalZ = analogRead(accelerometer_pinZ);
    sample_time = millis();
    if(millis() - time_already_vibrating > TIMEVIBRATING)
      turn_off_motor();
    if(abs(accelerometer_signalZ - accelerometer_previousZ) > 10 && (accelerometer_signalZ > 700 || accelerometer_signalZ < 200)){
      state = check_sensors; 
    }
    accelerometer_previousX = accelerometer_signalX;
    accelerometer_previousY = accelerometer_signalY;
    accelerometer_previousZ = accelerometer_signalZ;
  }
}

void setup() {
  accelerometer_previousX = analogRead(accelerometer_pinX);
  accelerometer_previousY = analogRead(accelerometer_pinY);
  accelerometer_previousZ = analogRead(accelerometer_pinZ);
  pinMode(led_Pin, OUTPUT);
  pinMode(motor_pin, OUTPUT);
  digitalWrite(led_Pin, LOW);
  digitalWrite(motor_pin, LOW);
  Serial.begin(9600);
}

void loop() {
  state();
}
