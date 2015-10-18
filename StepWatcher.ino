




int accelerometer_previousX = 0;
int accelerometer_previousY = 0;
int accelerometer_previousZ = 0;
int accelerometer_mean = 4;
int accelerometer_tolerance = 1;

int led_Pin = 13;

long sample_time = 0;

const int accelerometer_pinX = A0;
const int accelerometer_pinY = A1;
const int accelerometer_pinZ = A2;
const int r_i_pin = A3;
const int ultrasonic_pin = A4;
const int motor_pin = 2;

long timer = 0;

void foot_on_ground();
void (*state)() = foot_on_ground;

void check_sensors() {
  
}

int read_accelerometer() {
  //TODO
}

void foot_in_air(){
  if(millis() - sample_time > 100){
    int accelerometer_signalX = analogRead(accelerometer_pinX);
    sample_time = millis();
    if(abs(accelerometer_signalX - accelerometer_previousX) > 20 && accelerometer_signalX*accelerometer_previousX < 0){
      digitalWrite(led_Pin, HIGH);
      timer = millis();
      state = foot_on_ground;
    }
    if(millis() - timer > 1000){
      digitalWrite(led_Pin, LOW);
    }
    accelerometer_previousX = accelerometer_signalX;
    delay(75);
  }
}

void foot_on_ground() {
  if(millis() - sample_time > 100){
    int accelerometer_signalX = analogRead(accelerometer_pinX);
    int accelerometer_signalY = analogRead(accelerometer_pinY);
    int accelerometer_signalZ = analogRead(accelerometer_pinZ);
    sample_time = millis();
    if(abs(accelerometer_signalZ - accelerometer_previousZ) > 10 && (accelerometer_signalZ > 700 || accelerometer_signalZ < 200)){
      digitalWrite(led_Pin, HIGH);
      //state = foot_in_air;
      //Serial.println(accelerometer_signalZ);
      timer = millis();
    }
    if(millis() - timer > 500){
      digitalWrite(led_Pin, LOW);
    }
    accelerometer_previousX = accelerometer_signalX;
    accelerometer_previousY = accelerometer_signalY;
    accelerometer_previousZ = accelerometer_signalZ;
  }
}

void setup() {
  // put your setup code here, to run once:
  accelerometer_previousX = analogRead(accelerometer_pinX);
  accelerometer_previousY = analogRead(accelerometer_pinY);
  accelerometer_previousZ = analogRead(accelerometer_pinZ);
  pinMode(led_Pin, OUTPUT);
  digitalWrite(led_Pin, LOW);
  Serial.begin(9600);
}

void loop() {
  state();
//  int readingX = analogRead(accelerometer_pinX);
//  int readingY = analogRead(accelerometer_pinY);
//  int readingZ = analogRead(accelerometer_pinZ);
//  Serial.print(readingX);
//  Serial.print(" ");
//  Serial.print(readingY);
//  Serial.print(" ");
//  Serial.print(readingZ);
//  Serial.println();
}
