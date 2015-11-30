#include "LinkedList.h"
#define SAMPLERATE 50
#define TIMEVIBRATING 10
#define MONITORINGRATE 1000

int MINIMUMULTRA = 50;
int MINIMUMIR = 300;
int initial_prob[2] = { 0.5, 0.5};
int obstacle_given[2] = { 0.5, 0.5};
int ir_given_obstacle[2] = { 0.9, 0.1 };
int us_given_obstacle[2] = { 0.8, 0.1 };
int utility[2] = { 10, -12 };

float l_g[1][21] = {{0.1640, 0.1790, 0.1752, 0.1455, 0.0865, 0, -0.1056, -0.2159, -0.3131, -0.3800, -0.4038, -0.3800,
                    -0.3131, -0.2159, -0.1056, 0, 0.0865, 0.1455, 0.1752, 0.1790, 0.1640
}};

int pyramid_elements[2] = {1, 21};
int pyramid_length = 1;
int *buffer_accelerometer; 
LinkedList* list;
int threshold_accelerometer = -350;

const int accelerometer_max = 1000, ir_max = 1000, us_max = 1000;
const int accelerometer_min = 30, ir_min = 30, us_min = 30;

int accelerometer_previousX = 0;
int accelerometer_previousY = 0;
int accelerometer_previousZ = 0;


long sample_time = 0, monitoring_time = 0;
long time_already_vibrating = 0;

const int accelerometer_pinX = A0;
const int accelerometer_pinY = A1;
const int accelerometer_pinZ = A2;
const int r_i_pin = A3;
const int ultrasonic_pin = A4;
const int motor_pin = 2;
const int calibration_button = 3;
int led_Pin = 5;

long timer = 0;
int problem_monitoring = 0;

void foot_on_ground();
void (*state)() = foot_in_air;

class Heap {
private:
  int *vector, size_v = 0, last_pos_available = 1;

  void organize(int initial_position) {
    int index_father = int(initial_position / 2);
    if (index_father == 0) {
      return;
    }
    int temp = vector[index_father];
    if (vector[index_father] > vector[initial_position]) {
      vector[index_father] = vector[initial_position];
      vector[initial_position] = temp;
      organize(index_father);
    }
  };

public:
  Heap(int size_v) {
    vector = (int*)malloc(sizeof(int)*(size_v + 1));
    vector[0] = 0;
    this->size_v = size_v;
  };

  ~Heap() {
    free(vector);
  }

  void insert(int value) {
    vector[last_pos_available] = value;
    organize(last_pos_available);
    last_pos_available++;
  };

  int get_average_n_mid(int n) {
    int i, accumulator = 0, begin = int((size_v - n) / 2) + 1;
    for (i = begin; i < n + begin; i++) {
      accumulator += vector[i];
    }
    return int(accumulator / n);
  };
};

void turn_on_motor(){
  digitalWrite(motor_pin, HIGH);;
}


void turn_off_motor(){
  digitalWrite(motor_pin, LOW);;
}

void calibration(){
  int ultrasonic_reading = sequence_sensors(ultrasonic_pin, 9, 5);
  int infrared_reading = sequence_sensors(r_i_pin, 9, 5);
  MINIMUMULTRA = ultrasonic_reading;
  MINIMUMIR = infrared_reading;

}

int sequence_sensors(int pin, int buffer_size, int number_readings){
  Heap heap(buffer_size);
  for(int i = 0; i < buffer_size; i++){
    heap.insert(analogRead(pin));  
  }
  return heap.get_average_n_mid(number_readings);
}

void check_sensors() {
  int ultrasonic_reading = sequence_sensors(ultrasonic_pin, 9, 5);
  int infrared_reading = sequence_sensors(r_i_pin, 9, 5);
  //Serial.println(infrared_reading);
  int us_result = ultrasonic_reading > MINIMUMULTRA, ir_result = infrared_reading > MINIMUMIR;
  if(is_obstacle(ir_result, us_result)){
    turn_on_motor();
    time_already_vibrating = 0;
  }else{
    turn_off_motor();
  }
  state = foot_in_air;
}

int test_accelerometer(){
  //Serial.println(list->get_size());
  if(list->get_size() != pyramid_elements[1]){
    return 0;
  }
  int i=0;
  for(; i<pyramid_length;++i){
    Serial.println(variation(l_g[i], list));
    if(variation(l_g[i], list) > threshold_accelerometer){
      //Serial.println("ok");
      return 1;
    }
  }
  return 1;
}

void foot_in_air() {
  if(millis() - sample_time > SAMPLERATE){
    int accelerometer_signalX = sequence_sensors(accelerometer_pinX, 9, 5);
    int accelerometer_signalY = sequence_sensors(accelerometer_pinY, 9, 5);
    int accelerometer_signalZ = sequence_sensors(accelerometer_pinZ, 9, 5);
    sample_time = millis();
    if(millis() - time_already_vibrating > TIMEVIBRATING)
      turn_off_motor();
    list->insert(accelerometer_signalX + accelerometer_signalY + accelerometer_signalZ);
    //list->insert(accelerometer_signalX + accelerometer_signalY + accelerometer_signalZ);
    if(test_accelerometer()){
      state = check_sensors;
    }
    accelerometer_previousX = accelerometer_signalX;
    accelerometer_previousY = accelerometer_signalY;
    accelerometer_previousZ = accelerometer_signalZ;
  }
}

int monitoring(){
  if(millis() - monitoring_time > MONITORINGRATE){
    monitoring_time = millis();
    int accelerometer_signalX = sequence_sensors(accelerometer_pinX, 9, 5);
    int accelerometer_signalY = sequence_sensors(accelerometer_pinY, 9, 5);
    int accelerometer_signalZ = sequence_sensors(accelerometer_pinZ, 9, 5);
    int ultrasonic_reading = sequence_sensors(ultrasonic_pin, 9, 5);
    int infrared_reading = sequence_sensors(r_i_pin, 9, 5);
    if(accelerometer_signalX > accelerometer_max || accelerometer_signalX < accelerometer_min ||
       accelerometer_signalY > accelerometer_max || accelerometer_signalY < accelerometer_min ||
       accelerometer_signalZ > accelerometer_max || accelerometer_signalZ < accelerometer_min ||
       ultrasonic_reading > us_max || ultrasonic_reading < us_min ||
       infrared_reading > ir_max || infrared_reading < ir_min){
       problem_monitoring = 1;
       digitalWrite(led_Pin, HIGH);
    }else{
       problem_monitoring = 0;
       digitalWrite(led_Pin, LOW);
    }
  }
}

int is_obstacle(int value_ir, int value_ultrasonic) {
  float accumulator_prob[2];
  int i;
  for (i = 0; i < 2; i++) {
    accumulator_prob[i] = obstacle_given[i] * initial_prob[i];
  }
  float probability_prediction[2];
  int index_ir = abs(1 - value_ir), index_ultrasonic = abs(1 - value_ultrasonic);
  
  probability_prediction[0] = ir_given_obstacle[index_ir] *
    us_given_obstacle[index_ultrasonic] * accumulator_prob[0];
  probability_prediction[1] = ir_given_obstacle[!index_ir] * us_given_obstacle[!index_ultrasonic]*
    accumulator_prob[1];
  
  float sum = probability_prediction[0] + probability_prediction[1];
  probability_prediction[0] /= sum;
  probability_prediction[1] /= sum;
  initial_prob[0] = probability_prediction[0];
  initial_prob[1] = probability_prediction[1];
  if (abs(probability_prediction[0] * utility[0]) > abs(probability_prediction[1] * utility[1])) {
    return true;
  }
  return false;
}

void setup() {
  accelerometer_previousX = analogRead(accelerometer_pinX);
  accelerometer_previousY = analogRead(accelerometer_pinY);
  accelerometer_previousZ = analogRead(accelerometer_pinZ);
  pinMode(led_Pin, OUTPUT);
  pinMode(motor_pin, OUTPUT);
  digitalWrite(led_Pin, LOW);
  digitalWrite(motor_pin, LOW);
  attachInterrupt(digitalPinToInterrupt(calibration_button), calibration, FALLING);
  Serial.begin(9600);
  buffer_accelerometer = (int*)malloc(sizeof(int)*pyramid_elements[pyramid_length - 1]);
  list = new LinkedList(21);
}

void loop() {
  //monitoring();
  //if(!problem_monitoring)
    state();
}
