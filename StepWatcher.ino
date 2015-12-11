#define TIMEVIBRATING 1000
#define MONITORINGRATE 10

int previous_ultrasonic = 110000;
int MINIMUMULTRA = 50;
int MINIMUMIR = 300;
float initial_prob[2] = { 0.5, 0.5};
float obstacle_given[2] = { 0.5, 0.5};
float us_given_obstacle[2] = { 0.9, 0.1 };
int utility[2] = { 11, -10 };

int just_detected = 0;

int pyramid_elements[2] = {1, 21};
int pyramid_length = 1;
int *buffer_accelerometer;

int previous_us = 0;

const int ir_max = 1000, us_max = 1000;
const int ir_min = 30, us_min = 30;

long monitoring_time = 0, time_already_vibrating = 0;

const int r_i_pin = A5;
const int ultrasonic_pin = A3;
const int motor_pin = 7;
const int calibration_button = 3;
int led_Pin = 5;

int problem_monitoring = 0;

int called_and_detected_check_sensor = 0;

void (*state)() = foot_in_air;

int was_detected = 0;

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
      vector = (int*)malloc(sizeof(int) * (size_v + 1));
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

void turn_on_motor() {
  digitalWrite(motor_pin, HIGH);;
}


void turn_off_motor() {
  digitalWrite(motor_pin, LOW);;
}

void calibration() {
  int ultrasonic_reading = sequence_sensors(ultrasonic_pin, 9, 5);
  int infrared_reading = sequence_sensors(r_i_pin, 9, 5);
  MINIMUMULTRA = ultrasonic_reading;
  MINIMUMIR = infrared_reading;
  just_detected = 1;

}

int sequence_sensors(int pin, int buffer_size, int number_readings) {
  Heap heap(buffer_size);
  for (int i = 0; i < buffer_size; i++) {
    heap.insert(analogRead(pin));
  }
  return heap.get_average_n_mid(number_readings);
}

void check_sensors() {
  int ultrasonic_reading = sequence_sensors(ultrasonic_pin, 11, 5);
  int us_result = ultrasonic_reading < MINIMUMULTRA;
  if (is_obstacle(us_result, us_given_obstacle)  && !called_and_detected_check_sensor) {
    turn_on_motor();
    time_already_vibrating = millis();
    was_detected = 1;
    previous_ultrasonic = ultrasonic_reading;
    called_and_detected_check_sensor = 1;
  } else {
    initial_prob[0] = 0.5;
    initial_prob[1] = 0.5;
    called_and_detected_check_sensor = 0;
  }
  state = foot_in_air;
}

void check_change() {
  int ultrasonic_reading = sequence_sensors(ultrasonic_pin, 11, 5);
  if (abs(previous_us - ultrasonic_reading) > 200 ) {
    turn_on_motor();
    time_already_vibrating = millis();
    was_detected = 1;
  }
}

//Gather all the code and test monitoring,
void foot_in_air() {
  int no_previous = 0;
  if ((millis() - time_already_vibrating > TIMEVIBRATING ) && was_detected) {
    turn_off_motor();
    was_detected = 0;
  }

  int ir_reading = sequence_sensors(r_i_pin, 9, 5);
  if (ir_reading > MINIMUMIR - 10) {
    state = check_sensors;
  }
}

int monitoring() {
  if (millis() - monitoring_time > MONITORINGRATE) {
    monitoring_time = millis();
    int ultrasonic_reading = sequence_sensors(ultrasonic_pin, 9, 5);
    int infrared_reading = sequence_sensors(r_i_pin, 9, 5);
    if (
      ultrasonic_reading > us_max || ultrasonic_reading < us_min ||
      infrared_reading > ir_max || infrared_reading < ir_min) {
      problem_monitoring = 1;
      digitalWrite(led_Pin, HIGH);
    } else {
      problem_monitoring = 0;
      digitalWrite(led_Pin, LOW);
    }
  }
}

int is_obstacle(int value_ultrasonic, float* weights) {
  float accumulator_prob[2];
  int i;
  for (i = 0; i < 2; i++) {
    accumulator_prob[i] = obstacle_given[i] * initial_prob[i];
  }
  float probability_prediction[2];
  int index_ultrasonic = abs(1 - value_ultrasonic);
  probability_prediction[0] = weights[index_ultrasonic] * accumulator_prob[0];
  probability_prediction[1] = weights[!index_ultrasonic] * accumulator_prob[1];
  float sum = probability_prediction[0] + probability_prediction[1];
  probability_prediction[0] /= sum;
  probability_prediction[1] /= sum;
  if (probability_prediction[0] == 1 && probability_prediction[1] == 0) {
    initial_prob[0] = 0.99;
    initial_prob[1] = 0.01;
  } else if (probability_prediction[0] == 0 && probability_prediction[1] == 1) {
    initial_prob[0] = 0.01;
    initial_prob[1] = 0.99;
  } else {
    initial_prob[0] = probability_prediction[0];
    initial_prob[1] = probability_prediction[1];
  }
  if (abs(probability_prediction[0] * utility[0]) > abs(probability_prediction[1] * utility[1])) {
    return true;
  }
  return false;
}

void setup() {
  pinMode(led_Pin, OUTPUT);
  pinMode(motor_pin, OUTPUT);
  digitalWrite(led_Pin, LOW);
  digitalWrite(motor_pin, LOW);
  attachInterrupt(digitalPinToInterrupt(calibration_button), calibration, FALLING);
  Serial.begin(9600);
  buffer_accelerometer = (int*)malloc(sizeof(int) * pyramid_elements[pyramid_length - 1]);
}

void loop() {
  monitoring();
  if (!problem_monitoring)
    state();
}
