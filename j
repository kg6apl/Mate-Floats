// For 2025 MATEROV Season

#include <Servo.h>
#include <SPI.h>
Servo myservo;
  
// pins
const int pressure = A1;
const int temp = A0;
const int button = 4;
const int servo = 9;

// enter pool_depth in cm
const int pool_depth = 220;

// target depths
const int target_depths_size = 4;
int target_depths[target_depths_size] = {0, 20, 110, 210}; // example numbers if pool is 2.2m deep, is changed in setup()
  
const int num_points = 22;
const long raw_values[num_points] = {8316, 8300, 8703, 8830, 9115, 9335, 9744, 10000, 10370, 10655, 11071, 11430, 11790, 12080, 12327, 12745, 13112, 13358, 13630, 13960, 14255, 14694};
const int depths[num_points] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210};

long pres; // holds current depth
  
// motor values 
const int motor_stop = 91;
const int max_motor_down = 55;
const int max_motor_up = 103;
const int motor_hover = 79;
const int max_error_range = 35; // used for map() function as the upper bound

void setup() 
{
  target_depths[3] = pool_depth - 25;
  target_depths[2] = (pool_depth / 2) - 25;
  
  Serial.begin(9600);

  pinMode(2, INPUT);   // HX710 DOUT
  pinMode(3, OUTPUT);  // HX710 SCK
  pinMode(button, INPUT_PULLUP);

  delay(1000);
  myservo.attach(servo,1000,2000);
  Serial.println("Initializing ESC");
  myservo.write(180);
  delay(5000);
  myservo.write(motor_stop);
  delay(1000);
  Serial.println("ESC Initialized");
  delay(3000);
}
  
void loop() 
{ 
  Serial.println("press button to start program");
  while (digitalRead(button) == HIGH) {}
  delay(300);

  for (int i = 0; i < target_depths_size; i++)
  { 
    if (i == 0)
    {
      Serial.println("press button again to take air temp");
      while (digitalRead(button) == HIGH) {}
      delay(300);
      log_data(30, get_temperature());
      continue;
    }
    if (i == 1)
    {
      Serial.println("press button once MATE Float is in the water");
      while (digitalRead(button) == HIGH) {}
      delay(300);
      log_data(-20, get_temperature());
      continue;
    }

    go_to_depth(target_depths[i]);  
    hover(target_depths[i]);  
    log_data(-get_depth(), get_temperature()); 

    if (i == target_depths_size - 1)
    {
      // Return to surface or stop motor if needed
    }
  }
}

void ascent()
{
  Serial.println("ascending to surface...");
  go_to_depth(30);
  Serial.println("successful");
  myservo.write(motor_stop);
}

void go_to_depth(int target)
{
  Serial.print("going to depth "); Serial.println(target);

  int speed; 
  int error;

  while (abs(get_depth() - target) > 5)
  {
    long temp_depth = get_depth();
    Serial.print("depth = "); Serial.println(temp_depth);

    error = temp_depth - target;

    if (error < 0)
    {
      speed = map(abs(error), 0, 50, motor_hover, max_motor_down);
      speed = constrain(speed, max_motor_down, motor_hover);
      myservo.write(speed);
    }
    else
    {
      speed = map(abs(error), 0, max_error_range, motor_hover, max_motor_up);
      speed = constrain(speed, motor_hover, max_motor_up);
      myservo.write(speed);
    }
  }

  Serial.print("going to depth "); Serial.print(target); Serial.println(" successful"); 
}

void hover(int temp_target)
{
  Serial.print("hovering at "); Serial.println(temp_target);

  const unsigned long hover_duration = 2000; 
  unsigned long start_time = millis();

  while (true)
  {
    long temp_depth = get_depth();
    Serial.print("depth = "); Serial.println(temp_depth);

    if (abs(temp_depth - temp_target) <= 25)
    {
      if (millis() - start_time >= hover_duration)
      {
        Serial.println("hover complete");
        break;
      }
      myservo.write(motor_hover);
    }
    else
    {
      Serial.println("out of range, resetting countdown");
      start_time = millis(); 

      go_to_depth(temp_target);
    }
  }
}

long read_sensor() {
  while (digitalRead(2)) {}

  long result = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(3, HIGH);
    digitalWrite(3, LOW);
    result = result << 1;
    if (digitalRead(2)) {
      result++;
    }
  }
  result = result ^ 0x800000;

  for (char i = 0; i < 3; i++) {
    digitalWrite(3, HIGH);
    digitalWrite(3, LOW);
  }

  return result;
}

float interpolate_depth(long raw_value) 
{
  long rounded_value = round(raw_value / 1000.0);

  if (rounded_value <= raw_values[0]) return depths[0];
  if (rounded_value >= raw_values[num_points - 1]) 
  {
    float slope = (float)(depths[num_points - 1] - depths[num_points - 2]) / (raw_values[num_points - 1] - raw_values[num_points - 2]);
    return depths[num_points - 1] + slope * (rounded_value - raw_values[num_points - 1]);
  }

  for (int i = 0; i < num_points - 1; i++) 
  {
    if (rounded_value >= raw_values[i] && rounded_value <= raw_values[i + 1]) 
    {
      float slope = (float)(depths[i + 1] - depths[i]) / (raw_values[i + 1] - raw_values[i]);
      return depths[i] + slope * (rounded_value - raw_values[i]);
    }
  }
  return -1;
}

float get_depth() 
{
  long raw_data = read_sensor();
  return interpolate_depth(raw_data);
}

float get_temperature() 
{
  int adcVal = analogRead(temp);
  float v = adcVal * 5.0 / 1024;
  float Rt = 10 * v / (5 - v);
  float tempK = 1 / (log(Rt / 10) / 3950 + 1 / (273.15 + 25));
  float tempC = tempK - 273.15;
  return tempC;
}

void log_data(float temp_depth, float temp_temp)
{
  Serial.print(temp_depth);
  Serial.print(",");
  Serial.println(temp_temp);
}
