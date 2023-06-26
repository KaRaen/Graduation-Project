#include "Arduino_FreeRTOS.h"
#include "task.h"
#include <Encoder.h>
#include <math.h>
#include "AngleConvertor.h"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
int key_last;

//define button
int BUTTON_PIN_3 = 33;
int BUTTON_PIN_2 = 37;
int BUTTON_PIN_1 = 41;

int room = 0;
int room_old;
int selected;
int pwm_turnLeft;
int pwm_turnRight;
int nums_step;
// define Laser Sensor
Adafruit_VL53L0X sensor;
double dis_obstacle;

AngleConvertor conv;
SoftwareSerial BTSerial(10, 11); // RX, TX

// define room

int dir;
int pass;
int prev_step;
int spin;
boolean finished_step;

// Define JSON DATA
long new_received_data ;
int new_received_data_2;
int new_received_data_3;
String jsonStr;

// define RTOS
static TaskHandle_t line_handle,
       i2c_handle,
       control_handle,
       odometry_handle,
       bluetooth_handle,
       print_handle;

// define odometry
// 1 is left, 2 is right
Encoder Encoder_1(2, 3);
Encoder Encoder_2(18, 19);

volatile long  new_encoder_1 ;
volatile long  new_encoder_2 ;
volatile long  old_encoder_1 ;
volatile long  old_encoder_2 ;

void encoderInterupt_1()
{
  new_encoder_1 = Encoder_1.read();
}
void encoderInterupt_2()
{
  new_encoder_2 = Encoder_2.read();
}

double x, y, theta = 0;
double x_old, y_old, theta_old = 0;

double theta_rad;

float leftTicks = 616;
float rightTicks = 616;

float deltaLeft = 0, deltaRight = 0;

double leftDia = 8.35;//cm
double rightDia = 8.35;//cm

double pi = 3.142;

double distanceLeft = 0, distanceRight = 0, distanceCenter = 0;

double L = 40.45; //cm

// Define Line_follow
int line_pins[] = {23, 25, 27, 29, 31};
boolean line_values[] = {0, 0, 0, 0, 0};

float mode = 0;
float mode_old = 0;
float mode_tam = 0;

int in1 = 4;
int in2 = 5;

int in3 = 7;
int in4 = 6;

int leftVelocity;
int rightVelocity;

int default_speed_left = 100;
int default_speed_right = 58;

void leftMotor(int leftSpeed)
{
  analogWrite(in1, leftSpeed);
  digitalWrite(in2, LOW);

}
void rightMotor(int rightSpeed)
{
  analogWrite(in3, rightSpeed);
  digitalWrite(in4, LOW);
}

void button()
{
  int buttonState1 = digitalRead(BUTTON_PIN_1);
  int buttonState2 = digitalRead(BUTTON_PIN_2);
  int buttonState3 = digitalRead(BUTTON_PIN_3);
  if (selected == 0)
  {
    if (buttonState1 == LOW) {
      room = 1;
      selected = 1;
    }
    if (buttonState2 == LOW) {
      room = 2;
      selected = 1;
    }
    if (buttonState3 == LOW) {
      room = 3;
      selected = 1;
    }
  }
  else if (selected == 1)
  {
    room = room_old;
    if ((buttonState1 == HIGH) && (buttonState2 == HIGH) && (buttonState3 == HIGH))
    {
      room = 0;
      lcd.clear();
      selected = 0;
    }
  }
}
// assign velocity equal default speed with leap
void pwmLeap(int leftLeap, int rightLeap)
{
  leftVelocity = default_speed_left + leftLeap;
  rightVelocity = default_speed_right + rightLeap;
  leftMotor(leftVelocity);
  rightMotor(rightVelocity);
}
// increase or decrease velocity gradually
void pwmGradual(int leftCoefficient, int rightCoefficient)
{
  leftVelocity = leftVelocity + leftCoefficient;
  rightVelocity = rightVelocity + rightCoefficient;

  leftVelocity = constrain(leftVelocity, 100, 140);
  rightVelocity = constrain(rightVelocity, 53, 60);

  leftMotor(leftVelocity);
  rightMotor(rightVelocity);

  xTaskNotifyGive(line_handle);
  //vTaskDelay(pdMS_TO_TICKS(3));
}

void Stop()
{
  while ((new_encoder_1 != old_encoder_1) || (new_encoder_2 != old_encoder_2))
  {
    rightMotor(0);
    leftMotor(0);
  }
}
void Turn_right()
{
  float rightAngle;
  if (room == 3) {
    pwm_turnRight = 160;
    rightAngle = -1.7;

  }
  else if (room == 1) {
    pwm_turnRight = 120;
    rightAngle = 0.25;

  }
  else
  {
    pwm_turnRight = 140;
    rightAngle = -1.25;
  }
  while  (theta > rightAngle)
  {
    rightMotor(0);
    leftMotor(pwm_turnRight);
    xTaskNotifyGive(line_handle);
  }
  spin = 0;
  x = 0;
  y = 0;
  theta = 0;

  xTaskNotifyGive(bluetooth_handle);
  Stop();
}
void Turn_left()
{
  float leftAngle;
  if (room == 3) {
    leftAngle = 0.1;
    pwm_turnLeft = 80;
  }
  else if (room == 1) {
    leftAngle = 1.86;
    pwm_turnLeft = 120;
  }
  else
  {
    leftAngle = 0.1;
    pwm_turnLeft = 135;
  }
  while (theta < leftAngle)
  {
    rightMotor(pwm_turnLeft);
    leftMotor(0);
    xTaskNotifyGive(line_handle);
  }
  spin = 0;
  x = 0;
  y = 0;
  theta = 0;

  xTaskNotifyGive(bluetooth_handle);
  Stop();
}
void Forward()
{
  // right deviation
  if ((mode < 3) && (mode != 0))
  {
    pwmLeap(0, 1);
    while ((mode < 3) && (mode != 0))
    {
      pwmGradual(-1, 1);
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
  // left deviation
  else if ((mode > 3) && (mode != 6))
  {
    pwmLeap(3, -1);
    while ((mode > 3) && (mode != 6))
    {
      pwmGradual (1, -1);
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
  // Center
  else if (mode == 3)
  {
    pwmLeap(0, 0);
    while (mode == 3)
    {
      pwmGradual (0, 0);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  else if (mode == 6)
  {
    if (dir == 1)
    {
      // dieu chinh thong so khi dang trong line to
      if (mode_old == 0)
      {
        pwmLeap(40, 40);
      }
      // lech phai
      else if ((mode_old < 3) && (mode_old != 0))
      {
        pwmLeap(0, 1);
        while (mode == 6)
        {
          pwmGradual(0, 1);
          vTaskDelay(pdMS_TO_TICKS(2));
        }
      }
      // lech trai
      else if (mode_old > 3)
      {
        //18
        pwmLeap(8, -2);
        while (mode == 6)
        {
          pwmGradual (1, -1);
          vTaskDelay(pdMS_TO_TICKS(2));
        }
      }
    }
    if (dir != 1)
    {
      xTaskNotifyGive(control_handle);
    }
  }
  // Not detected
  else
  {
    Stop();
  }
}

void Bluetooth(void *prm)
{
  while (1) {
    // Tạo một đối tượng JSON
    StaticJsonDocument<300> jsonBuffer;
    JsonObject json = jsonBuffer.to<JsonObject>();

    // Đặt các trường dữ liệu

    json["theta"] = theta;
    json["spin"] = spin;
    json["mode"] = mode;
    json["pass"] = pass;
    json["prev_step"] = prev_step;
    json["dis_obstacle"] = dis_obstacle;
    json["room"] = room;
    json["x"] = x;
    json["y"] = y;
    // Chuyển đổi JSON thành chuỗi
    String jsonString;
    serializeJson(json, jsonString);

    // Gửi chuỗi JSON qua Bluetooth
    BTSerial.println(jsonString);

    if (BTSerial.available())
    {
      jsonStr = BTSerial.readStringUntil('\n');
      DynamicJsonDocument doc(200);
      DeserializationError error = deserializeJson(doc, jsonStr);
      if (error) {
        //   Serial.println("Error parsing JSON");
        pass = 0;
      }
      else pass = 1;

      new_received_data = doc["received_data"];
      new_received_data_2 = doc["nums_step"];
      new_received_data_3 = doc["key_last"];
      if (new_received_data_3 != key_last)
      {
        key_last = new_received_data_3;
      }
      if (new_received_data_2 != nums_step)
      {
        nums_step = new_received_data_2;
      }
      if (new_received_data != dir)
      {
        dir = new_received_data;
      }
    }
    xTaskNotifyGive(control_handle);
    // Chờ 100ms trước khi gửi lại
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

void Line_Follower(void *prm)
{
  while (1)
  {
    float number_pin = 0;
    float total_value = 0;

    for (int i = 0; i < 5; i++)
    {
      line_values[i] = !digitalRead(line_pins[i]); // have line = 1
      number_pin = number_pin + line_values[i];
      total_value = total_value + line_values[i] * (i + 1) * 1;
    }
    if (number_pin == 0) {
      mode = 0;
    }
    else if (number_pin == 5) {
      mode = 6;
      if (finished_step == false)
      {
        Stop();
        prev_step = prev_step + 1;
        spin = 1;
        finished_step = true;
      }
      xTaskNotifyGive(control_handle);
      xTaskNotifyGive(bluetooth_handle);
    }
    else {
      finished_step = false;
      mode = total_value / number_pin;
    }
    if (mode_tam != mode)
    {
      mode_old = mode_tam;
      mode_tam = mode;

    }
    xTaskNotifyGive(control_handle);
    vTaskDelay(pdMS_TO_TICKS(10));// sau 10ms quay lai
  }
}

void I2C(void *prm)
{
  while (1)
  {
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      dis_obstacle = measure.RangeMilliMeter;
    }
    else {
      dis_obstacle = 3000;
    }
    dis_obstacle = constrain(dis_obstacle, 30, 3000);

    button();
    if (room_old != room )
    {
      if (room == 0)
      {
        lcd.setCursor(7, 0);
        lcd.print("DE TAI");

        lcd.setCursor(2, 1);
        lcd.print("ROBOT VAN CHUYEN");

        lcd.setCursor(1, 2);
        lcd.print("XIN MOI CHON PHONG");

        lcd.setCursor(0, 3);
        lcd.print("3");

        lcd.setCursor(10, 3);
        lcd.print("2");

        lcd.setCursor(19, 3);
        lcd.print("1");

      }
      if (room != 0)
      {
        lcd.clear();
        lcd.setCursor(7, 1);
        lcd.print("PHONG ");
        lcd.print(room);
        // Lưu trạng thái hiện tại vào trạng thái trước đó
        room_old = room;
      }
    }
    xTaskNotifyGive(bluetooth_handle);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Control(void *prm)
{
  while (1)
  {
    if (prev_step < nums_step)
    {
      if (dir == 1)
      {
        Forward();
      }
      else if (dir == 2)
      {
        Turn_right();
      }
      else if (dir == 3)
      {
        Turn_left();
      }
      else if (dir == 0)
      {
        Stop();
      }
    }
    else
    {
      Stop();
      if (key_last == 1)
      {
        prev_step = 1;
        nums_step = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void Odometry(void *prm)
{
  while (1)
  {
    if ((new_encoder_1 != old_encoder_1) || (new_encoder_2 != old_encoder_2))
    {
      // Nếu giá trị encoder đã thay đổi, mới thực hiện tính toán
      // 1 là phải, 2 là trái
      deltaRight = new_encoder_1 - old_encoder_1;
      deltaLeft = new_encoder_2 - old_encoder_2;

      distanceLeft = pi * deltaLeft * leftDia / (2 * leftTicks);// van toc banh trai
      distanceRight = pi * deltaRight * rightDia / (2 * rightTicks);
      distanceCenter = (distanceLeft + distanceRight) / 2;

      theta = theta_old + (distanceRight - distanceLeft) / L;
      conv.setDegrees(theta);
      theta_rad = conv.getRadians();

      x = x_old + cos(theta) * distanceCenter;
      y = y_old + sin(theta) * distanceCenter;

      x_old = x;
      y_old = y;
      theta_old = theta;

      old_encoder_1 = new_encoder_1;
      old_encoder_2 = new_encoder_2;

    }

    vTaskDelay(pdMS_TO_TICKS(1));// sau 10ms quay lai
  }
}

void Print(void *prm)
{
  while (1)
  {
    //    Serial.println(nums_step);
    //        Serial.println(room);
    //    Serial.print(dis_obstacle); Serial.print("\t");
    //    Serial.print(mode_old); Serial.print ("\t");
    //    Serial.println(mode);
    //    Serial.print(dir); Serial.print ("\t");
    //    Serial.println(prev_step);
    // in encoder xung
    //    Serial.print (x); Serial.print ("\t");
    //    Serial.print (y); Serial.print ("\t");
    //    Serial.print (theta); Serial.print ("\t");
    //    Serial.println (theta_rad);
    // in encoder xung
    //  Serial.print (old_encoder_1);Serial.print ("\t");
    //  Serial.println (old_encoder_2);
    // in do line
    //  Serial.println(mode);Serial.print(" ");
    //  Serial.print(line_values[0]);Serial.print("\t");
    //  Serial.print(line_values[1]);Serial.print("\t");
    //  Serial.print(line_values[2]);Serial.print("\t");
    //0  Serial.print(line_values[3]);Serial.print("\t");
    //  Serial.println(line_values[4]);
    vTaskDelay(pdMS_TO_TICKS(1));// sau 10ms quay lai
  }
}

void setup() {
  // Khởi tạo các thành phần cơ bản của FreeRTOS
  // ..
  Serial.begin(9600);
  BTSerial.begin(9600);
  Wire.begin();
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP);
  pinMode(BUTTON_PIN_3, INPUT_PULLUP);

  lcd.init();
  lcd.clear();
  lcd.backlight();

  lcd.setCursor(7, 0);
  lcd.print("DE TAI");

  lcd.setCursor(2, 1);
  lcd.print("ROBOT VAN CHUYEN");

  lcd.setCursor(1, 2);
  lcd.print("XIN MOI CHON PHONG");

  lcd.setCursor(0, 3);
  lcd.print("3");

  lcd.setCursor(10, 3);
  lcd.print("2");

  lcd.setCursor(19, 3);
  lcd.print("1");
  if (!sensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  //  // power
  //  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  for (int i = 0; i < 5; i++)
  {
    pinMode (line_pins[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(2), encoderInterupt_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), encoderInterupt_2, CHANGE);// chinh chieu encoder cho nay
  // Tạo và khởi chạy các task
  xTaskCreate(Bluetooth, "Bluetooth", 1024, NULL, tskIDLE_PRIORITY + 1, &bluetooth_handle);
  xTaskCreate(Control, "Control", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &control_handle);
  xTaskCreate(Line_Follower, "Line_Follower", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &line_handle);
  xTaskCreate(I2C, "I2C", 1024, NULL, tskIDLE_PRIORITY + 1, &i2c_handle);

  xTaskCreate(Odometry, "Odometry", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &odometry_handle);
  xTaskCreate(Print, "Print", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &print_handle);
  // Bắt đầu quá trình lập lịch và thực thi các task
  vTaskStartScheduler();

}

void loop()
{}
