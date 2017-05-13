#include <PS3BT.h>
#include <usbhub.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

Adafruit_PCD8544 display = Adafruit_PCD8544(29, 31, 33, 35, 37); // (42, 43, 44, 45, 46);

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

// Константы
#define BRAKE 0
#define CW 1
#define CCW 2
#define CS_THRESHOLD 15 //безопасное значение тока

//MOTOR 1
#define MOTOR_A1_PIN 25 //7
#define MOTOR_B1_PIN 24 //8
#define PWM_MOTOR_1 45  //5

//MOTOR 2
#define MOTOR_A2_PIN 23 //4
#define MOTOR_B2_PIN 22 //9
#define PWM_MOTOR_2 46  //6

#define CURRENT_SEN_1 A8 //A2 // датчик тока М1
#define CURRENT_SEN_2 A9 //A3 // датчик тока М2

#define ENABLE_PIN_1 A0 // enable for motor 1
#define ENABLE_PIN_2 A1 // enable for motor 2

#define MOTOR_1 0
#define MOTOR_2 1

// реле
#define RELAY1 28
#define RELAY2 30
#define RELAY3 32
#define RELAY4 34

// кнопка передача вверх
#define GEARUP_BUTTON 36
// кнопка передача вниз
#define GEARDOWN_BUTTON 38
// кнопка задней передачи
#define BACKWARDGEAR_BUTTON 40
// педаль
#define PEDAL_PIN 42 //31

// шаг увеличения скорости
#define SPEEDUPSTEP 2
// шаг уменьшения скорости
#define BREAKSTEP 4

// текущая скорость. положительные значения - движение вперед, отрицательные - назад
float CurrentSpeed = 0;
// текущая передача
int8_t CurrentGear = 1;
// лимиты скорости для передач
uint8_t SpeedLimits[] = {64, 128, 192, 255};
// максимальная скорость
uint8_t MaxSpeed = 255;
// разрешено ли ручное управление
bool IsManualEnabled = true;

uint8_t Relay1State = HIGH;
uint8_t Relay2State = HIGH;
uint8_t Relay3State = HIGH;
uint8_t Relay4State = HIGH;

bool IsConnected = false;

void setup()
{
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial)
    ; // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ; //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));

  pinMode(PEDAL_PIN, INPUT_PULLUP);
  pinMode(GEARUP_BUTTON, INPUT_PULLUP);
  pinMode(GEARDOWN_BUTTON, INPUT_PULLUP);
  pinMode(BACKWARDGEAR_BUTTON, INPUT_PULLUP);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);

  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  // pinMode(CURRENT_SEN_1, INPUT);
  // pinMode(CURRENT_SEN_2, INPUT);

  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);

  digitalWrite(ENABLE_PIN_1, HIGH);
  digitalWrite(ENABLE_PIN_2, HIGH);

  digitalWrite(RELAY1, Relay1State);
  digitalWrite(RELAY2, Relay1State);
  digitalWrite(RELAY3, Relay1State);
  digitalWrite(RELAY4, Relay1State);

  display.begin();
  display.clearDisplay();
  display.setContrast(60);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  //display.setFont(&Picopixel);
  display.setCursor(0, 0);
  display.println("Waiting");
  display.println("for");
  display.println("connection");
  display.display();
}

void loop()
{
  Usb.Task();

  if (PS3.PS3Connected)
  {
    IsConnected = true;

    // повороты руля правой хаткой
    float steer = (PS3.getAnalogHat(RightHatX) - 127.5) * 2;

    // повороты руля кнопками LEFT/RIGHT
    if (PS3.getButtonPress(LEFT))
    {
      steer = -255;
    }

    if (PS3.getButtonPress(RIGHT))
    {
      steer = 255;
    }

    if (steer < -255)
    {
      steer = -255;
    }
    if (steer > 255)
    {
      steer = 255;
    }

    if (-20 > steer || steer > 20)
    {
      Serial.print("\r\nSteer: ");
      Serial.print(steer);
      MotorGo(MOTOR_2, steer);
    }
    else
    {
      MotorGo(MOTOR_2, 0);
    }

    // вперед/назад левой хаткой
    uint8_t leftHatYval = PS3.getAnalogHat(LeftHatY);
    if (leftHatYval > 137 || leftHatYval < 117)
    {
      ChangeSpeedWithRightHatY(leftHatYval);

      goto SetMotorSpeed;
    }

    // движение вперед курком
    if (PS3.getAnalogButton(R2))
    {
      CurrentSpeed += SPEEDUPSTEP;

      if (CurrentSpeed > SpeedLimits[CurrentGear - 1])
      {
        CurrentSpeed = SpeedLimits[CurrentGear - 1];
      }

      Serial.print(F("\r\nR2: "));
      Serial.print(PS3.getAnalogButton(R2));

      goto SetMotorSpeed;
    }

    // движение назад курком
    if (PS3.getAnalogButton(L2))
    {
      CurrentSpeed -= SPEEDUPSTEP;

      if (CurrentSpeed < -SpeedLimits[CurrentGear - 1])
      {
        CurrentSpeed = -SpeedLimits[CurrentGear - 1];
      }

      Serial.print(F("\r\nL2: "));
      Serial.print(PS3.getAnalogButton(L2));

      goto SetMotorSpeed;
    }

    // движение вперед/назад педелью
    if (digitalRead(PEDAL_PIN) == LOW && IsManualEnabled)
    {
      if (digitalRead(BACKWARDGEAR_BUTTON) == HIGH)
      {
        CurrentSpeed += SPEEDUPSTEP;

        if (CurrentSpeed > SpeedLimits[CurrentGear - 1])
        {
          CurrentSpeed = SpeedLimits[CurrentGear - 1];
        }
      }
      else // digitalRead(BACKWARDGEAR_BUTTON) == LOW
      {
        CurrentSpeed -= SPEEDUPSTEP;

        if (CurrentSpeed < -SpeedLimits[CurrentGear - 1])
        {
          CurrentSpeed = -SpeedLimits[CurrentGear - 1];
        }
      }

      goto SetMotorSpeed;
    }

    // если не нажаты кнопки/хатки плавно сбрасываем скорость до нуля
    if (CurrentSpeed < 0)
    {
      CurrentSpeed += BREAKSTEP;
      if (CurrentSpeed > 0)
      {
        CurrentSpeed = 0;
      }
    }
    else if (CurrentSpeed > 0)
    {
      CurrentSpeed -= BREAKSTEP;
      if (CurrentSpeed < 0)
      {
        CurrentSpeed = 0;
      }
    }

  SetMotorSpeed:
    MotorGo(MOTOR_1, CurrentSpeed);

    if (PS3.getButtonClick(PS))
    {
      Serial.print(F("\r\nPS"));
      //MotorGo(MOTOR_1, 0);
      //MotorGo(MOTOR_2, 0);
      //PS3.disconnect();
    }
    else
    {
      if (PS3.getButtonPress(TRIANGLE))
      {
        digitalWrite(RELAY1, LOW);
        Serial.print(F("\r\nTraingle down"));
      }
      else
      {
        digitalWrite(RELAY1, HIGH);
        //Serial.print(F("\r\nTraingle up"));
      }

      if (PS3.getButtonClick(CIRCLE))
      {
        if (Relay2State == HIGH)
        {
          Relay2State = LOW;
        }
        else
        {
          Relay2State = HIGH;
        }
        digitalWrite(RELAY2, Relay2State);
        Serial.print(F("\r\nCircle"));
      }

      if (PS3.getButtonClick(CROSS))
      {
        if (Relay3State == HIGH)
        {
          Relay3State = LOW;
        }
        else
        {
          Relay3State = HIGH;
        }
        digitalWrite(RELAY3, Relay3State);
        Serial.print(F("\r\nCross"));
      }

      if (PS3.getButtonClick(SQUARE))
      {
        if (Relay4State == HIGH)
        {
          Relay4State = LOW;
        }
        else
        {
          Relay4State = HIGH;
        }
        digitalWrite(RELAY4, Relay4State);
        Serial.print(F("\r\nSquare"));
      }

      if (PS3.getButtonClick(UP) || (digitalRead(GEARUP_BUTTON) == LOW && IsManualEnabled))
      {
        CurrentGear += 1;
        if (CurrentGear > 4)
        {
          CurrentGear = 4;
        }

        Serial.print(F("\r\nUp"));
        SetGearLed(CurrentGear);
      }

      if (PS3.getButtonClick(DOWN) || (digitalRead(GEARDOWN_BUTTON) == LOW && IsManualEnabled))
      {
        CurrentGear -= 1;
        if (CurrentGear < 1)
        {
          CurrentGear = 1;
        }
        Serial.print(F("\r\nDown"));
        SetGearLed(CurrentGear);
      }

      if (PS3.getButtonClick(L1))
        Serial.print(F("\r\nL1"));

      if (PS3.getButtonClick(R1))
        Serial.print(F("\r\nR1"));

      if (PS3.getButtonClick(L3))
      {
        Serial.print("\r\CURRENT_SEN_1: ");
        Serial.print(analogRead(CURRENT_SEN_1));
      }

      if (PS3.getButtonClick(R3))
      {
        Serial.print("\r\CURRENT_SEN_2: ");
        Serial.print(analogRead(CURRENT_SEN_2));
      }

      if (PS3.getButtonClick(SELECT))
      {
        IsManualEnabled = !IsManualEnabled;
        CurrentSpeed = 0;
        Serial.print(F("\r\nSelect - "));
        PS3.printStatusString();
      }
      if (PS3.getButtonClick(START))
      {
        Serial.print(F("\r\nStart"));
      }
    }
    DisplayTelemetry();
  } // if (PS3.PS3Connected)
  else
  {
    if (IsConnected)
    {
      IsConnected = false;
      display.setCursor(0, 0);
      display.println("Waiting");
      display.println("for");
      display.println("connection");
      display.display();
    }
  }
}

void DisplayTelemetry()
{
  display.clearDisplay();
  display.setCursor(0, 0);

  display.print("Gear: ");
  if (digitalRead(BACKWARDGEAR_BUTTON) == HIGH)
  {
    display.println(CurrentGear);
  }
  else
  {
    display.println("R");
  }

  display.print("S: ");
  display.println(round(CurrentSpeed));

  display.display();
}

// Функция установки
// motor(0 or 1) - выбор двигателя
// direction (cw or ccw) - направление
// pwm (0..255) - значение скорости
void MotorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor == MOTOR_1)
  {
    if (direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if (direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }

    analogWrite(PWM_MOTOR_1, pwm);
  }
  else if (motor == MOTOR_2)
  {
    if (direct == CW)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if (direct == CCW)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_2, pwm);
  }
}

// motor (0 or 1) - выбор двигателя
// speed (-255..+255) - скорость, отрицательные значения - движение назад
void MotorGo(uint8_t motor, float speed)
{
  short direction;
  if (speed > 0)
  {
    direction = CW;
  }
  else if (speed < 0)
  {
    direction = CCW;
  }
  else
  {
    direction = BRAKE;
  }

  MotorGo(motor, direction, round(abs(speed)));
}

void ChangeSpeedWithRightHatY(uint8_t hatValue)
{
  float speedChange;
  if (hatValue < 128)
  { // forward
    speedChange = (128 - hatValue) / 128.0 * SPEEDUPSTEP;
  }
  else
  { // backward
    speedChange = -(hatValue - 128) / 128.0 * SPEEDUPSTEP;
  }

  CurrentSpeed += speedChange;

  if (CurrentSpeed > MaxSpeed)
  {
    CurrentSpeed = MaxSpeed;
  }

  if (CurrentSpeed < -MaxSpeed)
  {
    CurrentSpeed = -MaxSpeed;
  }

  Serial.print("\r\nRightHatY: ");
  Serial.print(hatValue);
  Serial.print("  SpeedChange: ");
  Serial.print(speedChange);
  Serial.print("  Current speed: ");
  Serial.print(CurrentSpeed);
}

void SetGearLed(int gear)
{
  PS3.setLedRaw(1 << (gear - 1));
  Serial.print("\r\nCurrent gear:");
  Serial.print(gear);
}
