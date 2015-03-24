#include <Servo.h>

#define INTERVAL_EXEC(ID, INTERVAL, FUNC, ...) \
  do { \
    static unsigned long long nextExecOf_##FUNC_##ID = 0; \
    if(nextExecOf_##FUNC_##ID < (millis())) \
    { \
      FUNC(__VA_ARGS__); \
      nextExecOf_##FUNC_##ID += INTERVAL; \
    } \
  }while(0)

// Motors ------------------------------------------------------------------------------------------------

#define LeftMotorControl 6
#define lmf 9
#define lmb 11
#define RightMotorControl 5
#define rmf 8
#define rmb 7

void motorControl(int leftSpeed, int rightSpeed)
{
  if(leftSpeed >= 0)
  {
    digitalWrite(lmf, HIGH);
    digitalWrite(lmb, LOW);
  }
  else
  {
    digitalWrite(lmf, LOW);
    digitalWrite(lmb, HIGH);
  }
  
  if(rightSpeed >= 0)
  {
    digitalWrite(rmf, HIGH);
    digitalWrite(rmb, LOW);
  }
  else
  {
    digitalWrite(rmf, LOW);
    digitalWrite(rmb, HIGH);
  }
  
  analogWrite(LeftMotorControl, (abs(leftSpeed)));
  analogWrite(RightMotorControl, (abs(rightSpeed)));
}

#define spd 100

#define mstop() motorControl(0, 0)
#define t_left() motorControl((-spd - 10), spd)
#define t_right() motorControl((spd + 10), -spd - 10)
#define go() motorControl((spd + 10), spd)

// Sensor -----------------------------------------------------------------------------------------------

Servo sensor;

#define SensorLookLeft 180
#define SensorLookRight 6
#define SensorLookForward 90

#define ServoControl 3
#define SensorTrig 10
#define SensorEcho 12

#define SensorControl(DirectionToLookIn) sensor.write(DirectionToLookIn)

#define MAX_SEN_SAMPLES 5

#define l_sen A0

class OptSenData
{
  private:
    int sen_data[MAX_SEN_SAMPLES];
    int index;
    int sensor;
    
  public:
    OptSenData(int sensor)
    {
      this->index = 0;
      this->sensor = sensor;
    }
    
    void setReading()
    {
      int data = analogRead(this->sensor);
      this->sen_data[this->index++] = data;
      this->index %= MAX_SEN_SAMPLES;
    }
    
    int getReading()
    {
      int sum = 0;
      for(int i = 0; i < MAX_SEN_SAMPLES; i++)
      {
        sum += this->sen_data[i];
      }
      
      sum /= MAX_SEN_SAMPLES;
      return sum;
    }
};

class SonarSenData
{
  private:
    int sen_data[MAX_SEN_SAMPLES];
    int index;
    int sensor_trig;
    int sensor_echo;
    
  public:
    SonarSenData(int sensor_trig, int sensor_echo) : 
    index(0), sensor_trig(sensor_trig), sensor_echo(sensor_echo)
    {
    }
    
    void setReading()
    {
      unsigned int data = sensorRead();
      this->sen_data[this->index++] = data;
      this->index %= MAX_SEN_SAMPLES;
    }
    
    int getReading()
    {
      int sum = 0;
      for(int i = 0; i < MAX_SEN_SAMPLES; i++)
      {
        sum += this->sen_data[i];
      }
      
      sum /= MAX_SEN_SAMPLES;
      return sum;
    }
    
    private:
    
    unsigned long sensorRead()
    {
      unsigned long duration;
    
      digitalWrite(this->sensor_trig, LOW); 
      delayMicroseconds(2); 
    
      digitalWrite(this->sensor_trig, HIGH);
      delayMicroseconds(10); 
     
      digitalWrite(this->sensor_trig, LOW);
      duration = pulseIn(this->sensor_echo, HIGH);
      
      return duration;
    }
};

// PID -------------------------------------------------------------------------------------------------

#define Kp 0.3
#define Kd 0.9
#define target 450
#define interval 100

int do_PD(unsigned int sen_data)
{
  static float prev_error;
  float error;
  float data;
  
  error = target - sen_data;
  data = error * Kp;
  data += (error - prev_error) * Kd;
  prev_error = error;

  return (int)error;
}

// Control ---------------------------------------------------------------------------------------------

void execute_control(unsigned int sen_data)
{
  int adjustment = do_PD(sen_data);
  int left_m_speed = spd + adjustment;
  int right_m_speed = spd - adjustment;
  
  // ensure speed saturation
  left_m_speed = (left_m_speed > 255) ? 255 : left_m_speed;
  left_m_speed = (left_m_speed < -255) ? -255 : left_m_speed;
  
  right_m_speed = (right_m_speed > 255) ? 255 : right_m_speed;
  right_m_speed = (right_m_speed < -255) ? -255 : right_m_speed;
  /*
  Serial.print(left_m_speed);
  Serial.print(" ");
  Serial.println(right_m_speed);
  */
  left_m_speed *= 0.8;
  right_m_speed *= 0.8;
  
  motorControl(left_m_speed, right_m_speed);
}

// setup -----------------------------------------------------------------------------------------------

OptSenData lsen(l_sen);
SonarSenData fsen(SensorTrig, SensorEcho);

void setup()
{
  Serial.begin(9600);
  
  pinMode(ServoControl, OUTPUT);
  sensor.attach(ServoControl);
  
  pinMode(l_sen, INPUT);
  pinMode(SensorTrig, OUTPUT);
  pinMode(SensorEcho, INPUT);
  
  pinMode(LeftMotorControl, OUTPUT);
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(RightMotorControl, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  
  SensorControl(SensorLookForward);
}

unsigned long distance = 0;
#define LeftTarget 1150
#define LeftHole 1500
#define ForwardWall 550

void loop()
{
  fsen.setReading();
  distance = fsen.getReading();
  
  //Serial.println(distance);
  
  if(distance < ForwardWall)
  {
    t_right();
    delay(400);
    go();
  }
  
  //delay(500);
  lsen.setReading();
  distance = lsen.getReading();
  //Serial.println(distance);
  INTERVAL_EXEC(1, 200, execute_control, distance);
}
