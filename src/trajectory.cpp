//number motors 1 - 4, started with the front left and going counter clockwise
//number ultrasonic 1 - 3, started with front, then left, then right

#include <Arduino.h>

#define motor1Pin1 0
#define motor1Pin2 1
#define motor2Pin1 2
#define motor2Pin2 3
#define motor3Pin1 4
#define motor3Pin2 5
#define motor4Pin1 6
#define motor4Pin2 7

#define apin1 8
#define bpin1 9
#define apin2 10
#define bpin2 11
#define apin3 20
#define bpin3 21
#define apin4 22
#define bpin4 23

#define trigPin2 15
#define echoPin2 14

#define trigPin3 16
#define echoPin3 17

#define trigPin1 18
#define echoPin1 19

#define Kp 30
#define Ki 0.0001
#define Kd 0

#define MAX_ACCELERATION 1200
#define MAX_VELOCITY 500

#define MOVE_ERROR 10

#define ROTATE_FACTOR 2.5

typedef struct motionPeriods_s
{
    double x_goal;
    double x0;
    double v0;
    double v_max;
    double a_max;
    double T1;
    double T2;
    double T3;
    double a_acc;
    double a_dec;
    double v;
} MotionPeriod;

typedef struct setPoint_s
{
    double t;
    double x;
    double v;
    double a;
} SetPoint;

typedef struct motor_s {
  bool reverse;
  int encoderA,encoderB;
  int motorA,motorB;
  long encoderTicks;
  long encoderTicksVelocity;
  unsigned long encoderLastTick;
  unsigned long encoderPeriod;
  long setpoint;
  long startTime;
  MotionPeriod periods;
  SetPoint motionSetpoint;
  int dir;
  long p[2],i,d;
} Motor;

typedef struct ultrasonic_s {
  int trig;
  int echo;
}Ultrasonic;

typedef enum movementType_e {
  MOVE_X,
  MOVE_Y,
  MOVE_DIAGONALLEFT,
  MOVE_DIAGONALRIGHT,
  SPIN_PLZ
  
}MovementType;

typedef struct Movement_s {
  MovementType type;
  int delta;
  int delayms;
  bool ultrasonicCheck;
}Movement;

Motor *motor1;
Motor *motor2;
Motor *motor3;
Motor *motor4;
Ultrasonic *sensor1;
Ultrasonic *sensor2;
Ultrasonic *sensor3;
Movement *moves;

// Equation to determine position
double position1(double t, double xi, double vi, double a)
{
    return (xi + vi * t + 0.5 * a * pow(t, 2));
}

double velocity(double t, double vi, double a)
{
    return (vi + a * t);
}

SetPoint *create_empty_setpoint()
{
    SetPoint *setpoint = (SetPoint*)malloc(sizeof(SetPoint));
    setpoint->t = 0; 
    setpoint->x = 0;
    setpoint->v = 0;
    setpoint->a = 0;
    return setpoint;
}

void compute_period(MotionPeriod *periods, double x_goal, double x0, double v0, double v_max, double a_max)
{ 
    double x_stop = fabs((-1 * v0 + sqrt(fabs(pow(v0, 2) - 4 * (-0.5 * a_max) * x0)) / (2 * (-0.5 * a_max))));
    
    int d;
    if (x_goal > x_stop) {
      d = 1;
    }else {
      d = -1;
    }
    int xGoalSign = 0;
    if (x_goal < 0) {
      xGoalSign = -1;
    }else if (x_goal > 0) {
      xGoalSign = 1;
    }
    int x0Sign = 0;
    if (x0 < 0) {
      x0Sign = -1;
    }else if (x0 > 0) {
      x0Sign = 1;
    }
    if (x0Sign == -1 && xGoalSign == 0) {
      d *=-1;
    }
    if (x0Sign == xGoalSign) {
      if (fabs(x0) > fabs(x_goal)) {
        Serial.println("Changing direction");
        d *= -1;
      }
    }
    double v = d * v_max;
    double a_acc = d * a_max;
    double a_dec = -1 * d * a_max;
    double T1 = fabs((v - v0) / a_acc);
    double T3 = fabs(v / a_dec);

    double X1 = position1(T1, 0, v0, a_acc);
    double X3 = position1(T3, 0, v, a_dec);

    double T2 = (x_goal - x0 - X1 - X3) / v;

    if (T2 < 0)
    {
        T2 = 0;
        v = d * sqrt(fabs(d * a_max * (x_goal - x0) + 0.5 * pow(v0, 2)));
        T1 = fabs((v - v0) / a_acc);
        T3 = fabs(v / a_dec);
    }
    periods->x_goal = x_goal;
    periods->x0 = x0;
    periods->v0 = v0;
    periods->v_max = v_max;
    periods->a_max = a_max;
    periods->T1 = T1;
    periods->T2 = T2;
    periods->T3 = T3;
    periods->a_acc = a_acc;
    periods->a_dec = a_dec;
    periods->v = v;
}

void compute_setpoint(SetPoint *setpoint, double t, MotionPeriod *periods)
{
    double x1 = position1(periods->T1, periods->x0, periods->v0, periods->a_acc);
    double v2 = periods->v;
    double x2 = position1(periods->T2, x1, v2, 0);
    if (t <= 0)
    {
        setpoint->x = position1(0, periods->x0, periods->v0, 0);
        setpoint->v = velocity(0, periods->v0, 0);
        setpoint->a = 0;
    }
    else if (t < periods->T1)
    {
        setpoint->x = position1(t, periods->x0, periods->v0, periods->a_acc);
        setpoint->v = velocity(t, periods->v0, periods->a_acc);
        setpoint->a = periods->a_acc;
    }
    else if (t < periods->T1 + periods->T2)
    {
        setpoint->x = position1(t - periods->T1, x1, v2, 0);
        setpoint->v = velocity(t - periods->T1, v2, 0);
        setpoint->a = 0;
    }
    else if (t < periods->T1 + periods->T2 + periods->T3)
    {
        setpoint->x = position1(t - (periods->T1 + periods->T2), x2, v2, periods->a_dec);
        setpoint->v = velocity(t - (periods->T1 + periods->T2), v2, periods->a_dec);
        setpoint->a = periods->a_dec;
    }
    else
    {
        setpoint->x = periods->x_goal;
        setpoint->v = 0;
        setpoint->a = 0;
    }
}

double ultrasonic_read1(){
  //function to read the ultrasonic sensors
  int delayTime = 10; //can change the time between sensor reads

  digitalWrite(trigPin1, LOW);
  delayMicroseconds(10); 
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  return pulseIn(echoPin1, HIGH,100)/58.0;
}

double ultrasonic_read2() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  return pulseIn(echoPin2, HIGH,100)/58.0;
}

double ultrasonic_read3() {
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  return pulseIn(echoPin3, HIGH,100)/58.0;  
}

void moveCW(Motor *motor, int motorSpeed) {
  analogWrite(motor->motorA,0);
  analogWrite(motor->motorB,motorSpeed); 
}

void moveCCW(Motor *motor, int motorSpeed) {
  analogWrite(motor->motorB,0);
  analogWrite(motor->motorA,motorSpeed); 
}

double mm_to_ticks(int mm) {
  //140ticks/rev / 80mm/rev * mm
  return mm * (140.0/(3.14159*80.0));
}

double ticks_to_mm(int ticks) {
  return ((double)ticks)/(140.0/(3.14159*80.0));
}

void inc_a1() {
  int increment = 1;
  if (motor1->reverse) {
    increment = -1;
  }
  if (digitalRead(motor1->encoderB)) { //B->A
    motor1->encoderTicks = motor1->encoderTicks - increment;
  }else {
    motor1->encoderTicks = motor1->encoderTicks + increment;
  }
  unsigned long currentus = micros();
  motor1->encoderPeriod = currentus - motor1->encoderLastTick;
  motor1->encoderLastTick = currentus;
}

void inc_a2() {
  int increment = 1;
  if (motor2->reverse) {
    increment = -1;
  }
  if (digitalRead(motor2->encoderB)) { //B->A
    motor2->encoderTicks = motor2->encoderTicks - increment;
  }else {
    motor2->encoderTicks = motor2->encoderTicks + increment;
  }
  unsigned long currentus = micros();
  motor2->encoderPeriod = currentus - motor2->encoderLastTick;
  motor2->encoderLastTick = currentus;
}

void inc_a3() {
  int increment = 1;
  if (motor3->reverse) {
    increment = -1;
  }
  if (digitalRead(motor3->encoderB)) { //B->A
    motor3->encoderTicks = motor3->encoderTicks - increment;
  }else {
    motor3->encoderTicks = motor3->encoderTicks + increment;
  }
  unsigned long currentus = micros();
  motor3->encoderPeriod = currentus - motor3->encoderLastTick;
  motor3->encoderLastTick = currentus;
}

void inc_a4() {
  int increment = 1;
  if (motor4->reverse) {
    increment = -1;
  }
  if (digitalRead(motor4->encoderB)) { //B->A
    motor4->encoderTicks = motor4->encoderTicks - increment;
  }else {
    motor4->encoderTicks = motor4->encoderTicks + increment;
  }
  unsigned long currentus = micros();
  motor4->encoderPeriod = currentus - motor4->encoderLastTick;
  motor4->encoderLastTick = currentus;
}

Motor *create_motor(int motorA,int motorB,int encoderA,int encoderB,bool reverse, void (*func)()) {
  Motor *motor = (Motor*)malloc(sizeof(Motor));
  motor->encoderTicks = 0;
  motor->encoderLastTick = 0;
  motor->encoderPeriod = 0;
  motor->startTime = 0;
  motor->setpoint = 0;
  motor->reverse = reverse;
  motor->p[0]=0;
  motor->p[1]=0;
  motor->i=0;
  motor->d=0;
  motor->motorA=motorA;
  motor->motorB=motorB;
  motor->encoderA=encoderA;
  motor->encoderB=encoderB;
  motor->periods.x_goal=0;
  motor->periods.x0=0;
  motor->periods.v0=0;
  motor->periods.v_max=0;
  motor->periods.a_max=0;
  motor->periods.T1=0;
  motor->periods.T2=0;
  motor->periods.T3=0;
  motor->periods.a_acc=0;
  motor->periods.a_dec=0;
  motor->periods.v=0;
  motor->motionSetpoint.t=0;
  motor->motionSetpoint.x=0;
  motor->motionSetpoint.v=0;
  motor->motionSetpoint.a=0;
  motor->dir = 0;
  pinMode(motor->encoderA,INPUT);
  pinMode(motor->encoderB,INPUT);
  pinMode(motor->motorA,OUTPUT);
  pinMode(motor->motorB,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(motor->encoderA), func, RISING);
  return motor;
}

void update_setpoint(Motor *motor, int setpoint) {
  double currentmm = ticks_to_mm(motor->encoderTicks);
   motor->setpoint = setpoint;
  compute_period(&(motor->periods), motor->setpoint, currentmm, 0, MAX_VELOCITY, MAX_ACCELERATION);
/*  Serial.print("Currentmm:");
  Serial.println(currentmm);
  Serial.print("t1:");
  Serial.println(motor->periods.T1);
  Serial.print("t2:");
  Serial.println(motor->periods.T2);
  Serial.print("t3:");
  Serial.println(motor->periods.T3);
  Serial.print("Currentmm:");
  Serial.println(motor->periods.T3);*/
  motor->startTime = millis();
}

void move_setpoint_pid_static(Motor *motor) {
  //PID Variables
  long currentTime = millis();
  long deltaTime = currentTime - motor->startTime;
  double velocity = 0;
  compute_setpoint(&(motor->motionSetpoint), deltaTime/1000.0, &(motor->periods));
  int limitedSetpoint = (int)mm_to_ticks(motor->motionSetpoint.x);

  //Update PID values
  motor->p[1]= motor->p[0];
  motor->p[0] = limitedSetpoint - motor->encoderTicks;
  motor->i += motor->p[0];
  motor->d = (motor->p[0]-motor->p[1]);

  //Update motor voltage from setpoint
  int voltage = motor->p[0]*Kp + motor->i*Ki + motor->d*Kd;
  if (motor->reverse) {
    voltage = voltage * -1;
  }
  if (voltage > 0) { //Move CW
    moveCW(motor,fabs(voltage));
  }else { //Move CCW
    moveCCW(motor,fabs(voltage));
  }
}

int numberMoves = 3;

long lastms;
long lastmsPID;
long currentms;
long nextMove;
int currentMove;
int lastMove;
void setup() {

  lastms=0;
  lastmsPID=0;
  currentms=0;
  nextMove = 0;
  currentMove=0;
  lastMove=-1;

  motor1 = create_motor(motor1Pin1,motor1Pin2,apin1,bpin1,false,inc_a1);
  motor2 = create_motor(motor2Pin1,motor2Pin2,apin2,bpin2,false,inc_a2);
  motor3 = create_motor(motor3Pin1,motor3Pin2,apin3,bpin3,true,inc_a3);
  motor4 = create_motor(motor4Pin1,motor4Pin2,apin4,bpin4,true,inc_a4);

  moves = (Movement*)malloc(sizeof(Movement)*numberMoves);
    int std_delay = 500;
  int index = 0;

  moves[index].type = MOVE_X;
  moves[index].delta = 1025;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;

  moves[index].type = MOVE_X;
  moves[index].delta = 10;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = true;
  index++;

  moves[index].type = MOVE_Y;
  moves[index].delta = 10;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = true;
  index++;

  moves[index].type = MOVE_Y;
  moves[index].delta = 370;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_X;
  moves[index].delta = 300;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_DIAGONALRIGHT; 
  moves[index].delta = 300;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_X;
  moves[index].delta = 300;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = SPIN_PLZ; //180 DEG TURN
  moves[index].delta = 180;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_Y;
  moves[index].delta = -950;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_X;
  moves[index].delta = 1200;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_Y;
  moves[index].delta = -650;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_X;
  moves[index].delta = 375;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_Y;
  moves[index].delta = 300;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_DIAGONALLEFT;
  moves[index].delta = 275;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_Y;
  moves[index].delta = 1300;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;
  
  moves[index].type = MOVE_DIAGONALLEFT;
  moves[index].delta = 250;
  moves[index].delayms = std_delay;
  moves[index].ultrasonicCheck = false;
  index++;

  Serial.begin(9600);
}

void loop() {
    
  //update position
  move_setpoint_pid_static(motor1);
  move_setpoint_pid_static(motor2);
  move_setpoint_pid_static(motor3);
  move_setpoint_pid_static(motor4);
  //Print information
  currentms = millis();
  if ((currentms-lastms) > 1000 ) { //Debugging information
    Serial.print("CurrentMove:");
    Serial.println(currentMove);
    Serial.print("Setpoint(ticks):");
    Serial.print(mm_to_ticks(motor1->setpoint));
    Serial.print(",");
    Serial.print(mm_to_ticks(motor2->setpoint));
    Serial.print(",");
    Serial.print(mm_to_ticks(motor3->setpoint));
    Serial.print(",");
    Serial.println(mm_to_ticks(motor4->setpoint));
    Serial.print("Position(ticks):");
    Serial.print(motor1->encoderTicks);
    Serial.print(",");
    Serial.print(motor2->encoderTicks);
    Serial.print(",");
    Serial.print(motor3->encoderTicks);
    Serial.print(",");
    Serial.println(motor4->encoderTicks);
    Serial.print("p:");
    Serial.print(motor1->p[0]);
    Serial.print(",");
    Serial.print(motor2->p[0]);
    Serial.print(",");
    Serial.print(motor3->p[0]);
    Serial.print(",");
    Serial.println(motor4->p[0]);
    Serial.print("i:");
    Serial.println(motor1->i);
    Serial.print("d:");
    Serial.println(motor1->d);
    Serial.print("Voltage:");
    Serial.println(motor1->p[0]*Kp + motor1->i*Ki + motor1->d*Kd);
    Serial.print(",Ultrasonic:");
    //Serial.print(ultrasonic_read1());
    Serial.print(",");
    //Serial.print(ultrasonic_read2());
    Serial.print(",");
    //Serial.println(ultrasonic_read3());
    Serial.print("Move time:");
    Serial.print((currentms-motor1->startTime)/1000000.0);
    Serial.print(",");
    Serial.print((currentms-motor2->startTime)/1000000.0);
    Serial.print(",");
    Serial.print((currentms-motor3->startTime)/1000000.0);
    Serial.print(",");
    Serial.println((currentms-motor4->startTime)/1000000.0);
    Serial.print("Pos:");
    Serial.print(motor1->motionSetpoint.x);
    Serial.print(",");
    Serial.print(motor2->motionSetpoint.x);
    Serial.print(",");
    Serial.print(motor3->motionSetpoint.x);
    Serial.print(",");
    Serial.println(motor4->motionSetpoint.x);
    Serial.print("Vel:");
    Serial.print(motor1->motionSetpoint.v);
    Serial.print(",");
    Serial.print(motor2->motionSetpoint.v);
    Serial.print(",");
    Serial.print(motor3->motionSetpoint.v);
    Serial.print(",");
    Serial.println(motor4->motionSetpoint.v);
    Serial.print("Acc:");
    Serial.print(motor1->motionSetpoint.a);
    Serial.print(",");
    Serial.print(motor2->motionSetpoint.a);
    Serial.print(",");
    Serial.print(motor3->motionSetpoint.a);
    Serial.print(",");
    Serial.println(motor4->motionSetpoint.a);
    Serial.print("T1:");
    Serial.print(motor1->periods.T1);
    Serial.print(",");
    Serial.print(motor2->periods.T1);
    Serial.print(",");
    Serial.print(motor3->periods.T1);
    Serial.print(",");
    Serial.println(motor4->periods.T1);
    Serial.print("T2:");
    Serial.print(motor1->periods.T2);
    Serial.print(",");
    Serial.print(motor2->periods.T2);
    Serial.print(",");
    Serial.print(motor3->periods.T2);
    Serial.print(",");
    Serial.println(motor4->periods.T2);
    Serial.print("T3:");
    Serial.print(motor1->periods.T3);
    Serial.print(",");
    Serial.print(motor2->periods.T3);
    Serial.print(",");
    Serial.print(motor3->periods.T3);
    Serial.print(",");
    Serial.println(motor4->periods.T3);
    lastms=currentms;
  }   

  if (nextMove < currentms) {
    if (currentMove < numberMoves) {
      if (currentMove != lastMove) {
        lastMove = currentMove; 
        switch(moves[currentMove].type) {
          case MOVE_X:
          if (moves[currentMove].ultrasonicCheck) {
            int distance = ultrasonic_read2();
            Serial.print("Sensor Reading:");
            Serial.println(distance);
            moves[currentMove].delta -= distance;
            Serial.print("Compensating X by:");
            Serial.println(moves[currentMove].delta);
          }
            update_setpoint(motor1, motor1->setpoint+moves[currentMove].delta);
            update_setpoint(motor2, motor2->setpoint+moves[currentMove].delta);
            update_setpoint(motor3, motor3->setpoint+moves[currentMove].delta);
            update_setpoint(motor4, motor4->setpoint+moves[currentMove].delta);
          break;
          case MOVE_Y:
          if (moves[currentMove].ultrasonicCheck) {
            if (moves[currentMove].delta > 0) {
              int distance = ultrasonic_read3();
              moves[currentMove].delta -= distance;
              Serial.print("Compensating Y+ by:");
              Serial.println(moves[currentMove].delta);
            }else {
              int distance = ultrasonic_read1();
              moves[currentMove].delta += distance;
              Serial.print("Compensating Y- by:");
              Serial.println(moves[currentMove].delta);
            }
          }
            update_setpoint(motor1, motor1->setpoint-moves[currentMove].delta);
            update_setpoint(motor2, motor2->setpoint+moves[currentMove].delta);
            update_setpoint(motor3, motor3->setpoint-moves[currentMove].delta);
            update_setpoint(motor4, motor4->setpoint+moves[currentMove].delta);
          break;
          case MOVE_DIAGONALRIGHT:
            update_setpoint(motor1, motor1->setpoint+moves[currentMove].delta);
            update_setpoint(motor2, motor2->setpoint);
            update_setpoint(motor3, motor3->setpoint+moves[currentMove].delta);
            update_setpoint(motor4, motor4->setpoint);
          break;
          case MOVE_DIAGONALLEFT:
            update_setpoint(motor1, motor1->setpoint);
            update_setpoint(motor2, motor2->setpoint+moves[currentMove].delta);
            update_setpoint(motor3, motor3->setpoint);
            update_setpoint(motor4, motor4->setpoint+moves[currentMove].delta);
          break;
          case SPIN_PLZ:
          if (moves[currentMove].ultrasonicCheck) { //This requires previous move that reads ultrasonic value and a previous move that moves 
            if (moves[currentMove-1].type == MOVE_X) { //Use left or right sensor
              if (moves[currentMove].delta > 0) { //Checking right side sensor
                double distance = ultrasonic_read3();
                double delta = moves[currentMove-2].delta - distance;
                moves[currentMove].delta = atan(delta/moves[currentMove-1].delta)*180/3.14159265;
              }else { // Check left side sensor
                double distance = ultrasonic_read1();
                double delta = moves[currentMove-2].delta + distance;
                moves[currentMove].delta = atan(delta/moves[currentMove-1].delta)*180/3.14159265;
              }
            }else if (moves[currentMove-1].type == MOVE_Y) { //Using front sensor
                double distance = ultrasonic_read2();
                double delta = moves[currentMove-2].delta + distance;
                moves[currentMove].delta = atan(delta/moves[currentMove-1].delta)*180/3.14159265;
            }
          }
            update_setpoint(motor1, motor1->setpoint+moves[currentMove].delta*ROTATE_FACTOR);
            update_setpoint(motor2, motor2->setpoint+moves[currentMove].delta*ROTATE_FACTOR);
            update_setpoint(motor3, motor3->setpoint-moves[currentMove].delta*ROTATE_FACTOR);
            update_setpoint(motor4, motor4->setpoint-moves[currentMove].delta*ROTATE_FACTOR);
          break;
        }
      }
      int motor1AbsError = abs(motor1->setpoint - ticks_to_mm(motor1->encoderTicks));
      int motor2AbsError = abs(motor2->setpoint - ticks_to_mm(motor2->encoderTicks));
      int motor3AbsError = abs(motor3->setpoint - ticks_to_mm(motor3->encoderTicks));
      int motor4AbsError = abs(motor4->setpoint - ticks_to_mm(motor4->encoderTicks));
      if (motor1AbsError < MOVE_ERROR && motor2AbsError < MOVE_ERROR && motor3AbsError < MOVE_ERROR && motor4AbsError < MOVE_ERROR) {
        // Move complete
        //Serial.println("Move complete");
        nextMove = millis() + moves[currentMove].delayms;
        currentMove++;
      }
    }
  }
  delay(10);
}