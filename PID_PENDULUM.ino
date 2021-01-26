/*
  +PID_PENDULUM DEMO by MyhubEDU Sri Lanka.
  +Special thanks for Jeff Rowberg for providing the awesome I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP.
  +Big Applause to the arduino community in Sri Lanka & in the World.
  
*/

void setup() {
 initMPU();
}

void loop() {
 scanMPU();   
}



double prevErr = 0;
double totErr = 0;
int kp = 5;
int kd = 10;
float ki = 0.01;


void pidAlgo(double sY) {
  
  double sp = -1 * map(analogRead(A0), 0, 1023, 0, 90); //Get the set point from the potentiometer
  double error = sp - sY; //obtain the error from substracting

  double Pterm = error * kp;
  double Dterm = (error - prevErr) * kd;
  double Iterm = (double)ki * totErr;

  double pidVal =  Pterm + Dterm + Iterm ; 

  int mspeed = (int)pidVal;
  drM(-mspeed);

  prevErr = error;
  totErr += error;
  Serial.print(sp);
  Serial.print(",");
  Serial.print(sY);
  Serial.print(",");
  Serial.println(pidVal);
}

int En = 5;
int Pp = 6;
int Np = 7;
void drM(int m1) {
  if (m1 >= 255) {
    m1 = 255;
  }
  if (m1 <= -255) {
    m1 = -255;
  }
  if (m1 < 0) {
    m1 = -1 * m1;
    digitalWrite(Pp, LOW); digitalWrite(Np, HIGH);
    analogWrite(En, m1);
  } else {
    digitalWrite(Pp, HIGH); digitalWrite(Np, LOW);
    analogWrite(En, m1);
  }
}
