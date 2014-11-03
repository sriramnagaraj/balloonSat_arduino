balloonSat_arduino
==================

This is the arduino code that collects analog data from 5-6 sensors. It involves I2C communication, Serial communication and writing data values to SD card

#include<Wire.h>
#include<SD.h>
#include<SPI.h>

#define HMC5883L_ADDR 0x1E
//gy521
const int MPU=0x68;  // I2C address of the MPU-6050
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

long id;
int tempPin         =  0;
int ultrasonic_trig = 40;
int ultrasonic_echo = 41;
int mq2             =  1;
int mq135           =  2;
int aosong          =  3;
int sd_cs_pin       = 42;
int sd_pow_pin      = 43;
long tempC;
long distance;
long duration;
int iSensorValue    =  0; 
byte bySensorVal    =  0;
char cMsg[124];
int aosong_value;
int v1;

bool haveHMC5883L = false;

bool detectHMC5883L ()
{
  // read identification registers
  Wire.beginTransmission(HMC5883L_ADDR); //open communication with HMC5883
  Wire.write(10); //select Identification register A
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 3);
  if(3 == Wire.available()) {
    char a = Wire.read();
    char b = Wire.read();
    char c = Wire.read();
    if(a == 'H' && b == '4' && c == '3')
      return true;
  }

  return false;
}


void setup() 
{
  Serial.begin(9600);
  //sd card
  Serial.println("initialising card");
  pinMode(sd_pow_pin, OUTPUT);
  pinMode(sd_cs_pin,OUTPUT);
  pinMode(sd_pow_pin, HIGH);
  if (!SD.begin(sd_cs_pin))
  {
    Serial.println("card failed");
    return;
  }
  Serial.println("card ready");
  
  //SD CARD testing
  String dataString = " aerostatico";
  File datafile = SD.open("values.txt", FILE_WRITE);
  if(datafile)
  {
    datafile.println(dataString);
    datafile.close();
    Serial.println(dataString);
  }
  else
  {
    Serial.println("file failed");
  }
  
  //sd card data logging
  File logFile = SD.open("log.csv",FILE_WRITE);
  if(logFile)
  {
    logFile.println("1,2,3,4,5,6,7,8,9");
    logFile.close();
  }
  else
  {
    Serial.println("couldnt access log file");
  }//SD CARD testing
  dataString = " aerostatico";
  datafile = SD.open("values.txt", FILE_WRITE);
  if(datafile)
  {
    datafile.println(dataString);
    datafile.close();
    Serial.println(dataString);
  }
  else
  {
    Serial.println("file failed");
  }
  
  //sd card data logging
   logFile = SD.open("log.csv",FILE_WRITE);
  if(logFile)
  {
    logFile.println("1,2,3,4,5,6,7,8,9");
    logFile.close();
  }
  else
  {
    Serial.println("couldnt access log file");
  }
  
  
  pinMode(ultrasonic_trig, OUTPUT);
  pinMode(ultrasonic_echo, INPUT);
  
  // gy271
   Serial.begin(9600);
  Serial.println("GY271 TEST");
  Wire.begin();
  // lower I2C clock http://www.gammon.com.au/forum/?id=10896
  TWBR = 78;  // 25 kHz 
  TWSR |= _BV (TWPS0);  // change prescaler 
  
  //gy521
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}
 void loop()
{ 
     
  
  
  // temperature sensor
tempC = analogRead(tempPin);           //read the value from the sensor
tempC = (5.0 * tempC * 100.0)/1024.0;  //convert the analog data to temperature
Serial.print((byte)tempC);             //send the data to the computer

//hcsc04 ultrasonic sensor
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(ultrasonic_trig, LOW); 
 delayMicroseconds(2); 
 digitalWrite(ultrasonic_trig, HIGH);
 delayMicroseconds(10); 
 digitalWrite(ultrasonic_trig, LOW);
 duration = pulseIn(ultrasonic_echo, HIGH);
 //Calculate the distance (in cm) based on the speed of sound.
 distance = duration/58.2;
 Serial.println("distance");
 delay(50);
 
 //mq2
  // Read input value on A0 and map it from 0 to 100
  iSensorValue = analogRead(mq2);
  bySensorVal = map(iSensorValue, 0, 1023, 0, 100);
  
  // Display input value and mapped value
  sprintf(cMsg, "MQ-2 Sensor Value : %d (%d)", iSensorValue, bySensorVal);

  // Check for high value
  if (bySensorVal > 60) {
    Serial.print(cMsg);
    Serial.println(F(" *** DISTURBANCE IN THE FORCE! ***"));
  }
  else {
    Serial.println(cMsg);
  }
 
  // Loop 10 times per second
  
 // aosong sensor
aosong_value=analogRead(aosong);
Serial.println(aosong_value);

// mq135
v1=analogRead(mq135);
Serial.println(v1);

// gy271
bool detect = detectHMC5883L();

  if(!haveHMC5883L) 
  {
    if(detect) 
    {
      haveHMC5883L = true;
      Serial.println("We have HMC5883L, moving on");
      // Put the HMC5883 IC into the correct operating mode
      Wire.beginTransmission(HMC5883L_ADDR); //open communication with HMC5883
      Wire.write(0x02); //select mode register
      Wire.write(0x00); //continuous measurement mode
      Wire.endTransmission();
    }
    else
    {  
      Serial.println("No HMC5883L detected!");
      delay(2000);
      return;
    }
  }
  else
  {
    if(!detect) {
      haveHMC5883L = false;
      Serial.println("Lost connection to HMC5883L!");
      delay(2000);
      return;
    }
  }
  
  int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();

 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(HMC5883L_ADDR, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z);
  
  //gy521
   Wire.beginTransmission(MPU);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
      AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      Serial.print("AcX = "); Serial.print(AcX);
      Serial.print(" | AcY = "); Serial.print(AcY);
      Serial.print(" | AcZ = "); Serial.print(AcZ);
      Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
      Serial.print(" | GyX = "); Serial.print(GyX);
      Serial.print(" | GyY = "); Serial.print(GyY);
      Serial.print(" | GyZ = "); Serial.println(GyZ);
  
  // gy 65

// sd card data logging
 String dataString = String(id)+","+String(tempC)+","+String(distance)+","+String(v1)+","+String(cMsg)+","+String(aosong_value)+","+String(x)+","+String(y)+","+String(z)+","+String(AcX)+","+String(AcY)+","+String(AcZ)+","+String(GyX)+","+String(GyY)+","+String(GyZ);
 File logFile= SD.open("LOG.csv",FILE_WRITE);
 if (logFile)
 {
   logFile.println(dataString);
   logFile.close();
   Serial.println(dataString);
 }
else
{
  Serial.println("couldnt open log file");
}

}




  
