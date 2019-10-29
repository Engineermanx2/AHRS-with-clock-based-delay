#include <Arduino.h>
#include <SPI.h>
#include <Encoder.h>
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB
Encoder CUI(10 ,11);
float angle = 0.0f;
MPU9250_DMP imu;
float yawraw;
short angularyaw, yaw;
float gyroz;
const int mag = 100, numbofvariables = 4, datasize = (numbofvariables * 2)+2, samplerate = 50, periodms = (1000/samplerate);
const int offset = 1000;
bool wait;
unsigned long nowTime, reportTime;
byte data[datasize];
short datapack[numbofvariables]; //int an array to put all the data you want to send
float newpos;
void setup(){
  SerialUSB.begin(115200);
  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS){
    while (1){
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  imu.setSensors(INV_Z_GYRO); //enable all sensors needed
  imu.setGyroFSR(2000);       //how many degrees persecond opt(+-250, 500, 1000, 2000)dps
  imu.setAccelFSR(2);         //set high and low bound of g's to measure opt(+-2, 4, 8, 16)g
  imu.setLPF(42);             //set digital lowpass filter
  imu.setSampleRate(samplerate);//set accelerometer and gyro sample rate opt(4-1k)Hz
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |DMP_FEATURE_GYRO_CAL, samplerate ); 
               // Enable 6-axis quat  // Use gyro calibration // Set DMP FIFO rate in hz
  datapack[0] = offset;
  datapack[1] = mag;
  //the first two nubmers sent are used to manipulate the rest of the data so that we can use an unsinged 14 bit number and get negative numbers and percision
}

void communicate(void){   //data conversion and transmition over serial
  //byte data[datasize];//create byte array size of data
  //convert number to bytes with 'sync bits' as 8th bit
  
  //create first packet of data with a 1 in the 8th position
  data[0] = datapack[0]>>7;
  data[0] = data[0]|0x80;//8th bit to 1

  //create the rest of the data with a 8th bit as zero
  data[1] = datapack[0]&0xFF;
  data[1] = data[1]&0x7F;//8thbit to 0

  //loop so this can be used for multiple data sizes
  for(int i = 1; i <= numbofvariables; i++){
    int n = 2*i;
    data[n] = datapack[i]>>7;
    data[n] = data[n]&0x7F;//8thbit to 0
    data[n+1] = datapack[i]&0x7F;
    data[n+1] = data[n+1]&0x7F;//8thbit to 0
  }

  //create checksum but summing all the data
  unsigned short checksum = 0;
  for( int i = 0; i <= 1+((numbofvariables-1)*2); i++){
    checksum = checksum + data[i];
  }

  //fill end of data array with checksum
  data[datasize-2] = checksum>>7;
  data[datasize-2] = data[datasize-2]&0x7F;//8thbit to 0
  data[datasize-1] = checksum&0x7F;
  data[datasize-1] = data[datasize-1]&0x7F;//8thbit to 0
  //Serial.write(data, datasize);//send array with a buffer size of datasize
  //moved
}

void updateIMUvalues(void){  //updates Euler angles
  if ( imu.fifoAvailable()>= 4){
    // Use dmpUpdateFifo to update  values
    while(imu.fifoAvailable()>0){
      if ( imu.dmpUpdateFifo() == INV_SUCCESS){
      
      // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
      // are all updated.
      // Quaternion values are, by default, stored in Q30 long
      // format. calcQuat turns them into a float between -1 and 1
      imu.calcQuat(imu.qw);
      imu.calcQuat(imu.qx);
      imu.calcQuat(imu.qy);
      imu.calcQuat(imu.qz);
      imu.computeEulerAngles();
      yawraw = imu.yaw;
      imu.update(UPDATE_GYRO);
      gyroz = imu.calcGyro(imu.gz);
      }
    }
  }
}

void loop(){
  updateIMUvalues();
  newpos = (CUI.read()/4.0f);
  angle = ((newpos/(2048.0f))*360.0f);
  yaw = yawraw;
  //angularyaw = (gyroz) + offset;
  datapack[2] = yaw;
  datapack[3] = angle + offset /*angularyaw*/;
  communicate();
  wait = 1;
  while(wait == 1){
    nowTime = millis();
    if(nowTime >= reportTime){
      SerialUSB.write(data, datasize);
      //SerialUSB.println();
      reportTime = nowTime + periodms;
      wait = 0;
    }
  }
}