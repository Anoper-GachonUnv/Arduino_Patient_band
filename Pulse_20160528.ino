#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>

SoftwareSerial BTSerial(2, 3); 

/* time */
#define SENDING_INTERVAL 1000
#define SENSOR_READ_INTERVAL 50
#define ACCEL_BUFFER_COUNT 125
unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

/* Data buffer */
byte aAccelBuffer[ACCEL_BUFFER_COUNT];
int iAccelIndex = 2;
#define BUFFSIZE 100

/* MPU-6050 sensor */
#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I 0x75 // R
#define MPU6050_I2C_ADDRESS 0x68

typedef union accel_t_gyro_union {
	struct {
		uint8_t x_accel_h;
		uint8_t x_accel_l;
		uint8_t y_accel_h;
		uint8_t y_accel_l;
		uint8_t z_accel_h;
		uint8_t z_accel_l;
		uint8_t t_h;
		uint8_t t_l;
		uint8_t x_gyro_h;
		uint8_t x_gyro_l;
		uint8_t y_gyro_h;
		uint8_t y_gyro_l;
		uint8_t z_gyro_h;
		uint8_t z_gyro_l;
	} reg;

	struct {
		int x_accel;
		int y_accel;
		int z_accel;
		int temperature;
		int x_gyro;
		int y_gyro;
		int z_gyro;
	} value;
};

// Beat Rate Sensor variable
int pulsePin = 0;
int interval;
volatile int sensorVal;
volatile int sample[BUFFSIZE];
volatile int ind_sample;
volatile int BPM[3];
volatile boolean flag=false;

void setup(){
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  ind_sample=0;
  interval = 0;
  BPM[0] = BPM[1] = BPM[2] = 0;
  for(int i=0; i<BUFFSIZE; i++)
    sample[i]=0;
    
  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  initBuffer();
  Serial.println("Hello");
}

void loop() {
  sensorVal = analogRead(pulsePin);
  if(sensorVal > 250 && sensorVal < 800){
    sample[ind_sample++] = sensorVal;
    if( ind_sample == BUFFSIZE ){
      ind_sample = 0;
      sensorVal = 0;
      for(int i=0; i< BUFFSIZE-1; i++)
        sensorVal += (sample[i+1] - sample[i]);
      
      if( sensorVal < 0 && flag == true){
         BPM[0] = BPM[1];
         BPM[1] = BPM[2];
         BPM[2] = 60000/interval;
         Serial.print("BPM : ");
         Serial.println((BPM[0]+BPM[1]+BPM[2])/3);
         interval=0;
         flag = false;
      }
      else if( sensorVal > 0 )
        flag = true;
    }
  }
  interval+=2;
  delay(2);
  
 // Read from Gyro sensor
   curSensoredTime = millis();
  if(curSensoredTime - prevSensoredTime > SENSOR_READ_INTERVAL) {
    readFromSensor();  // Read from sensor
    prevSensoredTime = curSensoredTime;
  }
  // Send Gyro sensor buffer data through Bluetooth
  if(iAccelIndex >= ACCEL_BUFFER_COUNT - 3) {
      sendToRemote();
      initBuffer();
      //Serial.println("------------- Send 20 accel data to remote");
    }
   while (BTSerial.available()){ // if BT sends something
    byte data = BTSerial.read();
    Serial.write(data); // write it to serial(serial monitor)
  }
}
void sendToRemote() {
  // Write gabage bytes
  BTSerial.write( "accel" );
  // Write accel data
  BTSerial.write( (char*)aAccelBuffer );
  // Flush buffer
  //BTSerial.flush();
}

/**************************************************
 * Read data from sensor and save it
 **************************************************/
void readFromSensor() {
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  if(error != 0) {
    //Serial.print(F("Read accel, temp and gyro, error = "));
    //Serial.println(error,DEC);
  }
  
  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like x_accel_l does no
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
  
  // Print the raw acceleration values
  //Serial.print(F("accel x,y,z: "));
  //Serial.print(accel_t_gyro.value.x_accel, DEC);
  //Serial.print(F(", "));
  //Serial.print(accel_t_gyro.value.y_accel, DEC);
  //Serial.print(F(", "));
  //Serial.print(accel_t_gyro.value.z_accel, DEC);
  //Serial.print(F(", at "));
  //Serial.print(iAccelIndex);
  //Serial.println(F(""));
  
  if(iAccelIndex < ACCEL_BUFFER_COUNT && iAccelIndex > 1) {
    int tempX = accel_t_gyro.value.x_accel;
    int tempY = accel_t_gyro.value.y_accel;
    int tempZ = accel_t_gyro.value.z_accel;
    /*
    // Check min, max value
    if(tempX > 16380) tempX = 16380;
    if(tempY > 16380) tempY = 16380;
    if(tempZ > 16380) tempZ = 16380;
    
    if(tempX < -16380) tempX = -16380;
    if(tempY < -16380) tempY = -16380;
    if(tempZ < -16380) tempZ = -16380;
    
    // We dont use negative value
    tempX += 16380;
    tempY += 16380;
    tempZ += 16380;
    */
    char temp = (char)(tempX >> 8);
    if(temp == 0x00)
      temp = 0x7f;
    aAccelBuffer[iAccelIndex] = temp;
    iAccelIndex++;
    temp = (char)(tempX);
    if(temp == 0x00)
      temp = 0x01;
    aAccelBuffer[iAccelIndex] = temp;
    iAccelIndex++;
    
    temp = (char)(tempY >> 8);
    if(temp == 0x00)
      temp = 0x7f;
    aAccelBuffer[iAccelIndex] = temp;
    iAccelIndex++;
    temp = (char)(tempY);
    if(temp == 0x00)
      temp = 0x01;
    aAccelBuffer[iAccelIndex] = temp;
    iAccelIndex++;
    
    temp = (char)(tempZ >> 8);
    if(temp == 0x00)
      temp = 0x7f;
    aAccelBuffer[iAccelIndex] = temp;
    iAccelIndex++;
    temp = (char)(tempZ);
    if(temp == 0x00)
      temp = 0x01;
    aAccelBuffer[iAccelIndex] = temp;
    iAccelIndex++;
  }
  
  // The temperature sensor is -40 to +85 degrees Celsius.
  // It is a signed integer.
  // According to the datasheet:
  // 340 per degrees Celsius, -512 at 35 degrees.
  // At 0 degrees: -512 - (340 * 35) = -12412
  //Serial.print(F("temperature: "));
  dT = ( (double) accel_t_gyro.value.temperature + 12412.0) / 340.0;
  //Serial.print(dT, 3);
  //Serial.print(F(" degrees Celsius"));
  //Serial.println(F(""));

  // Print the raw gyro values. "gyro x,y,z : "
  //Serial.print(F("gyro x,y,z : "));
  //Serial.print(accel_t_gyro.value.x_gyro, DEC);
  //Serial.print(F(", "));
  //Serial.print(accel_t_gyro.value.y_gyro, DEC);
  //Serial.print(F(", "));
  //Serial.print(accel_t_gyro.value.z_gyro, DEC);
  //Serial.println(F(""));
}

/**************************************************
 * MPU-6050 Sensor read/write
 **************************************************/
int MPU6050_read(int start, uint8_t *buffer, int size)
{
	int i, n, error;
	
	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	
	n = Wire.write(start);
	if (n != 1)
		return (-10);
	
	n = Wire.endTransmission(false); // hold the I2C-bus
	if (n != 0)
		return (n);
	
	// Third parameter is true: relase I2C-bus after data is read.
	Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
	i = 0;
	while(Wire.available() && i<size)
	{
		buffer[i++]=Wire.read();
	}
	if ( i != size)
		return (-11);
	return (0); // return : no error
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
	int n, error;
	
	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	
	n = Wire.write(start); // write the start address
	if (n != 1)
		return (-20);
		
	n = Wire.write(pData, size); // write data bytes
	if (n != size)
		return (-21);
		
	error = Wire.endTransmission(true); // release the I2C-bus
	if (error != 0)
		return (error);
	return (0); // return : no error
}

int MPU6050_write_reg(int reg, uint8_t data)
{
	int error;
	error = MPU6050_write(reg, &data, 1);
	return (error);
}


/**************************************************
 * Utilities
 **************************************************/
void initBuffer() {
  iAccelIndex = 2;
  for(int i=iAccelIndex; i<ACCEL_BUFFER_COUNT; i++) {
    aAccelBuffer[i] = 0x00;
  }
  aAccelBuffer[0] = 0xfe;
  aAccelBuffer[1] = 0xfd;
  aAccelBuffer[122] = 0xfd;
  aAccelBuffer[123] = 0xfe;
  aAccelBuffer[124] = 0x00;
}
