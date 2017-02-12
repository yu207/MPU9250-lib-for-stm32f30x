
#include "stm32f30x.h"
#include "stm32f30x_i2c.h"
#include <stdio.h>
#include "MPU9250.h"

////////////////////////////////////////STRUCTURE DEFINE///////////////////////////////////////////////

#define FLAG_TIMEOUT ((int)0x4000)
#define LONG_TIMEOUT ((int)0x8000)
#define IMUAddress						0xd0
#define IMU_I2C								I2C1	

uint16_t Address_mpu6050 = 0xd0;
uint16_t MPU9250_mag_addr = 0x18;

////////////////////////////////////////FUNCTION DEFINE/////////////////////////////////////////////
uint8_t MPU9250_RegWrite(int addr_i2c,int addr_reg, char v);
int MPU9250_DataWrite(int address, char* data, int length, uint8_t repeated);
int I2C_Write(int address, char *data, int length, int stop);
int I2C_ByteWrite(int data);

uint8_t MPU9250_RegRead(int addr_i2c,int addr_reg, char *v);
int MPU9250_DataRead(int address, char* data, int length, uint8_t repeated);
int I2C_DataRead(int address, char *data, int length, int stop);
int I2C_ByteRead(int last);

////////////////////////////////////////Sensor fusion/////////////////////////////////////////////
float invSqrt(float x);
void I2C_Stop(void);
//void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
extern void delay_ms(__IO uint32_t nCount);

void I2C_Congiguration(void)
{

  GPIO_InitTypeDef  GPIO_InitStructure; 
  I2C_InitTypeDef  I2C_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB , ENABLE); //GPIO clock enable
    
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9; ////last version PB9&PB8 as SDA&SCL which is I2C1 channel
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_4);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; //0x0000
  I2C_InitStructure.I2C_OwnAddress1 = IMUAddress; //boot I2C1_MPU9250 = 0xd0 which means Address is low
	I2C1->CR2 |= (0xd0 << 1); // 7-bit address format allow
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // I2C_Ack_Enable = 0x0400
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // I2C_AcknowledgedAddress_7bit = 0x4000 , RESPONSE 1 BYTE
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_Timing = 400000; // i2c frequency 400000

  I2C_Init(I2C1, &I2C_InitStructure);
	
  I2C_Cmd(I2C1, ENABLE);

	I2C_AcknowledgeConfig(I2C1, ENABLE);    	
	
}


void MPU9250_Init(void)
{

	I2C_Congiguration();	
	delay_ms(100);
	MPU9250_RegWrite(Address_mpu6050,PWR_MGMT_1,0x80);
	delay_ms(10);
	MPU9250_RegWrite(Address_mpu6050,PWR_MGMT_1,0x03); //Select gyro Z axis for clock
	delay_ms(5);
	MPU9250_RegWrite(Address_mpu6050,SMPLRT_DIV, 19); //Sample rate=50Hz Fsample=1Khz/(19+1) = 50Hz set 0x00 achieve 8KHz
	delay_ms(5);
 	MPU9250_RegWrite(Address_mpu6050,CONFIG, 0x04); //FS & DLPF FS=2000º/s, DLPF = 20Hz LPF set 0x00 achieve Accel&Gyro max bandwidth 
	delay_ms(5);
	MPU9250_RegWrite(Address_mpu6050,GYRO_CONFIG, 0x18); //BITS_FS_2000DPS
	delay_ms(5);
	MPU9250_RegWrite(Address_mpu6050,ACCEL_CONFIG, 0x09);//Accel scale +/-4g (Full Scale 8192 LSB/mg) or 4096 LSB bits/g
	delay_ms(5);
	MPU9250_RegWrite(Address_mpu6050, INT_PIN_CFG, 0x22);  
  delay_ms(5);  
  MPU9250_RegWrite(Address_mpu6050, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay_ms(5);
 // MPU9250_RegWrite(Address_mpu6050, MPUREG_INT_PIN_CFG, 0x02); //INT_PIN_CFG (Bypass Enable Configuration Mode), Enable the access to the auxiliary I2C bus (and the magnetometer)
	delay_ms(1000);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MPU9250_Read_Acc(float *x, float *y, float *z)
{
    signed int x_a, y_a, z_a;
    char c_data[6];  
		char data = ACCEL_XOUT_H;
		MPU9250_DataWrite(Address_mpu6050, &data, 1, 0); //ACCEL_XOUT_H reg starting address
		MPU9250_DataRead(Address_mpu6050, c_data, 6, 0);   
    x_a =  c_data[0] << 8;
    x_a += c_data[1];
    y_a =  c_data[2] << 8;
    y_a += c_data[3];
    z_a =  c_data[4] << 8;
    z_a += c_data[5];   
    //two's complement  
    if(x_a >= 0x8000)
        x_a -= 0xFFFF;
    if(y_a >= 0x8000)
        y_a -= 0xFFFF;
    if(z_a >= 0x8000)
        z_a -= 0xFFFF;      
    *x = (float)((float)x_a / ACCEL_MGLSB) * 9.801f;  //return m/s^2
    *y = (float)((float)y_a / ACCEL_MGLSB) * 9.801f;
    *z = (float)((float)z_a / ACCEL_MGLSB) * 9.801f;

}

void MPU9250_Read_Gyro(float *x, float *y, float *z)
{
    int x_g, y_g, z_g;
    char c_data[6];
		char data = GYRO_XOUT_H;
		MPU9250_DataWrite(Address_mpu6050, &data, 1, 0); //GYRO_XOUT_H reg starting address
		MPU9250_DataRead(Address_mpu6050, c_data, 6, 0);
    x_g =  c_data[0] << 8;
    x_g += c_data[1];
    y_g =  c_data[2] << 8;
    y_g += c_data[3];
    z_g =  c_data[4] << 8;
    z_g += c_data[5];
    //two's complement  
    if(x_g >= 0x8000)
        x_g -= 0xFFFF;
    if(y_g >= 0x8000)
        y_g -= 0xFFFF;
    if(z_g >= 0x8000)
        z_g -= 0xFFFF;
    //convert to measurement to degrees/sec, then to radians/sec
    *x = toRadians((float)((float)x_g / GYRO_LSB_DPS)); 
    *y = toRadians((float)((float)y_g / GYRO_LSB_DPS));
    *z = toRadians((float)((float)z_g / GYRO_LSB_DPS));
}


void MPU9250_Mag_Init(float *x, float *y, float *z) // initAK8963
{
	MPU9250_RegWrite(Address_mpu6050,USER_CTRL,0x20);//enable i2c master mode
	MPU9250_RegWrite(Address_mpu6050,I2C_MST_CTRL,0x0d);//i2c master clock speed of 400khz
	MPU9250_RegWrite(Address_mpu6050,I2C_SLV0_ADDR,AK8963_ADDRESS);//assign AK8963 to slave 0 of i2c master
	MPU9250_RegWrite(Address_mpu6050,I2C_SLV0_REG,AK8963_CNTL2);//Point save 0 register at AK8963's control 2 (soft reset) register
	MPU9250_RegWrite(Address_mpu6050,I2C_SLV0_DO,0x01);//send 01 via slave 0 to AK8963 to trigger a soft restart 
	MPU9250_RegWrite(Address_mpu6050,I2C_SLV0_CTRL,0x81);//enable 1-byte i2c reading
	
	MPU9250_RegWrite(Address_mpu6050,I2C_SLV0_REG,AK8963_CNTL1);//Point save 0 register at AK8963's control 1 (mode) register
	MPU9250_RegWrite(Address_mpu6050,I2C_SLV0_DO,0x12);//set AK8963 to be: 16bits output/continuous measurement mode 1 (output size and measurement mode can be changed))
	MPU9250_RegWrite(Address_mpu6050,I2C_SLV0_CTRL,0x81);//enable 1-byte i2c reading (reading size can be changed)
	//now, 1-byte of AK8963 reading will be available in EXT_SENS_DATA_00(in this case of SLV0)

	//TODO:
	//Not sure if still need to do the following for new configurations with I2C master mode:
	//Power down?
	//ENter Fuse ROM access mode?
	//read the x-,y-,and z-axis calibration values?
	//return x-axis sensitivity adjustment values?
	
/*	
    char sen_data[3]; // x/y/z sensitivity register data stored here
		char data = AK8963_ASAX;
	
  MPU9250_RegWrite(AK8963_ADDRESS, AK8963_CNTL1, 0x00); // Power down
  delay_ms(10);
  MPU9250_RegWrite(AK8963_ADDRESS, AK8963_CNTL1, 0x0F); // Enter Fuse ROM access mode
  delay_ms(10);
  MPU9250_DataWrite(AK8963_ADDRESS, &data, 1, 0); //GYRO_XOUT_H reg starting address
  MPU9250_DataRead(AK8963_ADDRESS, sen_data, 3, 0);  // Read the x-, y-, and z-axis calibration values
	  *x = (float)(sen_data[0] - 128)/256.0f + 1.0f; // Return x-axis sensitivity adjustment values 
    *y = (float)(sen_data[1] - 128)/256.0f + 1.0f;  
    *z = (float)(sen_data[2] - 128)/256.0f + 1.0f;  
*/
}

void MPU9250_Read_Mag(float *x, float *y, float *z)
{
  int x_m, y_m, z_m;
	char rawData[6];  // x/y/z  register data stored here
	char vector;
	char data = AK8963_XOUT_L;
	
  /*
	MPU9250_RegWrite(AK8963_ADDRESS, AK8963_CNTL1, 0x01); // toggle enable data read from magnetometer, no continuous read mode!
  delay_ms(10);
	
  // Only accept a new magnetometer data read if the data ready bit is set and 
  // if there are no sensor overflow or data read errors
	MPU9250_RegRead(AK8963_ADDRESS, AK8963_ST1, &vector);
	*/
	
	//TODO:
	//read data from EXT_SENS_DATA_00 (this data register corresponds to I2C_SLV0, which is bonded to AK8963 by the configuration in MPU9250_Mag_Init())
	
	
  if( vector & 0x01)  // wait for magnetometer data ready bit to be set
	{
	MPU9250_DataWrite(AK8963_ADDRESS, &data, 1, 0); //GYRO_XOUT_H reg starting address
  MPU9250_DataRead(AK8963_ADDRESS, rawData, 6, 0);  // Read the six raw data registers sequentially into data array
  x_m = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  y_m = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  z_m = ((int16_t)rawData[5] << 8) | rawData[4] ; 
		    //two's complement  
    if(x_m >= 0x8000)
        x_m -= 0xFFFF;
    if(y_m >= 0x8000)
        y_m -= 0xFFFF;
    if(z_m >= 0x8000)
        z_m -= 0xFFFF;
		//fliped the mx and my axes and fliped the mx axis polarity
       *y = (float)((float)x_m / MAG_LSB_BPUT);   //Magnetic LSB Bit/uT
       *x = -(float)((float)y_m / MAG_LSB_BPUT);
       *z = (float)((float)z_m / MAG_LSB_BPUT);
  }
}

//////////////////////////////////////////I2C READ/WRITE FUNCTIONALITY OPERATION///////////////////////////////////////////////////////////
//////////////////////////////////////////I2C write///////////////////////////////////////////
int I2C_ByteWrite(int data) //////////byte write with timeout check
{ 
    int timeout;
    // Wait until the previous byte is transmitted
    timeout = FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET) {
        if ((timeout--) == 0) {
            return 0;
        }
    }		
    I2C1->TXDR = (uint8_t)data;
    return 1;
}

int I2C_Write(int address, char *data, int length, int stop) //////////I2C write with event check
{
    int timeout;
    int count;
		char byte_n;
    /* update CR2 register */
    I2C1->CR2 = (I2C1->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
               | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SoftEnd_Mode | (uint32_t)I2C_Generate_Start_Write);

    for (count = 0; count < length; count++) {
				byte_n = data[count];
        I2C_ByteWrite(byte_n);
    }
    // Wait transfer complete
    timeout = FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET) {
        timeout--;
        if (timeout == 0) {
            return -1;
        }
    }
		I2C_ClearFlag(I2C1, I2C_FLAG_TC);

    // If not repeated start, send stop.
    if (stop) {
			  I2C1->CR2 |= I2C_CR2_STOP;
        /* Wait until STOPF flag is set */
        timeout = FLAG_TIMEOUT;
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET) {
            timeout--;
            if (timeout == 0) {
                return -1;
            }
        }
        /* Clear STOP Flag */
				I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
        
    }
    return count;
}

uint8_t MPU9250_RegWrite(int addr_i2c, int addr_reg, char v)
{
		char data[2];
    data[0] = addr_reg;
		data[1] = v; 
		I2C_ByteWrite(addr_i2c);
		I2C_ByteWrite(data[0]);
		I2C_ByteWrite(data[1]);
    return MPU9250_DataWrite(addr_i2c, data, 2, 0) == 0;
}

int MPU9250_DataWrite(int address, char* data, int length, uint8_t repeated) 
{
    int stop = (repeated) ? 0 : 1;
    int written = I2C_Write(address, data, length, stop);

    return length != written;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////I2C read//////////////////////////////////////////////////////
int I2C_ByteRead(int last)
{
    int timeout;
    // Wait until the byte is received
    timeout = FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET) {
        if ((timeout--) == 0) {
            return -1;
        }
    }
    return (int)I2C1->RXDR;
}

int I2C_DataRead(int address, char *data, int length, int stop)
{  
    int timeout;
    int count;
    int value;
    /* update CR2 register */
    I2C1->CR2 = (I2C1->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
               | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SoftEnd_Mode | (uint32_t)I2C_Generate_Start_Read);

    // Read all bytes
    for (count = 0; count < length; count++) {
        value = I2C_ByteRead(0);
        data[count] = (char)value;
    }
    // Wait transfer complete
    timeout = LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET) {
        timeout--;
        if (timeout == 0) {
            return -1;
        }
    }
    I2C_ClearFlag(I2C1, I2C_FLAG_TC);
    // If not repeated start, send stop.
    if (stop) {
        I2C1->CR2 |= I2C_CR2_STOP;
        /* Wait until STOPF flag is set */
        timeout = FLAG_TIMEOUT;
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET) {
            timeout--;
            if (timeout == 0) {
                return -1;
            }
        }
        /* Clear STOP Flag */
       I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
    }
    return length;
}

uint8_t MPU9250_RegRead(int addr_i2c,int addr_reg, char *v)
{
    char data = addr_reg; 
    uint8_t result = 0;

    if ((MPU9250_DataWrite(addr_i2c, &data, 1, 0) == 0) && (MPU9250_DataRead(addr_i2c, &data, 1, 0) == 0)){
        *v = data;
        result = 1;
    }
    return result;
}

int MPU9250_DataRead(int address, char* data, int length, uint8_t repeated) 
{
    int stop = (repeated) ? 0 : 1;
    int read =  I2C_DataRead(address, data, length, stop);
    return length != read;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_Stop(void)
{
    // Generate the STOP condition
    I2C1->CR2 |= I2C_CR2_STOP; 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
