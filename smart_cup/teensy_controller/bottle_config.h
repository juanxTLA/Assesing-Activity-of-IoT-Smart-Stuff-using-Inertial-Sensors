#include <i2c_t3.h>

#define USB_Serial Serial
#define ESP_Serial Serial1

//Capacitance Definitions
#define CAP_S0 22
#define CAP_S1 23
#define A_CAP 1
#define B_CAP 1

// Accelerometer MMA8653FC definitions
#define I2C_ADDRESS_ACC 0x1D // address of Acc
#define OUT_X_MSB 0x01  // X Component MSB address 
#define OUT_X_LSB 0x02  // X Component LSB address 
#define OUT_Y_MSB 0x03  // Y Component MSB address 
#define OUT_Y_LSB 0x04  // Y Component LSB address 
#define OUT_Z_MSB 0x05  // Z Component MSB address  
#define OUT_Z_LSB 0x06  // Z Component LSB address 
#define F_READ   0x2A // Data format is limited to single byte
#define DATA_STATUS 0x00 // Data Status of XYZ components
#define XYZ_DATA_CFG 0x0E //configuration register
#define SYSMODE 0x0B // System Mode/ Read register
#define OFF_X 0x2F // Offset address for X Component
#define OFF_Y 0x30 // Offset address for Y Component
#define OFF_Z 0x31 // Offset address for Z Component
#define CTRL_REG1 0x2A // Control Register to activate the Speed of measurements
#define ctrl_reg_address 0x00 // to understand why 0x00 and not 0x01, look at the data-sheet p.19 or on the comments of the sub. 
                              //This is valid only becaus we use auto-increment
#define SYNC_DELAY 1000 // millisecond
#define ANGLE_THRESHOLD 5 // vertival angle threshold

#define ACC_SETUP_DELAY 2000 //Accelerometer Initialization wait

void setupI2C(){
    Wire.begin(I2C_MASTER, I2C_ADDRESS_ACC , I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
    Wire.beginTransmission(I2C_ADDRESS_ACC);
    Wire.send(1);  // register to read3
    Wire.endTransmission(I2C_NOSTOP, 1000);
}

struct MMA8653FC {
    void writeRegister(unsigned char r, unsigned char v){
        Wire.beginTransmission(I2C_ADDRESS_ACC);
        Wire.send(r);
        Wire.send(v);
        Wire.endTransmission();
    }

    unsigned char readRegister(unsigned char r){

        Wire.beginTransmission(I2C_ADDRESS_ACC);
        Wire.send(r);
        Wire.endTransmission();

        Wire.requestFrom(I2C_ADDRESS_ACC, 1);

        while(Wire.available() == 0) {

        }

        return Wire.receive();
    }

    void accInit(){
        writeRegister(CTRL_REG1, 0x00);
        delay(50);
        writeRegister(XYZ_DATA_CFG, 0x00);
        writeRegister(CTRL_REG1, 0x01);
    }
};