#include <Arduino.h>
#include "Wire.h"

//#define debug
#define serial_debug  Serial1

// LIS2DH12
#define ADDR                        0x19

#define LIS2DH12_DUMMY_REG          0x0F //always responds with 0x33
#define LIS2DH12_CTRL_REG1          0x20
  #define LIS2DH12_ODR_25HZ         0x30
  #define LIS2DH12_LP_EN            0x08
  #define LIS2DH12_Z_EN             0x04
  #define LIS2DH12_Y_EN             0x02
  #define LIS2DH12_X_EN             0x01

#define LIS2DH12_CTRL_REG4          0x23
  #define LIS2DH12_BDU_EN           0x80
  #define LIS2DH12_2G               0x00
  #define LIS2DH12_HR_LP            0x00

#define LIS2DH12_CTRL_REG6          0x25
  #define LIS2DH12_I2IA1_EN         0x40
  #define LIS2DH12_I2IA2_EN         0x20
  #define LIS2DH12_INT2_ACT_EN      0x08
  #define LIS2DH12_INT_POL_ACT_LOW  0x02
  #define LIS2DH12_INT_POL_ACT_HIGH 0x00

#define LIS2DH12_INT2_CFG           0x34
#define LIS2DH12_INT2_SRC           0x35
#define LIS2DH12_INT2_THS           0x36
#define LIS2DH12_INT2_DUR           0x37

#define LIS2DH12_ACT_THS            0x3E
#define LIS2DH12_ACT_DUR            0x3F

#define LIS2DH12_OUT_X_LSB          0x28

class LIS2DH12{
  public:
    LIS2DH12();
    boolean begin();
    void read();
    int16_t acc_x_value;
    int16_t acc_y_value;
    int16_t acc_z_value;
};
