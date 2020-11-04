//
// Created by ysbf on 8/21/20.
//

#ifndef BNO055DRIVER_SRC_MPU9250_MPU9250_REG_H
#define BNO055DRIVER_SRC_MPU9250_MPU9250_REG_H

#define MPU9250_SELF_TEST_X_GYRO        0x00
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02

#define MPU9250_SELF_TEST_X_ACCEL       0x0D
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F

#define MPU9250_XG_OFFSET_H             0x13
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x15
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPLRT_DIV              0x19
#define MPU9250_CONFIG                  0x1A
#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_ACCEL_CONFIG            0x1C
#define MPU9250_ACCEL_CONFIG2           0x1D
#define MPU9250_LP_ACCEL_ODR            0x1E
#define MPU9250_WOM_THR                 0x1F

#define MPU9250_FIFO_EN                 0x23
#define MPU9250_I2C_MST_CTRL            0x24
#define MPU9250_I2C_SLV0_ADDR           0x25
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36
#define MPU9250_INT_PIN_CFG             0x37
#define MPU9250_INT_ENABLE              0x38

#define MPU9250_INT_STATUS              0x3A
#define MPU9250_ACCEL_XOUT_H            0x3B
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60

#define MPU9250_I2C_SLV0_DO             0x63
#define MPU9250_I2C_SLV1_DO             0x64
#define MPU9250_I2C_SLV2_DO             0x65
#define MPU9250_I2C_SLV3_DO             0x66
#define MPU9250_I2C_MST_DELAY_CTRL      0x67
#define MPU9250_SIGNAL_PATH_RESET       0x68
#define MPU9250_MOT_DETECT_CTRL         0x69
#define MPU9250_USER_CTRL               0x6A
#define MPU9250_PWR_MGMT_1              0x6B
#define MPU9250_PWR_MGMT_2              0x6C

#define MPU9250_FIFO_COUNTH             0x72
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFO_R_W                0x74
#define MPU9250_WHO_AM_I                0x75
#define MPU9250_XA_OFFSET_H             0x77
#define MPU9250_XA_OFFSET_L             0x78

#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B

#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E
//
#define MPU9250_I2C_READ 0x80

//Magnetometer register maps
#define MPU9250_AK8963_WIA                 0x00
#define MPU9250_AK8963_INFO                0x01
#define MPU9250_AK8963_ST1                 0x02
#define MPU9250_AK8963_XOUT_L              0x03
#define MPU9250_AK8963_XOUT_H              0x04
#define MPU9250_AK8963_YOUT_L              0x05
#define MPU9250_AK8963_YOUT_H              0x06
#define MPU9250_AK8963_ZOUT_L              0x07
#define MPU9250_AK8963_ZOUT_H              0x08
#define MPU9250_AK8963_ST2                 0x09
#define MPU9250_AK8963_CNTL                0x0A
#define MPU9250_AK8963_CNTL2               0x0B
#define MPU9250_AK8963_RSV                 0x0B //DO NOT ACCESS <MPU9250_AK8963_CNTL2>
#define MPU9250_AK8963_ASTC                0x0C
#define MPU9250_AK8963_TS1                 0x0D //DO NOT ACCESS
#define MPU9250_AK8963_TS2                 0x0E //DO NOT ACCESS
#define MPU9250_AK8963_I2CDIS              0x0F
#define MPU9250_AK8963_ASAX                0x10
#define MPU9250_AK8963_ASAY                0x11
#define MPU9250_AK8963_ASAZ                0x12

#define MPU9250_AK8963_I2C_ADDR 0x0C
#define MPU9250_AK8963_POWER_DOWN 0x10
#define MPU9250_AK8963_FUSE_ROM_ACCESS 0x1F
#define MPU9250_AK8963_SINGLE_MEASUREMENT 0x11
#define MPU9250_AK8963_CONTINUOUS_MEASUREMENT 0x16 //MODE 2
#define MPU9250_AK8963_DATA_READY      (0x01)
#define MPU9250_AK8963_DATA_OVERRUN    (0x02)
#define MPU9250_AK8963_OVERFLOW        (0x80)
#define MPU9250_AK8963_DATA_ERROR      (0x40)
#define MPU9250_AK8963_CNTL2_SRST 0x01

//
#define MPU9250_I2C_SLV4_EN 0x80
#define MPU9250_I2C_SLV4_DONE 0x40
#define MPU9250_I2C_SLV4_NACK 0x10
//
#define MPU9250_I2C_IF_DIS (0x10)
#define MPU9250_I2C_MST_EN (0x20)
#define MPU9250_FIFO_RST (0x04)
#define MPU9250_FIFO_ENABLE (0x40)
//
#define MPU9250_RESET 0x80
#define MPU9250_CLOCK_MASK 0xF8
#define MPU9250_CLOCK_INTERNAL 0x00
#define MPU9250_CLOCK_PLL 0x01
#define MPU9250_CLOCK_PLLGYROZ 0x03
#define MPU9250_FS_SEL_MASK 0xE7
#define MPU9250_SLEEP_MASK 0x40
//
#define MPU9250_XYZ_GYRO 0xC7
#define MPU9250_XYZ_ACCEL 0xF8
//
#define MPU9250_RAW_RDY_EN (0x01)
#define MPU9250_RAW_DATA_RDY_INT (0x01)
#define MPU9250_FIFO_OVERFLOW (0x10)
//
#define MPU9250_INT_ANYRD_2CLEAR (0x10)
#define MPU9250_LATCH_INT_EN (0x20)
//
#define MPU9250_MAX_FIFO (1024)
#define MPU9250_FIFO_SIZE_1024  (0x40)
#define MPU9250_FIFO_SIZE_2048  (0x80)
#define MPU9250_FIFO_SIZE_4096  (0xC0)

#define MPU9250_TEMP_OUT (0x80)
#define MPU9250_GYRO_XOUT (0x40)
#define MPU9250_GYRO_YOUT (0x20)
#define MPU9250_GYRO_ZOUT (0x10)
#define MPU9250_ACCEL (0x08)

//
#define SMPLRT_DIV 0
#define MPU9250_SPIx_ADDR 0x00
//////////////////////////////////////////////////////////////////////////
//
enum MPU9250_GYRO_DLPF {
  MPU9250_GYRO_DLPF_250HZ = 0,
  MPU9250_GYRO_DLPF_184HZ,
  MPU9250_GYRO_DLPF_92HZ,
  MPU9250_GYRO_DLPF_41HZ,
  MPU9250_GYRO_DLPF_20HZ,
  MPU9250_GYRO_DLPF_10HZ,
  MPU9250_GYRO_DLPF_5HZ,
  MPU9250_GYRO_DLPF_3600HZ,
  NUM_GYRO_DLPF
};

enum MPU9250_GYRO_FSR {
  MPU9250_FSR_250DPS = 0,
  MPU9250_FSR_500DPS,
  MPU9250_FSR_1000DPS,
  MPU9250_FSR_2000DPS,
  MPU9250_NUM_GYRO_FSR
};

enum MPU9250_ACCEL_DLPF {
  MPU9250_ACCEL_DLPF_460HZ = 0,
  MPU9250_ACCEL_DLPF_184HZ,
  MPU9250_ACCEL_DLPF_92HZ,
  MPU9250_ACCEL_DLPF_41HZ,
  MPU9250_ACCEL_DLPF_20HZ,
  MPU9250_ACCEL_DLPF_10HZ,
  MPU9250_ACCEL_DLPF_5HZ,
  MPU9250_ACCEL_DLPF_460HZ2,
  MPU9250_NUM_ACCEL_DLPF
};

enum MPU9250_ACCEL_FSR {
  MPU9250_FSR_2G = 0,
  MPU9250_FSR_4G,
  MPU9250_FSR_8G,
  MPU9250_FSR_16G,
  MPU9250_NUM_ACCEL_FSR
};

enum MPU9250_CLK {
  MPU9250_CLK_INTERNAL = 0,
  MPU9250_CLK_PLL,
  MPU9250_NUM_CLK
};

static const unsigned short mpu_9250_st_tb[256] = {
    2620,2646,2672,2699,2726,2753,2781,2808, //7
    2837,2865,2894,2923,2952,2981,3011,3041, //15
    3072,3102,3133,3165,3196,3228,3261,3293, //23
    3326,3359,3393,3427,3461,3496,3531,3566, //31
    3602,3638,3674,3711,3748,3786,3823,3862, //39
    3900,3939,3979,4019,4059,4099,4140,4182, //47
    4224,4266,4308,4352,4395,4439,4483,4528, //55
    4574,4619,4665,4712,4759,4807,4855,4903, //63
    4953,5002,5052,5103,5154,5205,5257,5310, //71
    5363,5417,5471,5525,5581,5636,5693,5750, //79
    5807,5865,5924,5983,6043,6104,6165,6226, //87
    6289,6351,6415,6479,6544,6609,6675,6742, //95
    6810,6878,6946,7016,7086,7157,7229,7301, //103
    7374,7448,7522,7597,7673,7750,7828,7906, //111
    7985,8065,8145,8227,8309,8392,8476,8561, //119
    8647,8733,8820,8909,8998,9088,9178,9270,
    9363,9457,9551,9647,9743,9841,9939,10038,
    10139,10240,10343,10446,10550,10656,10763,10870,
    10979,11089,11200,11312,11425,11539,11654,11771,
    11889,12008,12128,12249,12371,12495,12620,12746,
    12874,13002,13132,13264,13396,13530,13666,13802,
    13940,14080,14221,14363,14506,14652,14798,14946,
    15096,15247,15399,15553,15709,15866,16024,16184,
    16346,16510,16675,16842,17010,17180,17352,17526,
    17701,17878,18057,18237,18420,18604,18790,18978,
    19167,19359,19553,19748,19946,20145,20347,20550,
    20756,20963,21173,21385,21598,21814,22033,22253,
    22475,22700,22927,23156,23388,23622,23858,24097,
    24338,24581,24827,25075,25326,25579,25835,26093,
    26354,26618,26884,27153,27424,27699,27976,28255,
    28538,28823,29112,29403,29697,29994,30294,30597,
    30903,31212,31524,31839,32157,32479,32804,33132
};
struct test_s {
  unsigned long gyro_sens;
  unsigned long accel_sens;
  unsigned char reg_rate_div;
  unsigned char reg_lpf;
  unsigned char reg_gyro_fsr;
  unsigned char reg_accel_fsr;
  unsigned short wait_ms;
  unsigned char packet_thresh;
  float min_dps;
  float max_dps;
  float max_gyro_var;
  float min_g;
  float max_g;
  float max_accel_var;

  float max_g_offset;
  unsigned short sample_wait_ms;

};
const struct test_s test = {
    .gyro_sens      = 32768/250,
    .accel_sens     = 32768/2,  //FSR = +-2G = 16384 LSB/G
    .reg_rate_div   = 0,    /* 1kHz. */
    .reg_lpf        = 2,    /* 92Hz low pass filter*/
    .reg_gyro_fsr   = 0,    /* 250dps. */
    .reg_accel_fsr  = 0x0,  /* Accel FSR setting = 2g. */
    .wait_ms        = 200,   //200ms stabilization time
    .packet_thresh  = 200,    /* 200 samples */
    .min_dps        = 20.f,  //20 dps for Gyro Criteria C
    .max_dps        = 60.f, //Must exceed 60 dps threshold for Gyro Criteria B
    .max_gyro_var   = .5f, //Must exceed +50% variation for Gyro Criteria A
    .min_g          = .225f, //Accel must exceed Min 225 mg for Criteria B
    .max_g          = .675f, //Accel cannot exceed Max 675 mg for Criteria B
    .max_accel_var  = .5f,  //Accel must be within 50% variation for Criteria A
    .max_g_offset   = .5f,   //500 mg for Accel Criteria C
    .sample_wait_ms = 10    //10ms sample time wait
};
#endif //BNO055DRIVER_SRC_MPU9250_MPU9250_REG_H
