
#ifndef _PINS_ARDUINO_
#define _PINS_ARDUINO_

#define V822A_PACKAGE

#ifdef V822A_PACKAGE
//
//            +--+---+--+
//            |  |ANT|  |
//            |  +---+  |
//   RF_GND --+         +-- TXD0
// BOOT_SEL --+         +-- GPIO16
//   GPIO10 --+         +-- GPIO20
//    GPIO6 --+         +-- GPIO5
//   GPIO12 --+         +-- GPIO22
//   GPIO13 --+         +-- GPIO28
//   GPIO29 --+         +-- GPIO30
//    GPIO4 --+         +-- GPIO31
//      GND --+         +-- GPIO14
//    VCC33 --+         +-- GPIO3
//    VCC5V --+         +-- GPIO2
//  Battery --+         +-- GPIO1
//            |  +---+  +-- ADC_IN2
//            |  |USB|  |
//            +--+---+--+
//
#define  ADC_IN2                  100
#define  GPIO0_LED                0
#define  GPIO1_RXD2               1
#define  GPIO2_TXD2               2
#define  GPIO3_P1PPS              3
#define  GPIO4_SCL                4
#define  GPIO5_SDA                5
#define  GPIO6_WHEEL_TIC_MEAS     6
#define  GPIO10_TRIG             10
#define  GPIO12_THERM_IN1        12
#define  GPIO13_THERM_OUT1       13
#define  GPIO14_THERM_IN2        14
#define  GPIO16_CHEAP_XTALO      16
#define  GPIO20_PWM0             20
#define  GPIO22_SPISL_MCS1       22
#define  GPIO28_SPIMS_CSN        28
#define  GPIO29_SPIMS_SCK        29
#define  GPIO30_SPIMS_MOSI       30
#define  GPIO31_SPIMS_MISO       31
#endif

#ifdef V822_PACKAGE
//
//           +--+---+--+
//           |  |ANT|  |
//           |  +---+  |
//  RF_GND --+         +-- TXD0
//    P3V0 --+         +-- GPIO20
//  GPIO16 --+         +-- GPIO5
//  GPIO15 --+         +-- GPIO22
//  GPIO12 --+         +-- GPIO8
//  GPIO13 --+         +-- GPIO28
//  GPIO29 --+         +-- GPIO30
//   GPIO4 --+         +-- GPIO31
//     GND --+         +-- GPIO14
//   VCC33 --+         +-- GPIO3
//   VCC5V --+         +-- GPIO2
// Battery --+         +-- GPIO1
//           |  +---+  |
//           |  |USB|  |
//           +--+---+--+
//
#define  ADC_IN2                 100
#define  GPIO0_LED               0
#define  GPIO1_RXD2              1
#define  GPIO2_TXD2              2
#define  GPIO3_P1PPS             3
#define  GPIO4_SCL               4
#define  GPIO5_SDA               5
#define  GPIO8                   8
#define  GPIO12_THERM_IN1       12
#define  GPIO13_THERM_OUT1      13
#define  GPIO14_THERM_IN2       14
#define  GPIO15_THERM_OUT2      15
#define  GPIO16_CHEAP_XTALO     16
#define  GPIO20_PWM0            20
#define  GPIO22_SPISL_MCS1      22
#define  GPIO28_SPIMS_CSN       28
#define  GPIO29_SPIMS_SCK       29
#define  GPIO30_SPIMS_MOSI      30
#define  GPIO31_SPIMS_MISO      31
#endif

#define  A0  ADC_IN2

#endif
