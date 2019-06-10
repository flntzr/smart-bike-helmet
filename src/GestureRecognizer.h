// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _GESTURE_RECOGNIZER_H_
#define _GESTURE_RECOGNIZER_H_
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
// #include <SPI.h>

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "Queue.h"
#include <math.h>

#define MPU_INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define SMOOTHING_SAMPLE_SIZE 12 // The amount of 'roll' values that are remembered for smoothing
#define WARMUP_LENGTH 100 // the amount of initial measurements that are discarded to the sensor needing to adjust
#define ACTIVITY_THRESHOLD 0.022 // The minimal absolute delta for a movement to be considered active.
#define GESTURE_THRESHOLD 0.5 // The threshold for the likelihood to actually register it as a gesture.
#define MIN_GESTURE_TIME_MS 1000 // the minimum time for a gesture to be detected. Blocks other gesture to be detected.

enum { NONE, ROLL_LEFT, ROLL_RIGHT};
struct Gesture {
    uint8_t type;
    unsigned long begin;
};

class GestureRecognizer {
    public:

        GestureRecognizer();
        /**
         * Initializes the MPU and internal class variables. 
         * Returns true if initialization was successful, otherwise false.
         */
        bool init();
        /**
         * Reads sensor data and updates the gestures accordingly. 
         * Returns true if the gesture changed, otherwise false.
         */
        bool processSensorData();
        /**
         * Clears all recognized gestures. 
         */
        void clearGestures();
        void enterSleep();
        void wakeUp();
        Gesture getActiveGesture();

    private:
        // class default I2C address is 0x68
        // specific I2C addresses may be passed as a parameter here
        // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
        // AD0 high = 0x69
        MPU6050 mpu;

        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        Queue<float> queue = Queue<float>(SMOOTHING_SAMPLE_SIZE);
        uint8_t warmupCountdown = WARMUP_LENGTH;
        Gesture activeGesture = { type: NONE };
        Gesture lastRecognizedGesture = { type: NONE };

        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        bool hasOverflown();
        void handleOverflow();
        bool updateGesture();
        float getAverageVal(Queue<float> * q);
        bool isActive(float);
        float getActivityLikelihood(Queue<float> * q, bool * positive);
        void recognizeGesture(Queue<float> * q, Gesture &gesture);

};
#endif