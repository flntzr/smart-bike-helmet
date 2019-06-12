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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "avr/sleep.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// #include "queue.c";
#include "Queue.h";

#include <math.h>;

#include <SPI.h>
#include "LedMatrix.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


#define MPU_INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SMOOTHING_SAMPLE_SIZE 12 // The amount of 'roll' values that are remembered for smoothing
#define WARMUP_LENGTH 100 // the amount of initial measurements that are discarded due to the sensor needing to adjust
#define ACTIVITY_THRESHOLD 0.022 // The minimal absolute delta for a movement to be considered active.
#define GESTURE_THRESHOLD 0.5 // The threshold for the likelihood to actually register it as a gesture.
#define MIN_GESTURE_TIME_MS 1000 // the minimum time for a gesture to be detected. Blocks other gesture to be detected.

#define NUMBER_OF_LED_MATRICES 4 // the number of used LED matrices
#define LED_CS_PIN 10 // the PIN on the arduino connected to the 'CS' input of the MAX7219
#define LED_UPDATE_FREQ_MS 100 // the frequency for the LED matrix to update
#define LED_INTENSITY 15 // The LED matrix intensity. Range is 0-15

#define BLINK_START_TONE_FREC 2500
#define BLINK_END_TONE_FREC 3000
#define BLINK_TONE_DURATION 200

#define POWER_BUTTON_PIN 3
#define POWER_BUTTON_DELAY_MS 1000 // delay for each power button press to be processed. Necessary so no double-presses register.

const uint8_t NUMBER_OF_LED_COLUMNS = NUMBER_OF_LED_MATRICES * 8; 

byte FACES[3][8] = {
    // :)
    {
        0b00000000,
        0b00100100,
        0b00100100,
        0b00000000,
        0b00000000,
        0b01000010,
        0b00111100,
        0b00000000
    },
    // :|
    {
        0b00000000,
        0b00100100,
        0b00100100,
        0b00000000,
        0b00000000,
        0b00000000,
        0b01111110,
        0b00000000
    },
    // :(
    {
        0b00000000,
        0b00100100,
        0b00100100,
        0b00000000,
        0b00000000,
        0b00111100,
        0b01000010,
        0b00000000
    }
};

byte ARROWS[2][4][8] = {
    {
        {
            0b00000011,
            0b00000110,
            0b00001100,
            0b00011000,
            0b00011000,
            0b00001100,
            0b00000110,
            0b00000011
        },
        {
            0b00000111,
            0b00001110,
            0b00011100,
            0b00111000,
            0b00111000,
            0b00011100,
            0b00001110,
            0b00000111
        },
        {
            0b00001111,
            0b00011110,
            0b00111100,
            0b01111000,
            0b01111000,
            0b00111100,
            0b00011110,
            0b00001111
        },
        {
            0b00011111,
            0b00111110,
            0b01111100,
            0b11111000,
            0b11111000,
            0b01111100,
            0b00111110,
            0b00011111
        }
    }, {
        {
            0b11000000,
            0b01100000,
            0b00110000,
            0b00011000,
            0b00011000,
            0b00110000,
            0b01100000,
            0b11000000
        },
        {
            0b11100000,
            0b01110000,
            0b00111000,
            0b00011100,
            0b00011100,
            0b00111000,
            0b01110000,
            0b11100000
        },
        {
            0b11110000,
            0b01111000,
            0b00111100,
            0b00011110,
            0b00011110,
            0b00111100,
            0b01111000,
            0b11110000
        },
        {
            0b11111000,
            0b01111100,
            0b00111110,
            0b00011111,
            0b00011111,
            0b00111110,
            0b01111100,
            0b11111000
        }
    }
};

uint8_t activeArrow = 0;
LedMatrix ledMatrix = LedMatrix(NUMBER_OF_LED_MATRICES, LED_CS_PIN);
enum { NONE, ROLL_LEFT, ROLL_RIGHT};
struct Gesture {
    uint8_t type;
    unsigned long begin;
};
enum {SMILEY_FACE, NEUTRAL_FACE, FROWNY_FACE};

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Queue<float> queue = Queue<float>(SMOOTHING_SAMPLE_SIZE);
uint8_t warmupCountdown = WARMUP_LENGTH;
Gesture activeGesture = {
    type: NONE
};
Gesture lastRecognizedGesture = {
    type: NONE
};
unsigned long lastLedUpdate = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void clearMatrix() {
    ledMatrix.clear();
    ledMatrix.commit();
}

void renderFace(byte expression) {
    ledMatrix.clear();
    for (int i = 0; i < NUMBER_OF_LED_COLUMNS; i++) {
        ledMatrix.setColumn(i, FACES[expression][i%8]);
    }
    ledMatrix.commit(); // commit transfers the byte buffer to the displays
}

void clearGestures() {
    unsigned long now = millis();
    queue.clear();
    activeGesture.type = NONE;
    activeGesture.begin = now;
    lastRecognizedGesture.type = NONE;
    lastRecognizedGesture.begin = now;
}

void afterWakeUp() {
    renderFace(SMILEY_FACE);

    // detach Interrupt so it only triggers once
    detachInterrupt(digitalPinToInterrupt(POWER_BUTTON_PIN));
    while (digitalRead(POWER_BUTTON_PIN) == LOW) {
        // Wait for button to be released. Can't use delay() because we are inside an interrupt.
        delayMicroseconds(POWER_BUTTON_DELAY_MS * 1000);
    }
    // reattach MPU interrupt
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    // TODO: clean the FIFO Buffer? Unfortunately we never hit loop() again if we do that here

    // clear all recognized gestures
    clearGestures();
}

void enterSleep() {
    clearMatrix();
    while (digitalRead(POWER_BUTTON_PIN) == LOW) {
        // wait for button to be released
        delay(POWER_BUTTON_DELAY_MS);
    }
    // detach the MPU interrupts so MPU readings don't wake us up from sleep
    detachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
    // instead only accept interrupts from the power button
    attachInterrupt(digitalPinToInterrupt(POWER_BUTTON_PIN), afterWakeUp, LOW);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(powerButtonPin), enterSleep, HIGH);
    // attachInterrupt(digitalPinToInterrupt(powerButtonPin), a, LOW);

    // pinMode(MPU_INTERRUPT_PIN, OUTPUT);
    ledMatrix.init();
    ledMatrix.setIntensity(LED_INTENSITY);
    renderFace(SMILEY_FACE);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(MPU_INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // gyro and accel offsets for position with cables pointed down
    mpu.setXGyroOffset(125);
    mpu.setYGyroOffset(-14);
    mpu.setZGyroOffset(63);
    mpu.setXAccelOffset(-6717);
    mpu.setYAccelOffset(-545);
    mpu.setZAccelOffset(3125);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        renderFace(FROWNY_FACE);
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

/**
 * Applies quadratic weighted moving averages to all values in the queue.
 * Does not modify the queue, instead returns the average.
 */
float getAverageVal(Queue<float> * q) {
    int beta = 0;
    float totalLikelihood = 0;
    int length =  (*q).count();
    for (int i = 0; i < length; i++) {
        int multiplier = (i+1) * (i+1);
        beta += multiplier;
        totalLikelihood += multiplier * (*q).findAt(i);
    }
    return totalLikelihood / beta;
}

/**
 * Tests if the given delta of movement measurements is considered being active. 
 */
inline bool isActive(float delta) {
    return abs(delta) > ACTIVITY_THRESHOLD;
}

/**
 * Returns a value indicating the likelihood of the movement being within an activaty window.
 * Additionally sets 'positive' according to the direction of the gesture.
 */
float getActivityLikelihood(Queue<float> * q, bool * positive) {
    int beta = 0;
    float totalLikelihood = 0;
    int length = (*q).count();
    if (length < SMOOTHING_SAMPLE_SIZE) {
        // the queue isn't full yet and we can't deduce activity of such few samples
        return 0;
    }
    float accumulatedDeltas = 0;
    for (int i = 0; i < length; i++) {
        int multiplier = (i+1) * (i+1);
        beta += multiplier;
        float cur = (*q).findAt(i);
        float prev = i == 0 ? cur : (*q).findAt(i - 1);
        float delta = cur - prev;
        accumulatedDeltas += delta;
        totalLikelihood += multiplier * isActive(delta);
    }
    *positive = accumulatedDeltas >= 0;
    return totalLikelihood / beta;
}

/**
 * Recognizes the gesture by reading the queue of motion measurements produced by the MPU6050.
 * Waits at least MIN_GESTURE_TIME_MS milliseconds until it tries to detect a new gesture.
 */
void recognizeGesture(Queue<float> * q, Gesture &gesture) {
    boolean isPositive;
    float likelihood = getActivityLikelihood(q, &isPositive);
    unsigned long now = millis();
    if ( gesture.type != NONE && (now - gesture.begin) < MIN_GESTURE_TIME_MS) {
        // the gesture has not run for its minimum time so we do nothing
        return;
    }
    if (gesture.type == NONE && likelihood >= GESTURE_THRESHOLD) {
        // we detected a new gesture
        gesture.type = isPositive ? ROLL_LEFT : ROLL_RIGHT;
        gesture.begin = now;
    } else if (gesture.type != NONE && likelihood < GESTURE_THRESHOLD) {
        // a gesture has ended
        gesture.type = NONE;
    }
}

/**
 * Updates the 'activeGesture'. Returns true if the active gesture has changed, otherwise false. 
 */
bool updateGesture() {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if (queue.count() == SMOOTHING_SAMPLE_SIZE) {
        queue.pop();
    }
    queue.push(ypr[2]);
    recognizeGesture(&queue, lastRecognizedGesture);
    unsigned long now = millis();
    if (activeGesture.type == NONE && lastRecognizedGesture.type != NONE && (now - activeGesture.begin) > MIN_GESTURE_TIME_MS) {
        // a gesture was recognized that causes a change of state
        activeGesture = lastRecognizedGesture;
        return true;
    } else if (
        activeGesture.type == ROLL_LEFT && lastRecognizedGesture.type == ROLL_RIGHT ||
        activeGesture.type == ROLL_RIGHT && lastRecognizedGesture.type == ROLL_LEFT
    ) {
        activeGesture.type = NONE;
        activeGesture.begin = lastRecognizedGesture.begin;
        return true;
    }
    return false;
}

/**
 * Renders the active gesture on the LED matrices.
 */
void renderGesture(Gesture &gesture) {
    unsigned long now = millis(); 
    if (now - lastLedUpdate < LED_UPDATE_FREQ_MS) {
        return;
    }
    lastLedUpdate = now;

    ledMatrix.clear();
    if (gesture.type != NONE) {
        bool arrowIdx = gesture.type == ROLL_LEFT;
        for (int i = 0; i < NUMBER_OF_LED_COLUMNS; i++) {
            ledMatrix.setColumn(i, ARROWS[arrowIdx][activeArrow][i%8]);
        }
        ++activeArrow;
        activeArrow %= NUMBER_OF_LED_MATRICES;
    }
    ledMatrix.commit(); // commit transfers the byte buffer to the displays
}

void playTone() {
    // louder ton
    tone(8, activeGesture.type == NONE ? BLINK_END_TONE_FREC : BLINK_START_TONE_FREC, BLINK_TONE_DURATION);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    if (digitalRead(POWER_BUTTON_PIN) == LOW) {
        enterSleep();
    }

    renderGesture(activeGesture);

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // discard the first readings
        if (warmupCountdown > 0) {
            warmupCountdown--;
            return;
        }

        boolean gestureUpdated = updateGesture();

        if (gestureUpdated) {
            playTone();
        }
    }
}
