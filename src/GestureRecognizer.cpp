#include "GestureRecognizer.h"

GestureRecognizer *recognizer;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

GestureRecognizer::GestureRecognizer() {
    // constructor
    queue = Queue<float>(SMOOTHING_SAMPLE_SIZE);
    recognizer = this;
}

bool GestureRecognizer::init() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

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
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
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
    }
    return dmpReady;
}

bool GestureRecognizer::processSensorData() {
    // wait for enough new packets available
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
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // if (warmupCountdown > 0) {
        //     warmupCountdown--;
        //     return false;
        // }

        return this->updateGesture();
    }

    return false;
}

bool inline GestureRecognizer::hasOverflown() {
    fifoCount = mpu.getFIFOCount();
    return (mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024;
}

void inline GestureRecognizer::handleOverflow() {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
}

/**
 * Updates the 'activeGesture'. Returns true if the active gesture has changed, otherwise false. 
 */
bool GestureRecognizer::updateGesture() {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if (queue.count() == SMOOTHING_SAMPLE_SIZE) {
        queue.pop();
    }
    // Serial.print("roll: ");
    // Serial.println(ypr[2]);
    queue.push(ypr[2]);
    this->recognizeGesture(&queue, lastRecognizedGesture);
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
 * Applies quadratic weighted moving averages to all values in the queue.
 * Does not modify the queue, instead returns the average.
 */
float GestureRecognizer::getAverageVal(Queue<float> * q) {
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
inline bool GestureRecognizer::isActive(float delta) {
    return abs(delta) > ACTIVITY_THRESHOLD;
}

/**
 * Returns a value indicating the likelihood of the movement being within an activaty window.
 * Additionally sets 'positive' according to the direction of the gesture.
 */
float GestureRecognizer::getActivityLikelihood(Queue<float> * q, bool * positive) {
    int beta = 0;
    float totalLikelihood = 0;
    int length = (*q).count();
    float accumulatedDeltas = 0;
    for (int i = 0; i < length; i++) {
        int multiplier = (i+1) * (i+1);
        beta += multiplier;
        float cur = (*q).findAt(i);
        float prev = i == 0 ? cur : (*q).findAt(i - 1);
        float delta = cur - prev;
        accumulatedDeltas += delta;
        totalLikelihood += multiplier * this->isActive(delta);
    }
    *positive = accumulatedDeltas >= 0;
    return totalLikelihood / beta;
}

/**
 * Recognizes the gesture by reading the queue of motion measurements produced by the MPU6050.
 * Waits at least MIN_GESTURE_TIME_MS milliseconds until it tries to detect a new gesture.
 */
void GestureRecognizer::recognizeGesture(Queue<float> * q, Gesture &gesture) {
    boolean isPositive;
    float likelihood = this->getActivityLikelihood(q, &isPositive);
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

void GestureRecognizer::clearGestures() {
    unsigned long now = millis();
    queue.clear();
    activeGesture.type = NONE;
    activeGesture.begin = now;
    lastRecognizedGesture.type = NONE;
    lastRecognizedGesture.begin = now;
}

void GestureRecognizer::enterSleep() {
    // detach the MPU interrupts so MPU readings don't wake us up from sleep
    detachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN));
    // TODO: put the MPU fully to sleep instead of just ignoring its interrupts
}

void GestureRecognizer::wakeUp() {
    // reattach MPU interrupt
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
    // TODO: clean the FIFO Buffer? Unfortunately we never hit loop() again if we do that here
}

Gesture GestureRecognizer::getActiveGesture() {
    return activeGesture;
}