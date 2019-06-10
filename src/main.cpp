// #include <Wire.h>

#include "GestureRecognizer.h"
#include "avr/sleep.h"
#include <math.h>

#include "LedMatrix.h"

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

GestureRecognizer gestureRecognizer;

uint8_t activeArrow = 0;
LedMatrix ledMatrix = LedMatrix(NUMBER_OF_LED_MATRICES, LED_CS_PIN);
enum {SMILEY_FACE, NEUTRAL_FACE, FROWNY_FACE};
unsigned long lastLedUpdate = 0;
bool initSuccessful = false;

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

void afterWakeUp() {
    renderFace(SMILEY_FACE);

    // detach Interrupt so it only triggers once
    detachInterrupt(digitalPinToInterrupt(POWER_BUTTON_PIN));
    while (digitalRead(POWER_BUTTON_PIN) == LOW) {
        // Wait for button to be released. Can't use delay() because we are inside an interrupt.
        delayMicroseconds(POWER_BUTTON_DELAY_MS * 1000);
    }
    gestureRecognizer.wakeUp();
    gestureRecognizer.clearGestures();
}

void enterSleep() {
    clearMatrix();
    while (digitalRead(POWER_BUTTON_PIN) == LOW) {
        // wait for button to be released
        delay(POWER_BUTTON_DELAY_MS);
    }
    gestureRecognizer.enterSleep();
    // instead only accept interrupts from the power button
    attachInterrupt(digitalPinToInterrupt(POWER_BUTTON_PIN), afterWakeUp, LOW);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    sleep_disable();
}

void setup() {
    // initialize serial communication
    Serial.begin(115200);
    pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);

    ledMatrix.init();
    ledMatrix.setIntensity(LED_INTENSITY);
    renderFace(SMILEY_FACE);

    gestureRecognizer = * new GestureRecognizer();
    initSuccessful = gestureRecognizer.init();
    if (!initSuccessful) {
        renderFace(FROWNY_FACE);
    }
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

void playTone(Gesture &gesture) {
    // louder ton
    tone(8, gesture.type == NONE ? BLINK_END_TONE_FREC : BLINK_START_TONE_FREC, BLINK_TONE_DURATION);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!initSuccessful) return;

    if (digitalRead(POWER_BUTTON_PIN) == LOW) {
        enterSleep();
    }

    Gesture gesture = gestureRecognizer.getActiveGesture();
    renderGesture(gesture);

    bool gestureChanged = gestureRecognizer.processSensorData();
    if (gestureChanged) {
        Gesture newGesture = gestureRecognizer.getActiveGesture();
        playTone(newGesture);
    }
}
