/*
 * ESP32PWMV3.h
 *
 *  Created on: Sep 22, 2018
 *      Author: hephaestus
 */

#ifndef LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_
#define LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_
#include "esp32-hal-ledc.h"
#if defined(ARDUINO)
#include "Arduino.h"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define NUM_PWM 6
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
#define NUM_PWM 8
#else 
#define NUM_PWM 16
#endif

#define PWM_BASE_INDEX 0
#define USABLE_ESP32_PWM (NUM_PWM - PWM_BASE_INDEX)
#include <cstdint>

class ESP32PWM {
private:
    void attach(int pin);
    int pwmChannel = 0; // Channel number for this PWM
    bool attachedState = false;
    int pin;
    uint8_t resolutionBits;
    double myFreq;
    int allocatenext(double freq);

    static double _ledcSetupTimerFreq(uint8_t channel, double freq, uint8_t bit_num);

    bool checkFrequencyForSideEffects(double freq);

    void adjustFrequencyLocal(double freq, double dutyScaled);
    static double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
        if (x > in_max)
            return out_max;
        if (x < in_min)
            return out_min;
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    double setup(double freq, uint8_t resolution_bits = 10);
    void attachPin(uint8_t pin);
    void deallocate();

public:
    ESP32PWM();
    virtual ~ESP32PWM();

    void detachPin(int pin);
    void attachPin(uint8_t pin, double freq, uint8_t resolution_bits = 10);
    bool attached() {
        return attachedState;
    }

    void write(uint32_t duty);
    void writeScaled(double duty);
    double writeTone(double freq);
    double writeNote(note_t note, uint8_t octave);
    void adjustFrequency(double freq, double dutyScaled = -1);

    uint32_t read();
    double readFreq();
    double getDutyScaled();

    static int timerAndIndexToChannel(int timer, int index);
    static void allocateTimer(int timerNumber);
    static bool explicateAllocationMode;
    int getTimer() {
        return timerNum;
    }
    int timerNum = -1;
    uint32_t myDuty = 0;
    int getChannel();
    static int PWMCount; // The total number of attached PWM channels
    static int timerCount[4];
    static ESP32PWM* ChannelUsed[NUM_PWM]; // Used to track whether a channel is in service
    static long timerFreqSet[4];

    int getPin() {
        return pin;
    }
    static bool hasPwm(int pin) {
#if defined(CONFIG_IDF_TARGET_ESP32S2)
        if ((pin >= 1 && pin <= 21) || (pin == 26) || (pin >= 33 && pin <= 42))
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
        if ((pin >= 1 && pin <= 21) || (pin >= 35 && pin <= 45) || (pin == 47) || (pin == 48))
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
        if ((pin >= 1 && pin <= 10) || (pin >= 18 && pin <= 21))
#else
        if ((pin == 2) || (pin == 4) || (pin == 5) || ((pin >= 12) && (pin <= 19)) || ((pin >= 21) && (pin <= 23)) || ((pin >= 25) && (pin <= 27)) || (pin == 32) || (pin == 33))
#endif
            return true;
        return false;
    }
    static int channelsRemaining() {
        return NUM_PWM - PWMCount;
    }
    static bool DISABLE_DAC;
};

ESP32PWM* pwmFactory(int pin);

#endif /* LIBRARIES_ESP32SERVO_SRC_ESP32PWM_H_ */
