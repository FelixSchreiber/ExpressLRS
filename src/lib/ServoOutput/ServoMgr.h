#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266) || defined(TARGET_R900_RX) 
#pragma once

#include <Arduino.h>
#if defined(TARGET_R900_RX)
#include <Servo.h>
    static constexpr uint8_t pwm_outputs[GPIO_PIN_PWM_OUTPUTS_COUNT] = GPIO_PIN_PWM_OUTPUTS_ARRAY;
#endif
class ServoMgr
{
public:
    ServoMgr(const uint8_t *const pins, const uint8_t outputCnt, uint32_t defaultInterval = 20000U);
    ~ServoMgr() { delete[] _refreshInterval; }

    // Initialize the pins for output
    void initialize();
    // Start/Update PWM by pulse width
    void writeMicroseconds(uint8_t ch, uint16_t valueUs);
    // Start/Update PWM by duty
    void writeDuty(uint8_t ch, uint16_t duty);
    // Stop PWM
    void stopPwm(uint8_t ch);
    // Stop any active PWM channels (and set LOW)
    void stopAllPwm();
    // Set a pin high/low (will stopPwm first if active)
    void writeDigital(uint8_t ch, bool value);

    inline uint16_t getRefreshInterval(uint8_t ch) const { return _refreshInterval[ch]; }
    void setRefreshInterval(uint8_t ch, uint16_t intervalUs);
    inline bool isPwmActive(uint8_t ch) const { return _activePwmChannels & (1 << ch); }
    inline bool isAnyPwmActive() const { return _activePwmChannels; }
    inline uint8_t getOutputCnt() const { return _outputCnt; }

    static const uint8_t PIN_DISCONNECTED = 0xff;

private:
#if defined(PLATFORM_ESP32)
    uint32_t _timerConfigs[8] = {0};
    void allocateLedcChn(uint8_t ch, uint16_t intervalUs, uint8_t pin);
#elif defined(TARGET_R900_RX)
    Servo *_servos[GPIO_PIN_PWM_OUTPUTS_COUNT];
#endif
    const uint8_t *const _pins;
    const uint8_t _outputCnt;
    uint16_t *_refreshInterval;
    uint32_t _activePwmChannels;
    uint8_t *_resolution_bits;
};

#endif