/* mbed Microcontroller Library
 * Copyright (c) 2013 Nordic Semiconductor
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed_assert.h"
#include "pwmout_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"

#define PWM_COUNT				8
#define PWM_PERIOD_DEFAULT      20000
#define COMENSATION_TICK		5

// 1uS precision, which with a 16bit timer gives us a maximum period of around 65ms.
#define TIMER_PRESCALER_MIN     4
#define TIMER_PRESCALER_MAX     9

#define NO_CHN (uint8_t)0xFF
#define NO_PIN (PinName)NC

static NRF_TIMER_Type* const timer = NRF_TIMER2;

// PWM period, both in absolute microseconds, and TIMER ticks (we cache both for simplicity and performance).
static uint32_t pwm_period_us = PWM_PERIOD_DEFAULT;
static volatile uint16_t pwm_period_ticks = PWM_PERIOD_DEFAULT;

// PWM levels, both in absolute microseconds, and TIMER ticks (we cache both for simplicity and performance).
static uint32_t pwm_us[PWM_COUNT] = {0};
static uint16_t pwm_ticks[PWM_COUNT] = {0};
static PinName pwm_pins[PWM_COUNT] = {NO_PIN};
static uint32_t pwm_pin_masks[PWM_COUNT] = {0};

static volatile uint8_t pwm_max = 0;
static uint8_t pwm_curr = 0;

// TIMER prescaler currently in use.
static uint8_t pwm_prescaler = TIMER_PRESCALER_MIN;

static uint8_t timer_enabled = 0;

static uint16_t PWM_US_TO_TICKS(uint32_t value) {
    return ((uint16_t) (value / ((uint32_t)(1 << (pwm_prescaler - TIMER_PRESCALER_MIN)))));
}

extern char* volatile out;
#define LOG(msg, arg...) out += sprintf(out, "\n" msg " @%d\n", ##arg, __LINE__)
#define LOGT(msg, arg...) out += sprintf(out, msg, ##arg)

#ifdef __cplusplus
extern "C" {
#endif

static volatile int irqCount = 0;

void TIMER2_IRQHandler(void) {
    timer->TASKS_STOP = 1;

    uint8_t next = pwm_curr;
    uint16_t tick = pwm_period_ticks;
    for (int i = 0; i < pwm_max; ++i) {
    	if (++next == pwm_max) next = 0;
    	if (pwm_ticks[next]) {
			NRF_GPIO->OUTCLR = pwm_pin_masks[pwm_curr];
    		NRF_GPIO->OUTSET = pwm_pin_masks[next];
    		tick = pwm_ticks[next];
    		pwm_curr = next;
    		break;
    	}
    }
//    LOGT("%d:%d\t", tick, pwm_curr);
    timer->CC[0] = tick - COMENSATION_TICK;
    timer->TASKS_START = 1;
    timer->EVENTS_COMPARE[0] = 0;
}

#ifdef __cplusplus
}
#endif

static void timer_init() {
    if (timer_enabled) return;

    timer->TASKS_STOP = 1;
    timer->POWER     = 0;
    timer->POWER     = 1;
    timer->MODE      = TIMER_MODE_MODE_Timer;
    timer->BITMODE   = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    timer->PRESCALER = pwm_prescaler;
    timer->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    timer->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;

    pwm_us[0] = pwm_period_us;
    pwm_ticks[0] = pwm_period_ticks;
    pwm_pins[0] = NO_PIN;
    pwm_pin_masks[0] = 0;
    pwm_max = 1;
    timer->CC[0] = pwm_ticks[0];

    NVIC_SetPriority(TIMER2_IRQn, 1);
    NVIC_EnableIRQ(TIMER2_IRQn);

    timer->TASKS_START = 1;
    timer_enabled = 1;
}

static uint8_t pwm_get_channel(PinName pin) {
    for (uint8_t chn = 1; chn < pwm_max; ++chn) {
        if (pwm_pins[chn] == pin) return chn;
    }
    return NO_CHN;
}

static void pwm_disconnect(PinName pin) {
    const uint8_t chn = pwm_get_channel(pin);
    if (chn == NO_CHN) return;
    pwm_us[0] += pwm_us[chn];
    pwm_ticks[0] += pwm_ticks[chn];
    pwm_us[chn] = 0;
    pwm_ticks[chn] = 0;
    pwm_pins[chn] = NO_PIN;
    NRF_GPIO->OUTCLR = pwm_pin_masks[chn];
    pwm_pin_masks[chn] = 0;
}

static uint8_t pwm_allocate_channel(PinName pin) {
	uint8_t chn;
//    LOG("start allocate 1 ~ %d", pwm_max - 1);
    for (chn = 1; chn < pwm_max; ++chn) {
//    	LOG("pin[%d]=%d", chn, pwm_pins[chn]);
        if (pwm_pins[chn] == NO_PIN) {
        	pwm_pins[chn] = pin;
        	pwm_pin_masks[chn] = 1UL << pin;
//        	LOG("old channel %d allocated, max %d", chn, pwm_max);
            return chn;
        }
    }
    if (pwm_max < PWM_COUNT) chn = pwm_max++;
    else pwm_disconnect(pwm_pins[chn = pwm_max - 1]);
    pwm_pins[chn] = pin;
    pwm_pin_masks[chn] = 1UL << pin;
//	LOG("new channel %d allocated, max %d", chn, pwm_max);
    return chn;
}

static uint8_t pwm_connect(PinName pin) {
	uint8_t chn = pwm_get_channel(pin);
//	LOG("connect channel %d", chn);
	if (chn == NO_CHN) chn = pwm_allocate_channel(pin);

    // Connect GPIO input buffers and configure PWM_OUTPUT_PIN_NUMBER as an output.
    NRF_GPIO->PIN_CNF[pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                            | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                            | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                            | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                            | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    NRF_GPIO->OUTCLR = pwm_pin_masks[chn];
    return chn;
}


void pwmout_init(pwmout_t *obj, PinName pin) {
    MBED_ASSERT(pin != NO_PIN);
    timer_init();

    const uint8_t chn = pwm_connect(pin);
    obj->pwm = chn;
    obj->pin = pin;
    pwmout_pulsewidth_us(obj, 0);
}

void pwmout_pulsewidth_us(pwmout_t *obj, int us) {
    PinName pin = (PinName)obj->pin;
    const uint8_t chn = pwm_connect(pin);
    uint16_t ticks = PWM_US_TO_TICKS(us);
    LOG("setPulseWidth pin %d chn %d ticks %d max %d div %d", pin, chn, ticks, pwm_max, pwm_prescaler);
    pwm_us[0] -= (us - pwm_us[chn]);
    pwm_ticks[0] -= (ticks - pwm_ticks[chn]);
    pwm_us[chn] = us;
    pwm_ticks[chn] = ticks;
    if (us == 0 || us >= pwm_period_us) {
        pwm_disconnect(pin);
        if (us) NRF_GPIO->OUTSET = (1UL << pin);
        return;
    }
//    for (int i = 0; i < pwm_max; ++i) {
//    	LOG("  %d. %d %d", i, pwm_pins[i], pwm_ticks[i]);
//    }

    // Configure to receive a one-shot interrupt when the next PWM period completes (also stop the timer at this point to avoid potential race conditions)
    __disable_irq();
    timer->EVENTS_COMPARE[0] = 0;
    timer->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    timer->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    __enable_irq();
}

void pwmout_free(pwmout_t *obj) {
    pwm_disconnect(obj->pin);
}

void pwmout_write(pwmout_t *obj, float value) {
    pwmout_pulsewidth_us(obj, (int)((float)pwm_period_us * value));
}

float pwmout_read(pwmout_t *obj) {
    const uint8_t chn = pwm_get_channel(obj->pin);
    if (chn == NO_CHN) return 0;
    return (float)pwm_us[chn] / (float)pwm_period_us;
}

void pwmout_period(pwmout_t *obj, float seconds) {
    pwmout_period_us(obj,(int)(seconds * (float)1000000));
}

void pwmout_period_ms(pwmout_t *obj, int ms) {
    pwmout_period_us(obj, ms * 1000);
}

// Set the PWM period, keeping the duty cycle the same.
void pwmout_period_us(pwmout_t *obj, int us) {
    uint32_t p = 0x10000;
    int prescaler = TIMER_PRESCALER_MIN;
    uint32_t old_period_us = pwm_period_us;

    // Quick validation - do nothing if the frequency is identical to it's current value.
    if (us == pwm_period_us) return;

    // Calculate the ideal prescaler for this value. If it's higher than our current prescaler.
    while (p < us && p != (uint32_t) (1 << 31)) {
        p = p << 1;
        prescaler++;
    }

    // Silently ignore requests to go out of scope (all we can do with the mbed API at present).
    if (prescaler > TIMER_PRESCALER_MAX) return;

    // Update global frequency timestamps.
    pwm_prescaler = prescaler;
    pwm_period_us = us;
    pwm_period_ticks = PWM_US_TO_TICKS(us);

    pwmout_t pwm;
    pwm.pin = NO_PIN;
    pwm.pwm = NO_CHN;
//    LOG("setPeriodUs %d", us);

    // Update all PWM channel values to maintin the same duty cycle (mbed standard behaviour).
    for (int i = 0; i < pwm_max; ++i) {
        if (pwm_ticks[i] != 0) {
            pwm.pin = pwm_pins[i];
            pwmout_pulsewidth_us(&pwm, (uint32_t) ((float)pwm_us[i] * ((float)us / (float)old_period_us)));
            pwm.pin = NO_PIN;
        }
    }
}

void pwmout_pulsewidth(pwmout_t *obj, float seconds) {
    pwmout_pulsewidth_us(obj,(int)(seconds * (float)1000000));
}

void pwmout_pulsewidth_ms(pwmout_t *obj, int ms) {
    pwmout_pulsewidth_us(obj, ms * 1000);
}


