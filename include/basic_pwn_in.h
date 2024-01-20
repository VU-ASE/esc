#ifndef BASIC_PWM_IN_H
#define BASIC_PWM_IN_H

#include <SimpleFOC.h>



// Values from PCA as tuned by the servo tester
#define PULSE_LOW_END 1085
#define PULSE_HIGH_END 1860


volatile long prev_time = 0;
volatile long current_time = 0;
volatile long disconnect_counter = 0;
volatile long time_between_pulses_micro_seconds = 0;

float throttle = 0.0;

void signal_change() {
	disconnect_counter = 0;
	current_time = micros();
	if (current_time > prev_time) {
		time_between_pulses_micro_seconds = current_time - prev_time;
		prev_time = current_time;
	}
}


#define DEAD_ZONE_THRESH 0.01f

void pwm_signal_read() {
	if (disconnect_counter > 200) {
		throttle = 0.0f;
		return;
	}

	if (time_between_pulses_micro_seconds <= PULSE_HIGH_END && time_between_pulses_micro_seconds >= PULSE_LOW_END) {
		throttle = (((time_between_pulses_micro_seconds - PULSE_LOW_END) / (float) (PULSE_HIGH_END - PULSE_LOW_END)) * 2.0f) - 1.0f;

		if (throttle > 1.0f) { throttle = 1.0f; }
		if (throttle < -1.0f) { throttle = -1.0f; }
		if ((throttle <= DEAD_ZONE_THRESH) && (throttle >= 0.0f - DEAD_ZONE_THRESH)) {
		  throttle = 0.0;
		}

	}

	disconnect_counter += 1;
}




void pwm_signal_init() {
	pinMode(A_PWM, INPUT);
	attachInterrupt(digitalPinToInterrupt(A_PWM), signal_change, CHANGE);
}






#endif
