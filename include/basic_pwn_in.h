#ifndef BASIC_PWM_IN_H
#define BASIC_PWM_IN_H

#include <SimpleFOC.h>


// Values for White remote controller
// TODO these are old values
#define PCA_LOW_END 1000
#define PCA_HIGH_END 2000


// Values for our GO implementation
// TODO These are values setup by the mod-Actuator
// #define PCA_LOW_END 940
// #define PCA_HIGH_END 3740


volatile long start_time = 0;
volatile long current_time = 0;
volatile long time_between_pulses_micro_seconds = 0;
float esc_pwm_input = 0.0;

void pulse_timer() {
	current_time = micros();
	if (current_time > start_time) {
		time_between_pulses_micro_seconds = current_time - start_time;
		start_time = current_time;
	}
	return;
}

void pca_input_init() {
	pinMode(A_PWM, INPUT);
	attachInterrupt(digitalPinToInterrupt(A_PWM), pulse_timer, CHANGE);
}


// TODO: Optimize such that the value is scaled to the speed range immediately
// instead of scaling to (-1.0, 1.0) first
void pca_input_read() {
	if (time_between_pulses_micro_seconds < PCA_HIGH_END + 100) {
		esc_pwm_input = (((time_between_pulses_micro_seconds - PCA_LOW_END) / (float) (PCA_HIGH_END - PCA_LOW_END)) * 2.0f) - 1.0f;
	}
	if (esc_pwm_input > 1.0f) {
		esc_pwm_input = 1.0f;
	}
	if (esc_pwm_input < -1.0f) {
		esc_pwm_input = -1.0f;
	}
}



#endif
