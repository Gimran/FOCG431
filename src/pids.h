#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>


static inline void setup_PIDs(FOCMotor *motor)
{
	motor -> voltage_limit = MOTOR_VOLTAGE_LIMIT;
	// angle loop PID
	motor -> P_angle.P = 5.0f;
	motor -> P_angle.I = 0.0f;
	motor -> P_angle.D = 0.0f;
	motor -> P_angle.output_ramp = 0.0f;
	motor -> LPF_angle.Tf = 0.0f;
	// velocity loop PID
	motor -> PID_velocity.P = 0.5f;
	motor -> PID_velocity.I = 5.0f;
	motor -> PID_velocity.output_ramp = 0.0f;
	motor -> LPF_velocity.Tf = 0.0f;
	// current q loop PID
	motor -> PID_current_q.P = 0.3;
	motor -> PID_current_q.I = 50.0f;
	motor -> PID_current_q.D = 0.0f;
	motor -> PID_current_q.output_ramp = 0.0f;
	motor -> PID_current_q.limit = 12.0f;
	motor -> LPF_current_q.Tf = 0.002f;
	// current d loop PID
	motor -> PID_current_d.P = 0.5f;
	motor -> PID_current_d.I = 50.0f;
	motor -> PID_current_d.D = 0.0f;
	motor -> PID_current_d.output_ramp = 0.0f;
	motor -> PID_current_d.limit = 12.0f;
	motor -> LPF_current_d.Tf = 0.002f;
}