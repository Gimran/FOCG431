#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>


static inline void setup_PIDs(FOCMotor *motor)
{

	motor -> PID_velocity.P = 0.5f;
	motor -> PID_velocity.I = 1.0f;	
	motor -> PID_velocity.P = 0.04f;
	motor -> PID_velocity.output_ramp = 0.0f;
	motor -> LPF_velocity.Tf = 0.0f;
	// current q loop PID
	motor -> PID_current_q.P = 0.5;
	motor -> PID_current_q.I = 10;
	motor -> PID_current_q.D = 0.0;
	motor -> PID_current_q.output_ramp = 0.0;
	motor -> PID_current_q.limit = 12.0;
	motor -> LPF_current_q.Tf = 0.01f;
	// current d loop PID
	motor -> PID_current_d.P = 0.5;
	motor -> PID_current_d.I = 1;
	motor -> PID_current_d.D = 0.0;
	motor -> PID_current_d.output_ramp = 0.0;
	motor -> PID_current_d.limit = 12.0;
	motor -> LPF_current_d.Tf = 0.01f;
}