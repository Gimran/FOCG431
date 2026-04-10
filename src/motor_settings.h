#ifndef MOTOR_SETTINGS_H_
#define MOTOR_SETTINGS_H_
#ifdef __cplusplus
extern "C" {
#endif


// #define BLDC_2804
#define BLDC_MOSRAC_U3822

#ifdef BLDC_2804
#define POLE_PAIRS        7
#define PHASE_RESISTANCE  2.3f
#define KV_RATING         220
#define L_Q               0.00086f

#define SHUNT_OM          0.005f
#define GAIN              10

#define MAX_CURRENT       0.3f
#define OPERATION_VOLTAGE 12.0f
#define PWM_FREQ          20000

#elifdef BLDC_MOSRAC_U3822

#define POLE_PAIRS 7
#define PHASE_RESISTANCE 0.7f
#define KV_RATING 1900
#define L_Q 0.00004f
#define SHUNT_OM 0.005f
#define GAIN 40

#define MAX_CURRENT       2.0f
#define OPERATION_VOLTAGE 27.0f
#define PWM_FREQ          20000

#endif


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* DRV8323_DEFS_H_ */
