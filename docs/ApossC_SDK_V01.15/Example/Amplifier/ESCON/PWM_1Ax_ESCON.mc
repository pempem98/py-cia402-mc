/**
*	@brief		Example program to control an ESCON by PWM.
*	@detail		With this example program an ESCON can be controlled by PWM signals
*				and a feedback signal of the actual speed is read back analog.
*				The example includes the possibility to set the PWM directly by a user parameter (variable dutyCycle)
*				or with the possibility to use the internal axis functions as profile generator (variable cvel) .
*				With the define PWM_MODE (line 34) you can switch between the two modes.
*
*				ESCON Settings:
*				Operating mode: Speed controller
*				Enable	:		Digital Input (high activ)
*				Cmd Velocity:	PWM: 10% = -5000 rpm, 90% = 5000
*				Ramp:			No ramp
*				Analog Output:	Actual velocity: 0V = -5000, 4V = 5000
*
*				Wiring:
*				ESCON					MACS
*				DI 1	----- PWM -----	DO 0 	(MACS HW Manual Number 1)
*				DI 2	---- Enable ---	DO 1	(MACS HW Manual Number 2)
*				AO 1	----- PWM -----	AI 0	(MACS HW Manual Number 1)
*
*
*	$Revision: 244 $
*
*	@example 	PWM_1Ax_ESCON.mc
*
*/
#include "..\..\..\SDK\SDK_ApossC.mc"

// Application settings to change the mode
#define PWM_AXIS_MODE	0
#define PWM_USER_MODE	1

#define PWM_MODE	PWM_AXIS_MODE
//#define PWM_MODE	PWM_USER_MODE

// ESCON control settings
#define ESCON_AXIS_NO				0			// Axis number (zero based) → PWM_AXIS_MODE
#define ESCON_PWM_USER_PARAM		1			// User parameter number → PWM_USER_MODE

#define ESCON_PWM_DIGOUTPUT_NO		0			// Digital output for PWM cmd value (zero based)
#define ESCON_ENABLE_DIGOUTPUT_NO	1			// Digital output for ESCON enable (zero based)

#define ESCON_PWM_FREQUENCY			50			// PWM frequency [Hz]
#define ESCON_PWM_CYCLE_RANGE		80			// PWM cycle range 10-90%
#define ESCON_PWM_POLARITY			HWPWMGEN_POLARITY_POSITIVE

// Analog feedback defines
#define ESCON_AI_FEEDBACK_NO		0			// Analog input number (zero based)
#define	ESCON_AI_SPEED_MIN_VOLTAGE	0			// 0 voltage → -5000 rpm
#define	ESCON_AI_SPEED_OFF_VOLTAGE	2000		// offset: 2 voltage → 0 rpm
#define	ESCON_AI_SPEED_MAX_VOLTAGE	4000		// 4 voltage → 5000 rpm

#define	ESCON_AI_SPEED_MIN_RPM		-5000		// 0 voltage → -5000 rpm
#define	ESCON_AI_SPEED_MAX_RPM		5000		// 4 voltage → 5000 rpm

// Set velocity for pwm axis mode (0 → +/- 100)
long cvel		=	0;

// Set duty cycle for pwm user parameter mode (10 → 90%)
long dutyCycle 	= 0x7FFF;  // 50%


long main(void)
{
	#if PWM_MODE == PWM_USER_MODE
		sdkSetupPwmGenerator_UserParamMode(	ESCON_PWM_USER_PARAM,
											ESCON_PWM_DIGOUTPUT_NO,
											ESCON_PWM_FREQUENCY,
											ESCON_PWM_CYCLE_RANGE,
											ESCON_PWM_POLARITY);

	#else
		sdkSetupPwmGenerator_AxisModeVel(	ESCON_AXIS_NO,
											ESCON_ENABLE_DIGOUTPUT_NO,
											ESCON_PWM_DIGOUTPUT_NO,
											ESCON_PWM_FREQUENCY,
											ESCON_PWM_CYCLE_RANGE,
											ESCON_PWM_POLARITY);
	#endif

	sdkScaleAnalogInput(	ESCON_AI_FEEDBACK_NO,
							ESCON_AI_SPEED_MIN_VOLTAGE,
							ESCON_AI_SPEED_MAX_VOLTAGE,
							ESCON_AI_SPEED_OFF_VOLTAGE,
							ESCON_AI_SPEED_MIN_RPM,
							ESCON_AI_SPEED_MAX_RPM);


	#if PWM_MODE == PWM_USER_MODE
		// Automatic disable if there is an error
		GLB_PARAM(ESCCONDGLB)=GLB_PARAM_ESCCONDGLB_OUT_OFF;

		// Reset enable output if there was an error
		DigOutput(ESCON_ENABLE_DIGOUTPUT_NO+1,0);
		Delay(50);

		// Enable ESCON Controller
		DigOutput(ESCON_ENABLE_DIGOUTPUT_NO+1,1);

	#else
		// Enable axis
		AxisControl(ESCON_AXIS_NO, ON);

		// Set profil
		Acc(ESCON_AXIS_NO,100);
		Dec(ESCON_AXIS_NO,100);

		// Set constant velocity and start axis
		Cvel(ESCON_AXIS_NO,cvel);
		AxisCvelStart(ESCON_AXIS_NO);
	#endif

	while(1)
	{
		#if PWM_MODE == PWM_USER_MODE
			// reset dutycycle
			USER_PARAM(ESCON_PWM_USER_PARAM)=dutyCycle;
			print("HWPWMGEN_MODE_UNSIGNED, PWM DuryCycle: ",HWPWMGEN_PROCESS(ESCON_PWM_DIGOUTPUT_NO,PO_HWPWMGEN_VALUE));
		#else
			// Set constant velocity and restart axis
			Cvel(ESCON_AXIS_NO,cvel);
			AxisCvelStart(ESCON_AXIS_NO);
			print("HWPWMGEN_MODE_SIGNED, PWM DuryCycle: ",HWPWMGEN_PROCESS(ESCON_PWM_DIGOUTPUT_NO,PO_HWPWMGEN_VALUE));
		#endif
		print("ESCON Feedback: ",AnalogInput(ESCON_AI_FEEDBACK_NO + 1)," Rpm");
		print("");
		Delay(2000);
	}
}