/**
*	@brief		Program example for setting the regulator parameters with the different controller steps
*	@detail		With the help of this programme an ECi30 Maxon motor is commissioned. To do this, the settings of the
*				motor/encoder must first be made (setupECi30_MiniMACS6()). It is important to select the correct axis module number.
*				It is recommended to implement the setup of a motor outside the main program in an external *.mh file.
*
*				After that there are 5 commissioning steps (depends on the motor and selected control system).
* 				To switch between the different steps you can change the macro C_COMM_METHODE.
* 				p.e. #define C_COMM_METHODE	C_METHODE_ENCODER_CHECK → #define C_COMM_METHODE	C_METHODE_HALL_CHECK
*
*				The parameters of the regulators can be set with the respective macro. A simplified setting
*				during the process is possible with the monitor file @file ut_Control_Loop_Parameter_Slider.zbm.
*				The methods are repeated cyclically in an endless loop.
*
*				If the macro C_RECORD_ENABLE is set to 1, a recording of the step methods is executed in each cycle.
*
* 				C_METHODE_ENCODER_CHECK
* 				The encoder values of the respective axis are displayed in the communication window.
*				The encoder position in cpt, qc and uu can be compared. If there is no result, the encoder is incorrectly connected,
*				incorrectly configured or in the wrong port.
*
* 				C_METHODE_HALL_CHECK
* 				The hall pattern is printed out. This must change according to the data sheet.
*
* 				C_METHODE_CURRENT_STEP
*				The initial values for the current controller (HWAMP_CURKPROP and HWAMP_CURKINT) can be calculated with the
*				current controller calculator (Tools → Current Control Calculator).
*				A current is applied to the motor in the form of a step. Attention, the rotor can move.
*				The settings for the step can make with the macros C_TIME_CURRENT_STEP [ms] and C_CURRENT_STEP [mA].
*				The step response can be viewed with the Osiloscope(Tools→Oszilocope(Record)).
* 				If necessary the correct the P and I value of the current controller have to be adapt.
*
*				C_METHODE_VELOCITY_STEP
*				Only if it needs a velocity controller, this is switched on.
*				The macros C_TIME_VELOCITY_STEP [ms] and C_VELOCITY_STEP[-0x4000 to 0x4000] are used to make the settings for the step.
*				Direction of rotation is changed in each cycle.
*				The step response can be viewed with the Osiloscope(Tools→Oszilocope(Record)).
* 				If necessary the correct the P and I value of the velocity controller have to be adapt.
*
* 				C_METHODE_POSITION_RAMP
* 				The macros C_POSITION_RAMP [uu] and C_CONTROLLER_CASCADING[HWAMP_MODE]are used to make the settings for the ramp.
*				In addition, the Acc(), Dec() and Vel() functions can be used to adjust the ramp.
*				The step response can be viewed with the Osiloscope(Tools→Oszilocope(Record)). A Free Run osziloscope can also be used for this purpose.
* 				If necessary the correct the P, I and D value of the position controller have to be adapt.
*				$Revision: 271 $
*
*/

#include <SysDef.mh>

#include "..\..\..\..\SDK\SDK_ApossC.mc"

// Axis module number
#define C_AXIS1						0						// Axis module number
#define C_AXIS1_ENCPORT 			0						// Encoder port number

// Definition of the methods
#define C_METHODE_ENCODER_CHECK		0
#define C_METHODE_HALL_CHECK		1
#define C_METHODE_CURRENT_STEP		2
#define C_METHODE_VELOCITY_STEP		3
#define C_METHODE_POSITION_RAMP		4

// Set methode and endable/ disable recording
#define C_COMM_METHODE				C_METHODE_ENCODER_CHECK
#define C_RECORD_ENABLE				1

// Settings for the current step
#define C_TIME_CURRENT_STEP			50			// [ms]
#define C_CURRENT_STEP				2000		// [mA]

// Settings for the velocity step
#define C_TIME_VELOCITY_STEP		2500		// [ms]
#define C_VELOCITY_STEP				0X2000		// scaled to the maximum speed with -0x4000 to 0x4000 internal units

// Settings for the position ramp
#define C_POSITION_RAMP				10*360					// [uu] → 5 turns
#define C_CONTROLLER_CASCADING		HWAMP_MODE_POS_VEL_CUR 	// SDO Dictionary: HWAMP_MODE

// Axis setup
#define AXIS_VELRES				100					    // Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define AXIS_RAMPTYPE			RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define AXIS_RAMPMIN			1000					// Maximum acceleration
#define AXIS_JERKMIN			500						// Minimum time (ms) required before reaching the maximum acceleration
#define AXIS_MAX_RPM			4000					// Maximum velocity in RPM
#define AXIS_POSERR				0						// Maximal track/ position error allowed in qc → disable for testing

#define	AXIS_POSENCREV			1						// Number of revolutions of the motor
#define	AXIS_POSENCQC			ECi30_621403_ENCRES		// Number of quadcounts in POSENCREV revolutions
#define	AXIS_POSFACT_Z			1						// Number of revolutions of the input shaft
#define	AXIS_POSFACT_N			1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	AXIS_FEEDREV			1						// Number of revolutions of the gear box output shaft
#define	AXIS_FEEDDIST			360						// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

// Encoder settings
#define ECi30_621403_ENCRES 			4*4096				// Resolution of the encoder for position feed back in increments (quadcounts)
#define ECi30_621403_ENC_LATCHTYPE 		0					// Defines the latch type: Index Line
#define ECi30_621403_ENC_LATCHPARAM 	0					// -
#define ECi30_621403_ENC_LATCHSLOPE 	HWLATCH_SLOPE_RISING// Defines the slope of the trigger signal (Default 1)

// Motor settings
#define ECi30_621403_CONTROLMODE	HWAMP_MODE_POS_VEL_CUR		// Define control typ
#define ECi30_621403_POLEPAIRS		4							// Number of pole pairs
#define ECi30_621403_CONTCUR		2280						// Nomial continious current allowed in mA
#define ECi30_621403_MAXCUR			ECi30_621403_CONTCUR*1.25	// Maximal current allowed in mA
#define ECi30_621403_THERMAL_TIME	27600						// Thermal time constant of the winding
#define ECi30_621403_MAX_RPM		AXIS_MAX_RPM				// Maximum velocity in RPM
#define ECi30_621403_DIRECTION		1							// Direction of the drive

// Parameter for the current controller
// The values were calculated with the Current Calculator Tool
#define ECi30_621403_CURKPROP		1500					// Proportional factor of current controller
#define ECi30_621403_CURKINT		150						// Integral factor of current controller
#define ECi30_621403_CURKILIM		32767					// Integral limit of current controller

// Parameter for the velocity controller
#define ECi30_621403_VELKPROP		2000					// Proportional factor of velocity controller
#define ECi30_621403_VELKINT		1000						// Integral factor of velocity controller
#define ECi30_621403_VELKILIM		0						// Integral limit of velocity controller

// Parameter for the position controller
#define	ECi30_621403_KPROP			500						// Proportional value for PID position control loop
#define	ECi30_621403_KINT			0						// Integral value for PID position control loop
#define	ECi30_621403_KDER			1000						// Derivative value for PID position control loop
#define	ECi30_621403_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	ECi30_621403_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	ECi30_621403_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	ECi30_621403_FFVEL			1000					// Velocity Feed forward
#define	ECi30_621403_KFFACC			0						// Acceleration Feed forward
#define	ECi30_621403_KFFDEC			0						// Deceleration Feed Forward

long setupECi30_621403(long axisNo);

long main(void)
{
	long waitingTime = 5000;
	long direction	= 1;

	setupECi30_621403(C_AXIS1);
	DefOrigin(C_AXIS1);

	while(1)
	{
		#if C_COMM_METHODE == C_METHODE_ENCODER_CHECK
			waitingTime=250;
			sdkMotorCommEncoderUserUnitCheck(C_AXIS1);
		#elif C_COMM_METHODE == C_METHODE_HALL_CHECK
			waitingTime=250;
			sdkMotorCommHallSignalCheck(C_AXIS1);
		#elif C_COMM_METHODE == C_METHODE_CURRENT_STEP
			sdkMotorCommCurrentStep(	C_AXIS1,
										C_CURRENT_STEP,
										C_TIME_CURRENT_STEP,
										C_RECORD_ENABLE);
		#elif C_COMM_METHODE == C_METHODE_VELOCITY_STEP
			sdkMotorCommVelStep(	C_AXIS1,
									direction*C_VELOCITY_STEP,
									C_TIME_VELOCITY_STEP,
									C_RECORD_ENABLE);
			direction = direction*-1;
		#elif C_COMM_METHODE == C_METHODE_POSITION_RAMP
			sdkMotorCommPositionRamp(	C_AXIS1,
										direction*C_POSITION_RAMP,
										C_CONTROLLER_CASCADING,
										C_RECORD_ENABLE);
			direction = direction*-1;
		#endif

		if(waitingTime>1000)
			print("Wait for ", waitingTime, " ms");
		Delay(waitingTime);
	}

    return(0);
}
// Axis, motor and encoder setup
long setupECi30_621403(long axisNo)
{
	print("ECi30 MiniMACS6");

	// Encoder setup
	sdkSetupIncEncoder(			axisNo,
								axisNo,
								ECi30_621403_ENCRES,
								ECi30_621403_ENC_LATCHTYPE,
								ECi30_621403_ENC_LATCHPARAM,
								ECi30_621403_ENC_LATCHSLOPE
								);
	// Amplifier setup
	#if 1
		sdkSetupAmpHallPmsmMotor(	axisNo,
									ECi30_621403_CONTROLMODE,
									ECi30_621403_POLEPAIRS,
									ECi30_621403_MAXCUR,
									ECi30_621403_ENCRES,
									ECi30_621403_MAX_RPM,
									-1
									);
	#else
		sdkSetupAmpBldcMotor(		axisNo,
									ECi30_621403_CONTROLMODE,
									ECi30_621403_POLEPAIRS,
									ECi30_621403_MAXCUR,
									ECi30_621403_ENCRES,
									ECi30_621403_MAX_RPM
									);
	#endif


	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								ECi30_621403_CURKPROP,
								ECi30_621403_CURKINT,
								ECi30_621403_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	axisNo,
								ECi30_621403_VELKPROP,
								ECi30_621403_VELKINT,
								ECi30_621403_VELKILIM
								);

	// Set the direction of the axis
	sdkSetupAxisDirection( 		axisNo,
								ECi30_621403_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									ECi30_621403_KPROP,
									ECi30_621403_KINT,
									ECi30_621403_KDER,
									ECi30_621403_KILIM,
									ECi30_621403_KILIMTIME,
									ECi30_621403_BANDWIDTH,
									ECi30_621403_FFVEL,
									ECi30_621403_KFFACC,
									ECi30_621403_KFFDEC);

	// Set the virtual I2T
	sdkSetupVirtualI2T(axisNo, ECi30_621403_CONTCUR, ECi30_621403_THERMAL_TIME);

	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								AXIS_VELRES,
								AXIS_MAX_RPM,
								AXIS_RAMPTYPE,
								AXIS_RAMPMIN,
								AXIS_JERKMIN,
								AXIS_POSERR
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								AXIS_POSENCREV,
								AXIS_POSENCQC,
								AXIS_POSFACT_Z,
								AXIS_POSFACT_N,
								AXIS_FEEDREV,
								AXIS_FEEDDIST
								);


	return(1);
}