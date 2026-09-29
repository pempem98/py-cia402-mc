/**
*	@brief		This program shows the control of a stepper motor with encoder
*	@detail		This example shows the setup of an stepper motor with encoder.\n
*				The incremental ecnoder is used for position control.
*				For this purpose the following must be configured: \n
*				- Encoder \n
*				- Amplifier \n
*				- ControllerMode and controller (position, velocity, current) \n
*				- Axis parameters (user units, direction, limits) \n
*
*				It is recommended to implement the setup of a motor outside the main program in an external *.mh file.
*
*	$Revision: 271 $
*
*/

#include <SysDef.mh>

// Relative path to the main folder ApossC_SDK_Vxx.xx
#include "..\..\..\..\..\SDK\SDK_ApossC.mc"

// Setting the axis number.
#define AXIS1_NO				0	// Axis module number
#define AXIS1_ENCPORT 			0	// Encoder port number.

// Axis settings
#define STEPPER_CL_STEPS 		200						// Resolution of the stepper → 200  Steps per rev → 1.8 deg/ step
#define STEPPER_CL_ENCRES 		4000					// Resolution of the encoder for position feed back in increments (quadcounts)

#define STEPPER_CL_CONTROLMODE	HWAMP_MODE_POS_CUR			// Define control typ
#define STEPPER_CL_CONTCUR		4000						// Nomial continious current allowed in mA
#define STEPPER_CL_MAXCUR		STEPPER_CL_CONTCUR * 1.25	// Maximal current allowed in mA
#define STEPPER_CL_THERMAL_TIME	25600						// Thermal time constant of the winding
#define STEPPER_CL_MAX_RPM		400						// Maximum velocity in RPM

// Portescap P532 258 0.7 10 (parallel): 0.35Ohm / 0.7mH / Tools → Current Regulator Calculator → if there is noise the values are to high
#define STEPPER_CL_CURKPROP		20000						// Proportional factor of current controller
#define STEPPER_CL_CURKINT		250							// Integral factor of current controller
#define STEPPER_CL_CURKILIM		32767						// Integral limit of current controller

#define STEPPER_CL_VELRES		100							// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define STEPPER_CL_RAMPTYPE		RAMPTYPE_JERKLIMITED		// Defines the ramptype
#define STEPPER_CL_RAMPMIN		100							// Maximum acceleration
#define STEPPER_CL_JERKMIN		100							// Minimum time (ms) required before reaching the maximum acceleration
#define STEPPER_CL_POSERR		2000						// Maximal track/ position error allowed in qc
#define STEPPER_CL_DIRECTION	1							// User units have normal orientation. Increasing encoder values result in increasing user positions.

#define	STEPPER_CL_POSENCREV	1							// Number of revolutions of the motor
#define	STEPPER_CL_POSENCQC		STEPPER_CL_ENCRES			// Number of quadcounts in POSENCREV revolutions
#define	STEPPER_CL_POSFACT_Z	1							// Number of revolutions of the input shaft
#define	STEPPER_CL_POSFACT_N	1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	STEPPER_CL_FEEDREV		1							// Number of revolutions of the gear box output shaft
#define	STEPPER_CL_FEEDDIST		36000			// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

#define	STEPPER_CL_KPROP			60						// Proportional value for PID position control loop
#define	STEPPER_CL_KINT				0						// Integral value for PID position control loop
#define	STEPPER_CL_KDER				3000						// Derivative value for PID position control loop
#define	STEPPER_CL_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	STEPPER_CL_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	STEPPER_CL_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	STEPPER_CL_FFVEL			0						// Velocity Feed forward
#define	STEPPER_CL_KFFAC			0						// Acceleration Feed forward
#define	STEPPER_CL_KFFDEC			0						// Deceleration Feed Forward

// Function delkratation
long setupStepper_CL(long axisNo, long encPort);

long main(void)
{
	ErrorClear();
	DefOrigin(AXIS1_NO);

	// Setup axis & amplifier
	setupStepper_CL(AXIS1_NO,AXIS1_ENCPORT);

	// Activate the axis
	AxisControl(AXIS1_NO, ON);

	// Start an endless movement with continuous move control with 75% velocity and 75% acceleration
	sdkStartContinuousMove(AXIS1_NO, 75, 75);

while(1)
	{
		// Print axis information all 500 ms
		sdkInfoPrintAxesPos();
		Delay(500);
	}

    return(0);
}


long setupStepper_CL(long axisNo, long encPort)
{
	long checkMotorAlignment=0;

	// Amplifier setup
	sdkSetupAmpStepMotor_CL(	axisNo,
								STEPPER_CL_CONTROLMODE,
								STEPPER_CL_STEPS,
								STEPPER_CL_MAXCUR,
								STEPPER_CL_ENCRES,
								STEPPER_CL_MAX_RPM
								);
	// At high speeds, the lag value can be determined empirically. Otherwise this line can be commented out
	HWAMP_PARAM(axisNo,HWAMP_ELPOS_LAGMULT) = 115;

	// Encoder setup
	sdkSetupIncEncoder(			axisNo,
								encPort,
								STEPPER_CL_ENCRES,
								0,
								0,
								0);

	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								STEPPER_CL_CURKPROP,
								STEPPER_CL_CURKINT,
								STEPPER_CL_CURKILIM
								);

	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								STEPPER_CL_VELRES,
								STEPPER_CL_MAX_RPM,
								STEPPER_CL_RAMPTYPE,
								STEPPER_CL_RAMPMIN,
								STEPPER_CL_JERKMIN,
								STEPPER_CL_POSERR
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								STEPPER_CL_POSENCREV,
								STEPPER_CL_POSENCQC,
								STEPPER_CL_POSFACT_Z,
								STEPPER_CL_POSFACT_N,
								STEPPER_CL_FEEDREV,
								STEPPER_CL_FEEDDIST
								);

	sdkSetupAxisDirection(axisNo, STEPPER_CL_DIRECTION);


	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									STEPPER_CL_KPROP,
									STEPPER_CL_KINT,
									STEPPER_CL_KDER,
									STEPPER_CL_KILIM,
									STEPPER_CL_KILIMTIME,
									STEPPER_CL_BANDWIDTH,
									STEPPER_CL_FFVEL,
									STEPPER_CL_KFFAC,
									STEPPER_CL_KFFDEC
									);
	// Motor Alignment
	checkMotorAlignment =	sdkMotorAlignment(
							axisNo,
							3,
							STEPPER_CL_MAXCUR,
							STEPPER_CL_CONTCUR);

	if(checkMotorAlignment<0)
	{
		print("Sorry alignement didn't work we exit");
		Exit(0);
	}

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,STEPPER_CL_CONTCUR, STEPPER_CL_THERMAL_TIME);

	return(1);
}