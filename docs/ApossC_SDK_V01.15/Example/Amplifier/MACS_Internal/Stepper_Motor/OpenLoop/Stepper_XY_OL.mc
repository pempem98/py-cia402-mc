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

// Axis settings
#define STEPPER_OL_STEPS 		200						// Resolution of the stepper → 200  Steps per rev → 1.8 deg/ step

#define STEPPER_OL_CONTCUR		4000						// Nomial continious current allowed in mA
#define STEPPER_OL_MAXCUR		STEPPER_OL_CONTCUR * 1.25	// Maximal current allowed in mA
#define STEPPER_OL_THERMAL_TIME	25600						// Thermal time constant of the winding
#define STEPPER_OL_MAX_RPM		400							// Maximum velocity in RPM

// Portescap P532 258 0.7 10 (parallel): 0.35Ohm / 0.7mH / Tools → Current Regulator Calculator → if there is noise the values are to high
#define STEPPER_OL_CURKPROP		20000						// Proportional factor of current controller
#define STEPPER_OL_CURKINT		250							// Integral factor of current controller
#define STEPPER_OL_CURKILIM		32767						// Integral limit of current controller

#define STEPPER_OL_VELRES		100							// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define STEPPER_OL_RAMPTYPE		RAMPTYPE_JERKLIMITED		// Defines the ramptype
#define STEPPER_OL_RAMPMIN		100							// Maximum acceleration
#define STEPPER_OL_JERKMIN		100							// Minimum time (ms) required before reaching the maximum acceleration
#define STEPPER_OL_POSERR		0						// Maximal track/ position error allowed in qc
#define STEPPER_OL_DIRECTION	1							// User units have normal orientation.

#define	STEPPER_OL_POSENCREV	1							// Number of revolutions of the motor
#define	STEPPER_OL_POSFACT_Z	1							// Number of revolutions of the input shaft
#define	STEPPER_OL_POSFACT_N	1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	STEPPER_OL_FEEDREV		1							// Number of revolutions of the gear box output shaft
#define	STEPPER_OL_FEEDDIST		36000						// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

#define	STEPPER_OL_KPROP			0						// Proportional value for PID position control loop
#define	STEPPER_OL_KINT				0						// Integral value for PID position control loop
#define	STEPPER_OL_KDER				0						// Derivative value for PID position control loop
#define	STEPPER_OL_KILIM			0						// Limit value for the integral sum of the PID position control loop
#define	STEPPER_OL_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	STEPPER_OL_BANDWIDTH		0						// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	STEPPER_OL_FFVEL			0						// Velocity Feed forward
#define	STEPPER_OL_KFFAC			0						// Acceleration Feed forward
#define	STEPPER_OL_KFFDEC			0						// Deceleration Feed Forward

// Function delkratation
long setupStepper_OL(long axisNo);

long main(void)
{
	ErrorClear();
	DefOrigin(AXIS1_NO);

	// Setup axis & amplifier
	setupStepper_OL(AXIS1_NO);

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


long setupStepper_OL(long axisNo)
{
	// The current position is set equal to the target position as there is no feedback
	sdkSetupAxisSimulation(axisNo);

	// The following error monitoring is switched off
	// If the FW is newer than 9.4.00 the parameter can also be set to 0 to switch off the monitoring
    AXE_PARAM(axisNo,POSERR) =0x7FFFFFFF;

	// Amplifier setup
	sdkSetupAmpStepMotor_OL(	axisNo,
								STEPPER_OL_STEPS,
								STEPPER_OL_MAXCUR,
								STEPPER_OL_MAX_RPM
								);

	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								STEPPER_OL_CURKPROP,
								STEPPER_OL_CURKINT,
								STEPPER_OL_CURKILIM
								);

	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								STEPPER_OL_VELRES,
								STEPPER_OL_MAX_RPM,
								STEPPER_OL_RAMPTYPE,
								STEPPER_OL_RAMPMIN,
								STEPPER_OL_JERKMIN,
								STEPPER_OL_POSERR
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								STEPPER_OL_POSENCREV,
								HWAMP_PARAM(axisNo, HWAMP_POLES) * HWAMP_ENCRES_MICROSTEP_RES,
								STEPPER_OL_POSFACT_Z,
								STEPPER_OL_POSFACT_N,
								STEPPER_OL_FEEDREV,
								STEPPER_OL_FEEDDIST
								);

	sdkSetupAxisDirection(axisNo, STEPPER_OL_DIRECTION);


	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									STEPPER_OL_KPROP,
									STEPPER_OL_KINT,
									STEPPER_OL_KDER,
									STEPPER_OL_KILIM,
									STEPPER_OL_KILIMTIME,
									STEPPER_OL_BANDWIDTH,
									STEPPER_OL_FFVEL,
									STEPPER_OL_KFFAC,
									STEPPER_OL_KFFDEC
									);

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,STEPPER_OL_CONTCUR, STEPPER_OL_THERMAL_TIME);


	return(1);
}