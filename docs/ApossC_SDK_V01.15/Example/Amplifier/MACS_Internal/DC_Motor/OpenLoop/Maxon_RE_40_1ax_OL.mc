/**
*	@brief		This program shows the control of a DC motor without encoder
*	@detail		This example shows the setup of an RE 40 without encoder.\n
*				A current can be specified, which is reached if possible. Without torque on the shaft, \n
*				the current can never be regulated to the desired value.
*				For this purpose the following must be configured: \n
*				- Amplifier \n
*				- ControllerMode and controller (current) \n
*				- Axis parameters (direction, limits) \n
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
#define AXIS1_ENCPORT 			0	// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual

// Axis settings
#define RE_40_CONTROLMODE	HWAMP_MODE_POS_CUR		// Define control typ
#define RE_40_POLEPAIRS		1						// Number of pole pairs
#define RE_40_CONTCUR		3200					// Nomial continious current allowed in mA
#define RE_40_MAXCUR		RE_40_CONTCUR*1.25		// Maximal current allowed in mA
#define RE_40_THERMAL_TIME	41500					// Thermal time constant of the winding

#define RE_40_MAX_RPM		3500					// Maximum velocity in RPM
#define RE_40_CURKPROP		1000					// Proportional factor of current controller
#define RE_40_CURKINT		100						// Integral factor of current controller
#define RE_40_CURKILIM		32767					// Integral limit of current controller

// not used in this example → HWAMP_MODE_POS_CUR
#define RE_40_VELKPROP		0						// Proportional factor of velocity controller
#define RE_40_VELKINT		1						// Integral factor of velocity controller
#define RE_40_VELKILIM		0						// Integral limit of velocity controller

#define RE_40_VELRES		100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define RE_40_RAMPTYPE		RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define RE_40_RAMPMIN		100						// Maximum acceleration
#define RE_40_JERKMIN		100						// Minimum time (ms) required before reaching the maximum acceleration
#define RE_40_POSERR		0						// Disable following error
#define RE_40_DIRECTION		1						// User units have normal orientation. Increasing encoder values result in increasing user positions.

#define	RE_40_KPROP			500						// Proportional value for PID position control loop
#define	RE_40_KINT			0						// Integral value for PID position control loop
#define	RE_40_KDER			1000					// Derivative value for PID position control loop
#define	RE_40_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	RE_40_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	RE_40_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	RE_40_FFVEL			0						// Velocity Feed forward
#define	RE_40_KFFAC			0						// Acceleration Feed forward
#define	RE_40_KFFDEC		0						// Deceleration Feed Forward

// Function delkratation
long setup_RE_40_OL(long axisNo, long encPort);

// Command current in mA, adaption while running possible (watch window)
long current = 1000;

long main(void)
{
	ErrorClear();
	DefOrigin(AXIS1_NO);

	// Setup axis & amplifier
	setup_RE_40_OL(AXIS1_NO,AXIS1_ENCPORT);

	// Activate the axis
	AxisControl(AXIS1_NO, USERREFCUR);

	while(1)
	{
		// Set current in mA. If the current is to be regulated to 0, sufficient torque must be applied to the motor shaft.
		// Alternatively, the axis can also be switched off with AxisControl(AXIS1_NO, OFF);
		AXE_PROCESS(AXIS1_NO,REG_USERREFCUR)=current;
	}

    return(0);
}


long setup_RE_40_OL(long axisNo, long encPort)
{
	// Disable following error and set compos to actpos
	VIRTCOUNTIN_PARAM(axisNo, VIRTCNTIN_PISRC_COUNTER) = AXE_PROCESS_SRCINDEX(axisNo, REG_COMPOS);

	// Amplifier setup
	sdkSetupAmpDcMotor(			axisNo,
								RE_40_CONTROLMODE,
								RE_40_POLEPAIRS,
								RE_40_MAXCUR,
								2000,	//Default value
								RE_40_MAX_RPM);
	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								RE_40_CURKPROP,
								RE_40_CURKINT,
								RE_40_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	axisNo,
								RE_40_VELKPROP,
								RE_40_VELKINT,
								RE_40_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								RE_40_VELRES,
								RE_40_MAX_RPM,
								RE_40_RAMPTYPE,
								RE_40_RAMPMIN,
								RE_40_JERKMIN,
								RE_40_POSERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		axisNo,
								RE_40_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									RE_40_KPROP,
									RE_40_KINT,
									RE_40_KDER,
									RE_40_KILIM,
									RE_40_KILIMTIME,
									RE_40_BANDWIDTH,
									RE_40_FFVEL,
									RE_40_KFFAC,
									RE_40_KFFDEC
									);

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,RE_40_CONTCUR, RE_40_THERMAL_TIME);

	return(1);
}