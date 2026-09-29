/**
*	@brief		This program shows the control of a BLDC motor without encoder
*	@detail		This example shows the setup of an EC45 Flat motor without encoder block commutated.\n
*				The Hall signal is used for position control.
*				For this purpose the following must be configured: \n
*				- Hall \n
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
#include "..\..\..\..\..\..\SDK\SDK_ApossC.mc"

// Setting the axis number. Hall ports are always bound to the respective axis number.
#define AXIS1_NO				0	// Axis module number

// Axe settings
#define EC45FLAT_CONTROLMODE	HWAMP_MODE_POS_CUR				// Define control typ
#define EC45FLAT_HALL_ALIGNMENT	HALL_ALIGNMENT_120DEGREE		// Hall alignment 120° standard
#define EC45FLAT_HALL_DIRECT	-1								// Hall direction is inverted when using maxon motors
#define EC45FLAT_POLEPAIRS		8								// Number of pole pairs
#define EC45FLAT_CONTCUR		2970							// Nomial continious current allowed in mA
#define EC45FLAT_MAXCUR			EC45FLAT_CONTCUR*1.25	// Maximal current allowed in mA
#define EC45FLAT_THERMAL_TIME	13400							// Thermal time constant of the winding

#define EC45FLAT_MAX_RPM		5500					// Maximum velocity in RPM
#define EC45FLAT_CURKPROP		2200					// Proportional factor of current controller
#define EC45FLAT_CURKINT		300						// Integral factor of current controller
#define EC45FLAT_CURKILIM		18000					// Integral limit of current controller

// not used in this example → HWAMP_MODE_POS_CUR
#define EC45FLAT_VELKPROP		2000					// Proportional factor of velocity controller
#define EC45FLAT_VELKINT		250						// Integral factor of velocity controller
#define EC45FLAT_VELKILIM		1000					// Integral limit of velocity controller

#define EC45FLAT_VELRES		100					    // Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define EC45FLAT_RAMPTYPE	RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define EC45FLAT_RAMPMIN	1000					// Maximum acceleration
#define EC45FLAT_JERKMIN	500						// Minimum time (ms) required before reaching the maximum acceleration
#define EC45FLAT_POSERR		500						// Maximal track/ position error allowed in qc
#define EC45FLAT_DIRECTION	1						// User units have normal orientation. Increasing encoder values result in increasing user positions.

#define	EC45FLAT_KPROP		1000					// Proportional value for PID position control loop
#define	EC45FLAT_KINT		0						// Integral value for PID position control loop
#define	EC45FLAT_KDER		2000					// Derivative value for PID position control loop
#define	EC45FLAT_KILIM		1000					// Limit value for the integral sum of the PID position control loop
#define	EC45FLAT_KILIMTIME	0						// Time used to increase or decrease the integral limit
#define	EC45FLAT_BANDWIDTH	1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	EC45FLAT_FFVEL		0						// Velocity Feed forward
#define	EC45FLAT_KFFACC		100						// Acceleration Feed forward
#define	EC45FLAT_KFFDEC		100						// Deceleration Feed Forward

#define	EC45FLAT_POSENCREV	1							// Number of revolutions of the motor
#define	EC45FLAT_POSENCQC	EC45FLAT_POLEPAIRS*6	// Number of quadcounts in POSENCREV revolutions
#define	EC45FLAT_POSFACT_Z	169							// Number of revolutions of the input shaft
#define	EC45FLAT_POSFACT_N	9							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	EC45FLAT_FEEDREV	1							// Number of revolutions of the gear box output shaft
#define	EC45FLAT_FEEDDIST	36000						// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

// Function delkratation
long setupEC45Flat_BC_Hall(long axisNo);

long main(void)
{
	ErrorClear();
	DefOrigin(AXIS1_NO);

	// Setup axis & amplifier
	setupEC45Flat_BC_Hall(AXIS1_NO);

	// Activate the axis
	AxisControl(AXIS1_NO, ON);

	// Start an endless movement with continuous move control with 50% velocity and 50% acceleration
	sdkStartContinuousMove(AXIS1_NO, 50, 50);

while(1)
	{
		// Print axis information all 500 ms
		sdkInfoPrintAxesPos();
		Delay(500);
	}

    return(0);
}


long setupEC45Flat_BC_Hall(long axisNo)
{
	// Hall setup for using it as encoder
	sdkSetupHallEncoder(		axisNo, HALL_ALIGNMENT_120DEGREE, EC45FLAT_HALL_DIRECT);

	// Amplifier setup
	HWAMP_PARAM(axisNo, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_BLDC; 				// Set motor type
	HWAMP_PARAM(axisNo, HWAMP_MODE)     = EC45FLAT_CONTROLMODE;         	// Set controller priciple
    HWAMP_PARAM(axisNo, HWAMP_POLES)    = EC45FLAT_POLEPAIRS;     			// Number of pole pairs
    HWAMP_PARAM(axisNo, HWAMP_MAXCUR)   = EC45FLAT_MAXCUR;  				// Max current in mA
	HWAMP_PARAM(axisNo, HWAMP_MAXRPM)   = EC45FLAT_MAX_RPM;  				// Given in RPM

	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								EC45FLAT_CURKPROP,
								EC45FLAT_CURKINT,
								EC45FLAT_CURKILIM
								);
	// Velocity control setup - not used at the moment
	sdkSetupVelocityPIControl( 	axisNo,
								EC45FLAT_VELKPROP,
								EC45FLAT_VELKINT,
								EC45FLAT_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								EC45FLAT_VELRES,
								EC45FLAT_MAX_RPM,
								EC45FLAT_RAMPTYPE,
								EC45FLAT_RAMPMIN,
								EC45FLAT_JERKMIN,
								EC45FLAT_POSERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		axisNo,
								EC45FLAT_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									EC45FLAT_KPROP,
									EC45FLAT_KINT,
									EC45FLAT_KDER,
									EC45FLAT_KILIM,
									EC45FLAT_KILIMTIME,
									EC45FLAT_BANDWIDTH,
									EC45FLAT_FFVEL,
									EC45FLAT_KFFACC,
									EC45FLAT_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								EC45FLAT_POSENCREV,
								EC45FLAT_POSENCQC,
								EC45FLAT_POSFACT_Z,
								EC45FLAT_POSFACT_N,
								EC45FLAT_FEEDREV,
								EC45FLAT_FEEDDIST
								);

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,EC45FLAT_CONTCUR, EC45FLAT_THERMAL_TIME);

	return(1);
}