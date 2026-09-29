/**
*	@brief		This program shows the control of two BLDC motor with encoder
*	@detail		This example shows the setup of two ECi30 motor with a incremental encoder sinus commutated.\n
*				The alignment is made with a hall sensor. The incremental ecnoder is used for position control.\n
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
#include "..\..\..\..\..\..\SDK\SDK_ApossC.mc"

// Setting the axis number. Hall ports are always bound to the respective axis number.
#define AXIS1_NO				0	// Axis module number
#define AXIS1_ENCPORT 			0	// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
#define AXIS2_NO				1	// Axis module number
#define AXIS2_ENCPORT 			1	// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual

// Axe settings
#define ECi30_ENCRES 			4*4096				// Resolution of the encoder for position feed back in increments (quadcounts)
#define ECi30_ENC_LATCHTYPE 	0					// Default latchTyp Encoder Z signal
#define ECi30_ENC_LATCHPARAM 	0					// Not used for Encoder Z Latch
#define ECi30_ENC_LATCHSLOPE 	HWLATCH_SLOPE_RISING// Defines the slope of the trigger signal (Default 1)

#define ECi30_CONTROLMODE		HWAMP_MODE_POS_CUR				// Define control typ
#define ECi30_HALL_ALIGNMENT	HALL_ALIGNMENT_120DEGREE		// Hall alignment 120° standard
#define ECi30_POLEPAIRS			4								// Number of pole pairs
#define ECi30_CONTCUR			2280							// Nomial continious current allowed in mA
#define ECi30_MAXCUR			ECi30_CONTCUR*1.25				// Maximal current allowed in mA
#define ECi30_THERMAL_TIME		27600							// Thermal time constant of the winding

#define ECi30_MAX_RPM		4000					// Maximum velocity in RPM
#define ECi30_CURKPROP		1500					// Proportional factor of current controller
#define ECi30_CURKINT		150						// Integral factor of current controller
#define ECi30_CURKILIM		32767					// Integral limit of current controller

// not used in this example → HWAMP_MODE_POS_CUR
#define ECi30_VELKPROP		0						// Proportional factor of velocity controller
#define ECi30_VELKINT		0						// Integral factor of velocity controller
#define ECi30_VELKILIM		0						// Integral limit of velocity controller

#define ECi30_VELRES		100					    // Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define ECi30_RAMPTYPE		RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define ECi30_RAMPMIN		1000					// Maximum acceleration
#define ECi30_JERKMIN		500						// Minimum time (ms) required before reaching the maximum acceleration
#define ECi30_POSERR		500						// Maximal track/ position error allowed in qc
#define ECi30_DIRECTION		1						// User units have normal orientation. Increasing encoder values result in increasing user positions.

#define	ECi30_KPROP			80						// Proportional value for PID position control loop
#define	ECi30_KINT			0						// Integral value for PID position control loop
#define	ECi30_KDER			200					// Derivative value for PID position control loop
#define	ECi30_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	ECi30_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	ECi30_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	ECi30_FFVEL			0						// Velocity Feed forward
#define	ECi30_KFFACC		0						// Acceleration Feed forward
#define	ECi30_KFFDEC		0						// Deceleration Feed Forward

#define	ECi30_POSENCREV		1						// Number of revolutions of the motor
#define	ECi30_POSENCQC		ECi30_ENCRES			// Number of quadcounts in POSENCREV revolutions
#define	ECi30_POSFACT_Z		1						// Number of revolutions of the input shaft
#define	ECi30_POSFACT_N		1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	ECi30_FEEDREV		1						// Number of revolutions of the gear box output shaft
#define	ECi30_FEEDDIST		360						// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

// Function delkratation
long setupECi30_SC_Hall_Inc(long axisNo, long encPort);

long main(void)
{
	long direction = 1;
	long turns = 100;
	ErrorClear();
	DefOrigin(AXALL);

	// Setup axis & amplifier
	setupECi30_SC_Hall_Inc(AXIS1_NO,AXIS1_ENCPORT);
	setupECi30_SC_Hall_Inc(AXIS2_NO,AXIS2_ENCPORT);

	// Activate the axis
	AxisControl(AXIS1_NO, ON, AXIS2_NO, ON);

	// Set movements parameter
	Acc(AXIS1_NO,50,AXIS2_NO,50);	// Set acceleration to 50% of VELMAX
	Dec(AXIS1_NO,50,AXIS2_NO,50);	// Set deceleration to 50% of RAMPMIN
	Vel(AXIS1_NO,50,AXIS2_NO,50);	// Set velocity to 50% of RAMPMIN

	while(1)
	{
		// Do xx turns: direction * distance travelled per revolution * turns p.e. 1 * 36000 1/100 degree * 100 turns
		AxisPosAbsStart(AXIS1_NO,direction*ECi30_FEEDDIST*turns,AXIS2_NO,direction*ECi30_FEEDDIST*turns);
		AxisWaitReached(AXIS1_NO,AXIS2_NO);
		direction = direction*-1;
	}

    return(0);
}


long setupECi30_SC_Hall_Inc(long axisNo, long encPort)
{
	// Encoder setup
	sdkSetupIncEncoder(			axisNo,
								encPort,
								ECi30_ENCRES,
								ECi30_ENC_LATCHTYPE,
								ECi30_ENC_LATCHPARAM,
								ECi30_ENC_LATCHSLOPE
								);
	// Amplifier setup
	sdkSetupAmpHallPmsmMotor(	axisNo,
								ECi30_CONTROLMODE,
								ECi30_POLEPAIRS,
								ECi30_MAXCUR,
								ECi30_ENCRES,
								ECi30_MAX_RPM,
								-1
								);

	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								ECi30_CURKPROP,
								ECi30_CURKINT,
								ECi30_CURKILIM
								);
	// Velocity control setup - not used at the moment
	sdkSetupVelocityPIControl( 	axisNo,
								ECi30_VELKPROP,
								ECi30_VELKINT,
								ECi30_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								ECi30_VELRES,
								ECi30_MAX_RPM,
								ECi30_RAMPTYPE,
								ECi30_RAMPMIN,
								ECi30_JERKMIN,
								ECi30_POSERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		axisNo,
								ECi30_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									ECi30_KPROP,
									ECi30_KINT,
									ECi30_KDER,
									ECi30_KILIM,
									ECi30_KILIMTIME,
									ECi30_BANDWIDTH,
									ECi30_FFVEL,
									ECi30_KFFACC,
									ECi30_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								ECi30_POSENCREV,
								ECi30_POSENCQC,
								ECi30_POSFACT_Z,
								ECi30_POSFACT_N,
								ECi30_FEEDREV,
								ECi30_FEEDDIST
								);

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,ECi30_CONTCUR, ECi30_THERMAL_TIME);

	return(1);
}

//$X {KPROP,1,1,0,-1,0,-1,0,(-1),-1},0x2300,12,0
//$X {KDER,1,1,0,-1,0,-1,0,(-1),-1},0x2300,13,0
//$X {KINT,1,1,0,-1,0,-1,0,(-1),-1},0x2300,14,0
