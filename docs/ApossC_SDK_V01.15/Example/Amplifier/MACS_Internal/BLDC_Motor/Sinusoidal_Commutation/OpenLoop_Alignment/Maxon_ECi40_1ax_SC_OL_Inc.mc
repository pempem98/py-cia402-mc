/**
*	@brief		Program for controlling a single ECi40 motor from Maxon.
*	@detail		This program controls an ECi40 motor from maxon. An incremental encoder with a resolution of 1024 qc is used. \n
*				The purpose of this program is to show how to set up a motor, an encoder and the axis using the SDK. \n
*				An openloop alignment is performed. This means that the motor must perform an alignment run before the first farht is started.\n
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

#define C_AXIS1					0						// Axis module number
#define C_AXIS1_ENCPORT 		0						// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
#define C_AXIS1_ENCRES 			1024					// Resolution of the encoder for position feed back in increments (quadcounts)
#define C_AXIS1_ENC_LATCHTYPE 	1						// Defines the latch type: Digital input
#define C_AXIS1_ENC_LATCHPARAM 	1						// Latch on digital input 1
#define C_AXIS1_ENC_LATCHSLOPE 	HWLATCH_SLOPE_RISING	// Defines the slope of the trigger signal (Default 1)

#define C_MOTOR1_CONTROLMODE	HWAMP_MODE_POS_VEL_CUR	// Define control typ
#define C_MOTOR1_POLEPAIRS		8						// Number of pole pairs
#define C_MOTOR1_MAXCUR			8670					// Maximal current allowed in mA
#define C_MOTOR1_MAX_RPM		7740					// Maximum velocity in RPM
#define C_MOTOR1_CURKPROP		320						// Proportional factor of current controller
#define C_MOTOR1_CURKINT		90						// Integral factor of current controller
#define C_MOTOR1_CURKILIM		32767					// Integral limit of current controller
#define C_MOTOR1_VELKPROP		2000					// Proportional factor of velocity controller
#define C_MOTOR1_VELKINT		250						// Integral factor of velocity controller
#define C_MOTOR1_VELKILIM		1000					// Integral limit of velocity controller

#define	C_MOTOR1_BRUSHLESS		1						// Mode of aligment: For brushless motors
#define C_MOTOR1_ALIGN_CUR		2000					// Current for the alignment function in mA

#define C_AXIS1_VELRES			1000					// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define C_AXIS1_RAMPTYPE		RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define C_AXIS1_RAMPMIN			1000					// Maximum acceleration
#define C_AXIS1_JERKMIN			100						// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS1_TRACKERR		C_AXIS1_ENCRES			// Maximal track/ position error allowed in qc

#define C_AXIS1_DIRECTION		1						// User units have normal orientation. Increasing encoder values result in increasing user positions.

#define	C_AXIS1_KPROP			80						// Proportional value for PID position control loop
#define	C_AXIS1_KINT			0						// Integral value for PID position control loop
#define	C_AXIS1_KDER			600						// Derivative value for PID position control loop
#define	C_AXIS1_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	C_AXIS1_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	C_AXIS1_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	C_AXIS1_FFVEL			1000					// Velocity Feed forward
#define	C_AXIS1_KFFAC			0						// Acceleration Feed forward
#define	C_AXIS1_KFFDEC			0						// Deceleration Feed Forward

#define	C_AXIS1_POSENCREV		1						// Number of revolutions of the motor
#define	C_AXIS1_POSENCQC		C_AXIS1_ENCRES			// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS1_POSFACT_Z		1						// Number of revolutions of the input shaft
#define	C_AXIS1_POSFACT_N		1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS1_FEEDREV			1						// Number of revolutions of the gear box output shaft
#define	C_AXIS1_FEEDDIST		C_AXIS1_ENCRES			// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

long main(void)
{
	long checkMotorAlignment;

	// Encoder setup
	sdkSetupIncEncoder(			C_AXIS1,
								C_AXIS1_ENCPORT,
								C_AXIS1_ENCRES,
								C_AXIS1_ENC_LATCHTYPE,
								C_AXIS1_ENC_LATCHPARAM,
								C_AXIS1_ENC_LATCHSLOPE
								);
	// Amplifier setup
	sdkSetupAmpPmsmMotor(		C_AXIS1,
								C_MOTOR1_CONTROLMODE,
								C_MOTOR1_POLEPAIRS,
								C_MOTOR1_MAXCUR,
								C_AXIS1_ENCRES,
								C_MOTOR1_MAX_RPM
								);
	// Current control setup
	sdkSetupCurrentPIControl( 	C_AXIS1,
								C_MOTOR1_CURKPROP,
								C_MOTOR1_CURKINT,
								C_MOTOR1_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	C_AXIS1,
								C_MOTOR1_VELKPROP,
								C_MOTOR1_VELKINT,
								C_MOTOR1_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	C_AXIS1,
								C_AXIS1_VELRES,
								C_MOTOR1_MAX_RPM,
								C_AXIS1_RAMPTYPE,
								C_AXIS1_RAMPMIN,
								C_AXIS1_JERKMIN,
								C_AXIS1_TRACKERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		C_AXIS1,
								C_AXIS1_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS1,
									C_AXIS1_KPROP,
									C_AXIS1_KINT,
									C_AXIS1_KDER,
									C_AXIS1_KILIM,
									C_AXIS1_KILIMTIME,
									C_AXIS1_BANDWIDTH,
									C_AXIS1_FFVEL,
									C_AXIS1_KFFAC,
									C_AXIS1_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS1,
								C_AXIS1_POSENCREV,
								C_AXIS1_POSENCQC,
								C_AXIS1_POSFACT_Z,
								C_AXIS1_POSFACT_N,
								C_AXIS1_FEEDREV,
								C_AXIS1_FEEDDIST
								);
	// Motor Alignment
	checkMotorAlignment =	sdkMotorAlignment(
							C_AXIS1,
							C_MOTOR1_BRUSHLESS,
							C_MOTOR1_MAXCUR,
							C_MOTOR1_ALIGN_CUR);

	if(checkMotorAlignment<0)
	{
		print("Sorry alignement didn't work we exit");
		Exit(0);
	}

	// Activate the axis
	AxisControl(C_AXIS1, ON);

	// Start an endless movement with continuous move control
	sdkStartContinuousMove(C_AXIS1, 1000, 100);

while(1)
	{
		// Print axis information all 500 ms
		sdkInfoPrintAxesPos();
		Delay(500);
	}
    return(0);
}
