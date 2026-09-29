/**
*	@brief		Program for controlling four ECi40 motor from Maxon.
*	@detail		This program controls four ECi40 motor from maxon. Four incremental encoder with a resolution of 1024 qc is used. \n
*				The purpose of this program is to show how to set up a motor, an encoder and the axis using the SDK. \n
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
#define C_AXIS2					1						// Axis module number
#define C_AXIS2_ENCPORT 		1						// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
#define C_AXIS3					2						// Axis module number
#define C_AXIS3_ENCPORT 		2						// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual

#define C_AXIS1_DIRECTION		1						// User units have normal orientation. Increasing encoder values result in increasing user positions.
#define C_AXIS2_DIRECTION		-1						// User units are inverted. Increasing encoder values result in decreasing user positions.
#define C_AXIS3_DIRECTION		-1						// User units are inverted. Increasing encoder values result in decreasing user positions.

#define C_AXIS_ENCRES 			1024					// Resolution of the encoder for position feed back in increments (quadcounts)
#define C_AXIS_ENC_LATCHTYPE 	1						// Defines the latch type: Digital input
#define C_AXIS_ENC_LATCHPARAM 	1						// Latch on digital input 1
#define C_AXIS_ENC_LATCHSLOPE 	HWLATCH_SLOPE_RISING	// Defines the slope of the trigger signal (Default 1)

#define C_MOTOR_CONTROLMODE		HWAMP_MODE_POS_VEL_CUR	// Define control typ
#define C_MOTOR_POLEPAIRS		8						// Number of pole pairs
#define C_MOTOR_MAXCUR			8670					// Maximal current allowed in mA
#define C_MOTOR_MAX_RPM			7740					// Maximum velocity in RPM
#define C_MOTOR_CURKPROP		320						// Proportional factor of current controller
#define C_MOTOR_CURKINT			90						// Integral factor of current controller
#define C_MOTOR_CURKILIM		32767					// Integral limit of current controller
#define C_MOTOR_VELKPROP		2000					// Proportional factor of velocity controller
#define C_MOTOR_VELKINT			250						// Integral factor of velocity controller
#define C_MOTOR_VELKILIM		1000					// Integral limit of velocity controller

#define	C_MOTOR_BRUSHLESS		1						// Mode of aligment: For brushless motors
#define C_MOTOR_ALIGN_CUR		2000					// Current for the alignment function in mA

#define C_AXIS_VELRES			1000					// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define C_AXIS_RAMPTYPE			RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define C_AXIS_RAMPMIN			1000					// Maximum acceleration
#define C_AXIS_JERKMIN			100						// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS_TRACKERR			C_AXIS_ENCRES			// Maximal track/ position error allowed in qc

#define	C_AXIS_KPROP			80						// Proportional value for PID position control loop
#define	C_AXIS_KINT				0						// Integral value for PID position control loop
#define	C_AXIS_KDER				600						// Derivative value for PID position control loop
#define	C_AXIS_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	C_AXIS_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	C_AXIS_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	C_AXIS_FFVEL			1000					// Velocity Feed forward
#define	C_AXIS_KFFAC			0						// Acceleration Feed forward
#define	C_AXIS_KFFDEC			0						// Deceleration Feed Forward

#define	C_AXIS_POSENCREV		1						// Number of revolutions of the motor
#define	C_AXIS_POSENCQC			C_AXIS_ENCRES			// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS_POSFACT_Z		1						// Number of revolutions of the input shaft
#define	C_AXIS_POSFACT_N		1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS_FEEDREV			1						// Number of revolutions of the gear box output shaft
#define	C_AXIS_FEEDDIST			C_AXIS_ENCRES			// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft

long main(void)
{
	long i, checkMotorAlignment;

	 // Encoder setup
	sdkSetupIncEncoder(			C_AXIS1,
								C_AXIS1_ENCPORT,
								C_AXIS_ENCRES,
								C_AXIS_ENC_LATCHTYPE,
								C_AXIS_ENC_LATCHPARAM,
								C_AXIS_ENC_LATCHSLOPE
								);

	sdkSetupIncEncoder(			C_AXIS2,
								C_AXIS2_ENCPORT,
								C_AXIS_ENCRES,
								C_AXIS_ENC_LATCHTYPE,
								C_AXIS_ENC_LATCHPARAM,
								C_AXIS_ENC_LATCHSLOPE
								);

	sdkSetupIncEncoder(			C_AXIS3,
								C_AXIS3_ENCPORT,
								C_AXIS_ENCRES,
								C_AXIS_ENC_LATCHTYPE,
								C_AXIS_ENC_LATCHPARAM,
								C_AXIS_ENC_LATCHSLOPE
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		C_AXIS1,
								C_AXIS1_DIRECTION);

	sdkSetupAxisDirection( 		C_AXIS2,
								C_AXIS2_DIRECTION);

	sdkSetupAxisDirection( 		C_AXIS3,
								C_AXIS3_DIRECTION);

	for(i=C_AXIS1;i<=C_AXIS3;i++)
	{
	 // Amplifier setup
	sdkSetupAmpPmsmMotor(		i,
								C_MOTOR_CONTROLMODE,
								C_MOTOR_POLEPAIRS,
								C_MOTOR_MAXCUR,
								C_AXIS_ENCRES,
								C_MOTOR_MAX_RPM
								);
	// Current control setup
	sdkSetupCurrentPIControl( 	i,
								C_MOTOR_CURKPROP,
								C_MOTOR_CURKINT,
								C_MOTOR_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	i,
								C_MOTOR_VELKPROP,
								C_MOTOR_VELKINT,
								C_MOTOR_VELKILIM
								);
	 // Movement parameters for the axis
	sdkSetupAxisMovementParam(	i,
								C_AXIS_VELRES,
								C_MOTOR_MAX_RPM,
								C_AXIS_RAMPTYPE,
								C_AXIS_RAMPMIN,
								C_AXIS_JERKMIN,
								C_AXIS_TRACKERR
								);
	 // Position control setup
	sdkSetupPositionPIDControlExt( 	i,
									C_AXIS_KPROP,
									C_AXIS_KINT,
									C_AXIS_KDER,
									C_AXIS_KILIM,
									C_AXIS_KILIMTIME,
									C_AXIS_BANDWIDTH,
									C_AXIS_FFVEL,
									C_AXIS_KFFAC,
									C_AXIS_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		i,
								C_AXIS_POSENCREV,
								C_AXIS_POSENCQC,
								C_AXIS_POSFACT_Z,
								C_AXIS_POSFACT_N,
								C_AXIS_FEEDREV,
								C_AXIS_FEEDDIST
								);
	}

	// Motor Alignment
	checkMotorAlignment = 		sdkMotorMultiAlignment(
								C_AXIS1,
								3,
								C_MOTOR_BRUSHLESS,
								C_MOTOR_MAXCUR,
								C_MOTOR_ALIGN_CUR);

	if(checkMotorAlignment<0)
	{
		print("Sorry alignement didn't work we exit");
		Exit(0);
	}

	for(i=C_AXIS1;i<=C_AXIS3;i++)
	{
		AxisControl(i, ON);
		// Start an endless movement with continuous move control
		sdkStartContinuousMove(i, 1000, 500);
	}

while(1)
	{
		// Print axis information all 500 ms
		sdkInfoPrintAxesPos();
		Delay(2000);
	}
    return(0);
}//$X {REG_ACTPOS,1,1,0,-1,0,-1,0,(-1),-1},0x2500,1,0
//$X {User Parameter (1),1,1,0,-1,0,-1,0,(-1),-1},0x2201,1,0
//$X {REG_TRACKERR,1,1,0,-1,0,-1,0,(-1),-1},0x2500,6,0
//$X {REG_TRACKERR,1,1,0,-1,0,-1,0,(-1),-1},0x2501,6,0
//$X {REG_TRACKERR,1,1,0,-1,0,-1,0,(-1),-1},0x2502,6,0
//$X {REG_TRACKERR,1,1,0,-1,0,-1,0,(-1),-1},0x2503,6,0
