/**
*	@brief		This program shows the control of a BLDC motor with SSI encoder
*	@detail		This example shows the setup of an ECi52 motor with SSI encoder.\n
*				Since an SSI encoder is an absolute encoder, an absolute alignment can be performed.\n
*				No additional Hall sensor is required for this. In a first step the alignment offset is determined openloop.\n
*				This offset is then returned and can be entered in the macro ECI52_ALIGN_POS. If this is 0 the open loop alignment is executed again.\n
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

// Axis settings
#define ECI52_ENCRES 			4096				// Resolution of the encoder for position feed back in increments (quadcounts)
#define	ECI52_SSI_CLK_FREQ		2000000					// Clock frequency of the SSI encoder (Hz)
#define	ECI52_FAST_UPDATE		1						// fastUpdate @b 1: The encoder is updated with the current controller update rate (24 kHz on MiniMACS6)
#define	ECI52_DATLEN			12						// Databit length of endat position
#define	ECI52_IS_BINARY			0						// 1 if data coding is binary otherwise it is gray coded.
#define	ECI52_I_POS				0						// Define an indexposition

#define	ECI52_BRUSHLESS		1						// Mode of aligment: For brushless motors
#define ECI52_ALIGN_CUR		3000					// Current for the alignment function in mA
#define ECI52_ALIGN_POS		463						// Offset of the encoder position to the electrical position

#define ECI52_CONTROLMODE	HWAMP_MODE_POS_CUR		// Define control typ
#define ECI52_POLEPAIRS		8						// Number of pole pairs
#define ECI52_ELPOL			-1						// Always -1 with a maxon motor
#define ECI52_CONTCUR		9360					// Nomial continious current allowed in mA
#define ECI52_MAXCUR		ECI52_CONTCUR*1.25		// Maximal current allowed in mA
#define ECI52_THERMAL_TIME	12800					// Thermal time constant of the winding

#define ECI52_MAX_RPM		2000					// Maximum velocity in RPM
#define ECI52_CURKPROP		1100						// Proportional factor of current controller
#define ECI52_CURKINT		150						// Integral factor of current controller
#define ECI52_CURKILIM		15000					// Integral limit of current controller
// not used in this example → HWAMP_MODE_POS_CUR
#define ECI52_VELKPROP		0						// Proportional factor of velocity controller
#define ECI52_VELKINT		0						// Integral factor of velocity controller
#define ECI52_VELKILIM		0						// Integral limit of velocity controller

#define ECI52_VELRES		100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define ECI52_RAMPTYPE		RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define ECI52_RAMPMIN		1000					// Maximum acceleration
#define ECI52_JERKMIN		100						// Minimum time (ms) required before reaching the maximum acceleration
#define ECI52_POSERR		2000					// Minimum time (ms) required before reaching the maximum acceleration
#define ECI52_DIRECTION		1						// User units have normal orientation. Increasing encoder values result in increasing user positions.

#define	ECI52_KPROP			100						// Proportional value for PID position control loop
#define	ECI52_KINT			0						// Integral value for PID position control loop
#define	ECI52_KDER			300						// Derivative value for PID position control loop
#define	ECI52_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	ECI52_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	ECI52_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	ECI52_FFVEL			0						// Velocity Feed forward
#define	ECI52_KFFAC			0						// Acceleration Feed forward
#define	ECI52_KFFDEC		0						// Deceleration Feed Forward

#define	ECI52_POSENCREV		1						// Number of revolutions of the motor
#define	ECI52_POSENCQC		ECI52_ENCRES			// Number of quadcounts in POSENCREV revolutions
#define	ECI52_POSFACT_Z		1						// Number of revolutions of the input shaft
#define	ECI52_POSFACT_N		1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	ECI52_FEEDREV		1						// Number of revolutions of the gear box output shaft
#define	ECI52_FEEDDIST		36000					// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft


// Function delkratation
long setupECi52_SC_SSI(long axisNo, long encPort, long posOffset);


long main(void)
{
	ErrorClear();
	DefOrigin(AXIS1_NO);

	// Setup axis & amplifier
	setupECi52_SC_SSI(AXIS1_NO,AXIS1_ENCPORT,ECI52_ALIGN_POS);

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

long setupECi52_SC_SSI(long axisNo, long encPort, long posOffset)
{
	long checkMotorAlignment;

	// Amplifier setup
	sdkSetupAmpHallPmsmMotor(		axisNo,
								ECI52_CONTROLMODE,
								ECI52_POLEPAIRS,
								ECI52_MAXCUR,
								ECI52_ENCRES,
								ECI52_MAX_RPM,
								-1
								);

	// Encoder setup
	sdkSetupAbsSSIEncoder	(	axisNo,
								encPort,
								ECI52_ENCRES,
								ECI52_SSI_CLK_FREQ,
								ECI52_FAST_UPDATE,
								ECI52_DATLEN,
								ECI52_IS_BINARY,
								ECI52_I_POS
								);
	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								ECI52_CURKPROP,
								ECI52_CURKINT,
								ECI52_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	axisNo,
								ECI52_VELKPROP,
								ECI52_VELKINT,
								ECI52_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								ECI52_VELRES,
								ECI52_MAX_RPM,
								ECI52_RAMPTYPE,
								ECI52_RAMPMIN,
								ECI52_JERKMIN,
								ECI52_POSERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		axisNo,
								ECI52_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									ECI52_KPROP,
									ECI52_KINT,
									ECI52_KDER,
									ECI52_KILIM,
									ECI52_KILIMTIME,
									ECI52_BANDWIDTH,
									ECI52_FFVEL,
									ECI52_KFFAC,
									ECI52_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								ECI52_POSENCREV,
								ECI52_POSENCQC,
								ECI52_POSFACT_Z,
								ECI52_POSFACT_N,
								ECI52_FEEDREV,
								ECI52_FEEDDIST
								);
	// Motor Alignment
	if(posOffset==0)
	{
		print("The first alignment is open loop and is used in commissioning to detect the offset of the encoder position to the electrical position.");
		print("");

		checkMotorAlignment =	sdkMotorAlignment(
								axisNo,
								ECI52_BRUSHLESS,
								ECI52_MAXCUR,
								ECI52_ALIGN_CUR);

		if(checkMotorAlignment<0)
		{
			print("Sorry alignement didn't work we exit");
			Exit(0);
		}
		else
		{
			print("");
			print("Alignement posiston offset: ", HWAMP_PARAM(axisNo, HWAMP_POSEL_OFFSET));
			print("The determined alingment offset can now be entered.");
			print("The value must be specified in the macro: ");
			print("#define ECI52_ALIGN_POS ", HWAMP_PARAM(axisNo, HWAMP_POSEL_OFFSET));
			Exit(0);
		}
	}
	else
	{
	// Motor Alignment
	sdkSetMotorAlignmentOffset( axisNo,
								-1,
								posOffset
						  		);

	}

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,ECI52_CONTCUR, ECI52_THERMAL_TIME);

	return(1);
}

