/**
*	@brief		This program shows the control of a BLDC motor without encoder
*	@detail		This example shows the setup of an EC45  motor without encoder block commutated.\n
*				The Hall signal is used for velocity control.
*				For this purpose the following must be configured: \n
*				- Hall \n
*				- Amplifier \n
*				- ControllerMode and controller (velocity, current) \n
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
#define EC45_CONTROLMODE	HWAMP_MODE_POS_VEL_CUR			// Define control typ, use velocity controller
#define EC45_HALL_ALIGNMENT	HALL_ALIGNMENT_120DEGREE		// Hall alignment 120° standard
#define EC45_HALL_DIRECT	-1								// Hall direction is inverted when using maxon motors
#define EC45_POLEPAIRS		1								// Number of pole pairs
#define EC45_CONTCUR		7940							// Nomial continious current allowed in mA
#define EC45_MAXCUR			EC45_CONTCUR*1					// Maximal current allowed in mA
#define EC45_THERMAL_TIME	31000							// Thermal time constant of the winding

#define EC45_MAX_RPM		9400							// Maximum velocity in RPM
#define EC45_CURKPROP		650								// Proportional factor of current controller
#define EC45_CURKINT		100								// Integral factor of current controller
#define EC45_CURKILIM		18000							// Integral limit of current controller

#define EC45_VELKPROP		1500							// Proportional factor of velocity controller
#define EC45_VELKINT		3								// Integral factor of velocity controller
#define EC45_VELKILIM		3000							// Integral limit of velocity controller

// The speed resolution was set to the maximum rpm value. This means that the speed values can be given as rpm.
#define EC45_VELRES		EC45_MAX_RPM		    // Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands

// Without an encoder signal, slow movements should be avoided. It is therefore advisable not to make a jerklimited run.
#define EC45_RAMPTYPE	RAMPTYPE_TRAPEZ			// Defines the ramptype. Trapez works better here to faster get out of the low velocity
#define EC45_RAMPMIN	300						// Maximum acceleration
#define EC45_JERKMIN	0						// Minimum time (ms) required before reaching the maximum acceleration

// In this speed mode, the following error can be completely ignored. To do this, it is set to 0.
#define EC45_POSERR		0						// Maximal track/ position error allowed in qc - 0→ Disable

// For the drilling application, the motor CCW should proceed as follows
#define EC45_DIRECTION	-1						// User units have normal orientation. Increasing encoder values result in increasing user positions.

// The position controller is switched off in this example. Only the FFVEL is used
#define	EC45_KPROP		0						// Proportional value for PID position control loop
#define	EC45_KINT		0						// Integral value for PID position control loop
#define	EC45_KDER		0						// Derivative value for PID position control loop
#define	EC45_KILIM		1000					// Limit value for the integral sum of the PID position control loop
#define	EC45_KILIMTIME	0						// Time used to increase or decrease the integral limit
#define	EC45_BANDWIDTH	1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	EC45_FFVEL		1000					// Velocity Feed forward
#define	EC45_KFFACC		0						// Acceleration Feed forward
#define	EC45_KFFDEC		0						// Deceleration Feed Forward

// The UserUnits are specified with uu/s. FEEDDIST 60 is used to scale from rps to rpm.
#define	EC45_POSENCREV	1						// Number of revolutions of the motor
#define	EC45_POSENCQC	(EC45_POLEPAIRS*6)		// Number of quadcounts in POSENCREV revolutions
#define	EC45_POSFACT_Z	1						// Number of revolutions of the input shaft
#define	EC45_POSFACT_N	1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	EC45_FEEDREV	1						// Number of revolutions of the gear box output shaft
#define	EC45_FEEDDIST	60						// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft → rps to rpm

// Function delkratation
long setupEC45_BC_Hall_Vel(long axisNo);

long main(void)
{
	long startTime;
	ErrorClear();
	DefOrigin(AXIS1_NO);

	// Setup axis & amplifier
	setupEC45_BC_Hall_Vel(AXIS1_NO);

	// Due to the operation without encoder, the acceleration should be as maximum as possible.
	Acc(AXIS1_NO, EC45_VELRES);
	Dec(AXIS1_NO, EC45_VELRES);

	// The velocity can be specified here in RPM.
	Cvel(AXIS1_NO, EC45_MAX_RPM);

	AxisControl(AXIS1_NO, ON);
	print("Start axis");
	AxisCvelStart(AXIS1_NO);	//start velocity ramp

	while(1)
	{
		if(Time()%500==0)
		{
			// Print velocity information all 500 ms
			print("Actual velocity: ",Avel(AXIS1_NO));
			Delay(1);
			USER_PARAM(1)++;
		}

		if(Avel(AXIS1_NO)>=EC45_MAX_RPM)
		{
			print("Actual velocity: ",Avel(AXIS1_NO));
			print("Max RPM is reached");
			print("Stop cmd after 5000 ms Delay");
			Delay(5000);
			break;
		}
	}

	AxisCvelStop(AXIS1_NO);		//stop motor
	print("Stop axis");
	while(AxisMoving(AXIS1_NO));
	print("Finished");

    return(0);
}


long setupEC45_BC_Hall_Vel(long axisNo)
{
	// Hall setup for using it as encoder
	sdkSetupHallEncoder(axisNo, HALL_ALIGNMENT_120DEGREE, EC45_HALL_DIRECT);

	// Amplifier setup
	HWAMP_PARAM(axisNo, HWAMP_COMMTYPE) = HWAMP_COMMTYPE_BLDC; 			// Set motor type
	HWAMP_PARAM(axisNo, HWAMP_MODE)     = EC45_CONTROLMODE;         	// Set controller priciple
    HWAMP_PARAM(axisNo, HWAMP_POLES)    = EC45_POLEPAIRS;     			// Number of pole pairs
    HWAMP_PARAM(axisNo, HWAMP_MAXCUR)   = EC45_MAXCUR;  				// Max current in mA
	HWAMP_PARAM(axisNo, HWAMP_MAXRPM)   = EC45_MAX_RPM;  				// Given in RPM
	HWAMP_PARAM(axisNo, HWAMP_ENCRES) = EC45_POSENCQC;					//Hall = 6inc x 1 pole pairs

		//Special setup for exact HALL regulation
	HWHALL_PARAM(axisNo, HWHALL_MODE) = HWHALL_MODE_ENABLE_VEL;

	//Configure feedback of amplifier to hall velocity
	HWAMP_PARAM(axisNo, HWAMP_PISRC_ACTVEL) = HWHALL_PROCESS_SRCINDEX(axisNo, PO_HWHALL_VEL);
	HWAMP_PARAM(axisNo, HWAMP_POSDIR) = -1;

	//Smoothen the axis velocity display
	AXE_PROCESS(axisNo, REG_VELSMPT) = 100;

	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								EC45_CURKPROP,
								EC45_CURKINT,
								EC45_CURKILIM
								);
	// Velocity control setup - not used at the moment
	sdkSetupVelocityPIControl( 	axisNo,
								EC45_VELKPROP,
								EC45_VELKINT,
								EC45_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								EC45_VELRES,
								EC45_MAX_RPM,
								EC45_RAMPTYPE,
								EC45_RAMPMIN,
								EC45_JERKMIN,
								EC45_POSERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		axisNo,
								EC45_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									EC45_KPROP,
									EC45_KINT,
									EC45_KDER,
									EC45_KILIM,
									EC45_KILIMTIME,
									EC45_BANDWIDTH,
									EC45_FFVEL,
									EC45_KFFACC,
									EC45_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								EC45_POSENCREV,
								EC45_POSENCQC,
								EC45_POSFACT_Z,
								EC45_POSFACT_N,
								EC45_FEEDREV,
								EC45_FEEDDIST
								);

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,EC45_CONTCUR, EC45_THERMAL_TIME);

	return(1);
}

