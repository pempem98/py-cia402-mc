/**
*	@brief		This program shows the control of a BLDC motor with encoder
*	@detail		This example shows the setup of an ECi30 motor with an encoder in sinusoidal commutated mode (SC).\n
*				The incremental encoder is used for position control.
*				For this purpose the following must be configured: \n
*				- Encoder \n
*				- Amplifier \n
*				- ControllerMode and controller (position, current) \n
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

// Axe settings
#define EC_i30_ENCRES 			4*4096				// Resolution of the encoder for position feed back in increments (quadcounts)
#define EC_i30_ENC_LATCHTYPE 	0					// Default latchTyp Encoder Z signal
#define EC_i30_ENC_LATCHPARAM 	0					// Not used for Encoder Z Latch
#define EC_i30_ENC_LATCHSLOPE 	HWLATCH_SLOPE_RISING// Defines the slope of the trigger signal (Default 1)

#define EC_i30_CONTROLMODE		HWAMP_MODE_POS_CUR			// Define control typ
#define EC_i30_HALL_ALIGNMENT	HALL_ALIGNMENT_120DEGREE	// Hall alignment 120° standard
#define EC_i30_POLEPAIRS		4							// Number of pole pairs
#define EC_i30_CONTCUR			2280						// Nomial continious current allowed in mA
#define EC_i30_MAXCUR			EC_i30_CONTCUR*1.1			// Maximal current allowed in mA
#define EC_i30_THERMAL_TIME		27600						// Thermal time constant of the winding

#define EC_i30_MAX_RPM			6000					// Maximum velocity in RPM
#define EC_i30_CURKPROP			1500					// Proportional factor of current controller
#define EC_i30_CURKINT			150						// Integral factor of current controller
#define EC_i30_CURKILIM			32000					// Integral limit of current controller

// not used in this example → HWAMP_MODE_POS_CUR
#define EC_i30_VELKPROP			0						// Proportional factor of velocity controller
#define EC_i30_VELKINT			0						// Integral factor of velocity controller
#define EC_i30_VELKILIM			1000					// Integral limit of velocity controller

#define EC_i30_VELRES			100					    // Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands
#define EC_i30_RAMPTYPE			RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define EC_i30_RAMPMIN			1000					// Maximum acceleration ramp (ms) from 0 to EC_i30_MAX_RPM
#define EC_i30_JERKMIN			500						// Minimum time (ms) required before reaching the maximum acceleration
#define EC_i30_POSERR			16384					// Maximal track/ position error allowed in qc
#define EC_i30_DIRECTION		1						// User units have normal orientation. Increasing encoder values result in increasing user positions.

#define	EC_i30_KPROP			100						// Proportional value for PID position control loop
#define	EC_i30_KINT				1						// Integral value for PID position control loop
#define	EC_i30_KDER				180						// Derivative value for PID position control loop
#define	EC_i30_KILIM			1000					// Limit value for the integral sum of the PID position control loop
#define	EC_i30_KILIMTIME		0						// Time used to increase or decrease the integral limit
#define	EC_i30_BANDWIDTH		1000					// Bandwidth within which the PID filter is active. 1000 equals to 100% velocity setpoint
#define	EC_i30_FFVEL			0						// Velocity Feed forward
#define	EC_i30_KFFACC			50						// Acceleration Feed forward
#define	EC_i30_KFFDEC			50						// Deceleration Feed Forward

#define	EC_i30_POSENCREV		1						// Number of revolutions of the motor
#define	EC_i30_POSENCQC			EC_i30_ENCRES			// Number of quadcounts in POSENCREV revolutions
#define	EC_i30_POSFACT_Z		1						// Number of revolutions of the input shaft
#define	EC_i30_POSFACT_N		1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	EC_i30_FEEDREV			1						// Number of revolutions of the gear box output shaft
#define	EC_i30_FEEDDIST			36000					// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft


// Function declaration
void ErrorHandler(void);
long setupEC_i30_SC_Hall_Inc(long axisNo, long encPort);



/////////////////////////////////////////////////////////
long main(void)
{

	long MoveDistance = 10*36000; // we do 10 rotation
	long MoveState = 1;
	long CycleNo = 0;

	print(" Run motor config example for axis: ",AXIS1_NO);

	ErrorClear();
	InterruptSetup(ERROR, ErrorHandler);

	// Setup axis & amplifier
	setupEC_i30_SC_Hall_Inc(AXIS1_NO,AXIS1_ENCPORT);

	// Activate the axis
	AxisControl(AXIS1_NO, ON);

	// Set 50% velocity and 50% acceleration/deceleration
	Vel(AXIS1_NO, 50);
	Acc(AXIS1_NO, 50);
	Dec(AXIS1_NO, 50);

	// record some process data (internal scope)
	RecordIndex(AXE_PROCESS_INDEX(AXIS1_NO,REG_COMPOS), AXE_PROCESS_INDEX(AXIS1_NO,REG_ACTPOS), AXE_PROCESS_INDEX(AXIS1_NO,REG_TRACKERR), HWAMP_PROCESS_INDEX(AXIS1_NO, PO_HWAMP_CURRENT) );
	RecordStart(0);   // start recording

	DefOrigin(AXIS1_NO); 		// Set zero at actual position

	AxisPosAbsStart(AXIS1_NO, MoveDistance);  // Move to "MoveDistance"


	while (1) {

	   if (MoveState == 1) {
		  if (((AxisStatus(AXIS1_NO)) & AXSTAT_POSREACHED) != 0) {
			 print(" 1 cpos: ",Cpos(AXIS1_NO),"  apos: ",Apos(AXIS1_NO) );
			 Delay(1000);
			 AxisPosAbsStart(AXIS1_NO, 0);
			 MoveState = 2;
		  }
	   }
	   else if (MoveState == 2) {
		  if (((AxisStatus(AXIS1_NO)) & AXSTAT_POSREACHED) != 0) {
			 print(" 2 cpos: ",Cpos(AXIS1_NO),"  apos: ",Apos(AXIS1_NO) );
			 Delay(1000);
			 AxisPosAbsStart(AXIS1_NO, MoveDistance);
			 MoveState = 1;
			 CycleNo++;
			 if (CycleNo >= 2) {
				RecordStop(0, 0);    // Stop recording
			 }
		  }
	   }

	}
    return(0);
}


/////////////////////////////////////////////////////////////////
void ErrorHandler(void) {

   print(" !!!!! ERROR: ",ErrorNo(),"   ax: ",ErrorAxis() );
   if (ErrorNo() == 89)  { // CAN Error
      printf("SDO Abort Code 0x%lx", SYS_PROCESS(SYS_CANOM_SDOABORT) );
   }
   else if (ErrorNo() == 8) {
      print("track error limit reached");            // see:  EC_i30_POSERR
   }
   ErrorClear();

   AxisControl(AXIS1_NO,ON);

}

/////////////////////////////////////////////////////////////////
long setupEC_i30_SC_Hall_Inc(long axisNo, long encPort)
{
	// Encoder setup
	sdkSetupIncEncoder(			axisNo,
								encPort,
								EC_i30_ENCRES,
								EC_i30_ENC_LATCHTYPE,
								EC_i30_ENC_LATCHPARAM,
								EC_i30_ENC_LATCHSLOPE
								);
	// Amplifier setup
	sdkSetupAmpHallPmsmMotor(	axisNo,
								EC_i30_CONTROLMODE,
								EC_i30_POLEPAIRS,
								EC_i30_MAXCUR,
								EC_i30_ENCRES,
								EC_i30_MAX_RPM,
								-1
								);

	// Current control setup
	sdkSetupCurrentPIControl( 	axisNo,
								EC_i30_CURKPROP,
								EC_i30_CURKINT,
								EC_i30_CURKILIM
								);
	// Velocity control setup - not used at the moment
	sdkSetupVelocityPIControl( 	axisNo,
								EC_i30_VELKPROP,
								EC_i30_VELKINT,
								EC_i30_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axisNo,
								EC_i30_VELRES,
								EC_i30_MAX_RPM,
								EC_i30_RAMPTYPE,
								EC_i30_RAMPMIN,
								EC_i30_JERKMIN,
								EC_i30_POSERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection( 		axisNo,
								EC_i30_DIRECTION);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axisNo,
									EC_i30_KPROP,
									EC_i30_KINT,
									EC_i30_KDER,
									EC_i30_KILIM,
									EC_i30_KILIMTIME,
									EC_i30_BANDWIDTH,
									EC_i30_FFVEL,
									EC_i30_KFFACC,
									EC_i30_KFFDEC
									);
	// Definition of the user units
	sdkSetupAxisUserUnits(		axisNo,
								EC_i30_POSENCREV,
								EC_i30_POSENCQC,
								EC_i30_POSFACT_Z,
								EC_i30_POSFACT_N,
								EC_i30_FEEDREV,
								EC_i30_FEEDDIST
								);

	// Setup virtual I2T
	sdkSetupVirtualI2T(axisNo,EC_i30_CONTCUR, EC_i30_THERMAL_TIME);

	return(1);
}

//$X {KPROP,1,1,0,-1,0,-1,0,(-1),-1},0x2300,12,0
//$X {KDER,1,1,0,-1,0,-1,0,(-1),-1},0x2300,13,0
//$X {KINT,1,1,0,-1,0,-1,0,(-1),-1},0x2300,14,0
