/**
*	@brief		This test program shows the use of an EPOS4 in csp mode as CAN slave.
*	@detail		An EPOS4 is set up in csp mode. For this, the pdo must be defined correctly
*				and the respective modules must be assigned. Afterwards a generic Epos4
*				homing is called. If this is successfully executed, a small example
*				application starts. The Epos4 must already be configured via the EposStudio.
*
*	$Revision: 275 $
*
*	@example 	CAN_1Ax_EPOS4-Test_csp.mc
*
*/
#include "..\..\..\SDK\SDK_ApossC.mc"

// EPOS4 Settings
#define C_AXIS1				0				// Axis module number
#define C_DRIVE_BUSID1		1				// The CAN drive busId
#define C_PDO_NUMBER		1				// Used PDO number

#define C_AXISPOLARITY		0				// Definition of the polarity 0: Normal, 1: Inverse

// Encoder settings & axis user units (MACS)
#define C_AXIS_ENCRES 			4*4096						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS_POSENCQC			C_AXIS_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS_FEEDDIST			C_AXIS_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS_MAX_RPM			4000					// Maximum velocity in RPM
#define C_AXIS_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS_RAMPTYPE			RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define C_AXIS_RAMPMIN			800						// Maximum acceleration
#define C_AXIS_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS_TRACKERR			0						// There is also a following error on EPOS4, could be set to zero on the MACS

// Axis MACS control loop settings
// the position controller of the MACS control is active, but the values are all set to 0.
#define	C_AXIS_KPROP			0
#define	C_AXIS_KINT				0
#define	C_AXIS_KDER				0
#define	C_AXIS_KILIM			0
#define	C_AXIS_KILIMTIME		0
#define	C_AXIS_BANDWIDTH		1000
#define	C_AXIS_FFVEL			1000
#define	C_AXIS_KFFAC			0
#define	C_AXIS_KFFDEC			0

long main(void) {

    long i, homingState=0, retval;

	print("-----------------------------------------------------------");
	print(" Test application CANopen Master with 1 EPOS4 drive");
	print("-----------------------------------------------------------");

	ErrorClear();
	AmpErrorClear(C_AXIS1);

	InterruptSetup(ERROR, ErrorHandler);

	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------

	if(GLB_PARAM(CANBAUD)!=88)
	{
		print("Set new Baudrate and save global parameters");
		// Set Baudrate of CAN 1 & CAN 2 to 1MBaud
		GLB_PARAM(CANBAUD)=88;
		CanOpenRestart();
		Save(GLBPARS);
	}


	//while(1);
	// Cycle time for sending SYNC telegrams on the CAN bus - must be set before calling the sdkEpos4_SetupCanSdoParam() function.
	GLB_PARAM(CANSYNCTIMER)= 1;

	// set all slaves to PREOPERATIONAL by sending an NMT.
	SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 0;

	// initialising maxon drives
	sdkEpos4_SetupCanSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXISPOLARITY, EPOS4_OP_CSP);

	// setup CANopen bus module for csp mode
	sdkEpos4_SetupCanBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, EPOS4_OP_CSP);

	// setup virtual amplifier for csp mode
	sdkEpos4_SetupCanVirtAmp(C_AXIS1, C_AXIS_MAX_RPM, EPOS4_OP_CSP);

	// setup irtual counter for csp mode
	sdkEpos4_SetupCanVirtCntin(C_AXIS1, EPOS4_OP_CSP);

	// start all slaves commanding them into OPERATIONAL.
	SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 1;

	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	C_AXIS1,
								C_AXIS_VELRES,
								C_AXIS_MAX_RPM,
								C_AXIS_RAMPTYPE,
								C_AXIS_RAMPMIN,
								C_AXIS_JERKMIN,
								C_AXIS_TRACKERR
								);

	// Definition of the user units
	sdkSetupAxisUserUnits(		C_AXIS1,
								C_AXIS_POSENCREV,
								C_AXIS_POSENCQC,
								C_AXIS_POSFACT_Z,
								C_AXIS_POSFACT_N,
								C_AXIS_FEEDREV,
								C_AXIS_FEEDDIST
								);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	C_AXIS1,
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

	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	// Homing setup
	print("\nEPOS4 Homing:");
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_METHOD, 				0,   37);	// 0x6098 Set homing method to “-4" : Homing Method -4 (Current Threshold Negative Speed).”
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_SPEEDS, 				1,   20);    						// Homing Speed / Speed for switch speed [velocity units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_SPEEDS, 				2,   20);    						// Homing Speed / Speed for zero search [velocity units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOMING_ACCELERATION, 		0,   20);    						// Homing acceleration [acceleration units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOME_OFFSET_MOVE_DISTANCE, 	0,   6400);   						// Home offset move distance [position units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_HOME_POSITION, 				0,   0);   							// Home position [position units]
	SdoWrite( C_DRIVE_BUSID1, EPOS4_CURRENT_THRESHOLD_FOR_HOMING_MODE, 0,   1500);  				// Current threshold for homing mode [mA]

	// Homing statemachine
	retval=0;

	while(! retval)
	{
		retval = sdkEpos4_AxisHomingStart(C_AXIS1, C_DRIVE_BUSID1, EPOS4_OP_CSP, homingState);

		// Homing error - Exit programm
		if(retval==-1)
		{
			print("Exit programm");
			Exit(0);
		}
	}

	print("");
	print("-----------------------------------------------------------");
	print("                Movement in CSP Mode                       ");
	print("----------------------------------------------------------- \n");

	Vel(C_AXIS1, 50);
	Acc(C_AXIS1, 30);
	Dec(C_AXIS1, 30);

	AxisControl(C_AXIS1, ON);

	for(i=10;i>=0;i--)
	{
		print("Start, move to target position");
		AxisPosAbsStart(C_AXIS1, 20000);

		AxisWaitReached(C_AXIS1);
		print("Target position is reached \n");
		print("Start, back to start position");

		AxisPosAbsStart(C_AXIS1, 0);
		AxisWaitReached(C_AXIS1);
		print("Start position is reached");
		print(i, " repetitions to go \n");
	}

	AxisControl(C_AXIS1, OFF);

	print("Program done, Axis OFF ");
    return(0);
}

void ErrorHandler(void)
{
	long axeNbr 	= ErrorAxis();
	long errNbr		= ErrorNo();
	long errInfoNbr	= ErrorInfo();
	long eposErr, sdoAbortCode;

    AxisControl(AXALL,OFF);

  	switch(errNbr)
	{
		case F_AMP:		if(	axeNbr==C_AXIS1)
						{
							eposErr = SdoRead(C_DRIVE_BUSID1,EPOS4_ERROR_CODE,0x00);
							printf("Error Axis: %d, Epos4 Error 0x%lX: ", axeNbr , eposErr);
							sdkEpos4_PrintErrorDescription(eposErr);
							print();
							AmpErrorClear(axeNbr); // Clear error on EPOS4
						}
						else
						{
							print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
						}
						break;

		case F_CANIO:	print("ErrorNo: ",errNbr," info: ",errInfoNbr);
						sdoAbortCode = SYS_PROCESS(SYS_CANOM_SDOABORT);
						printf("SDO Abort Code 0x%lX: ", sdoAbortCode );
						sdkErrorPrint_SdoErrorDescription(sdoAbortCode);
						print();
						print("Check Can baudrate & Can bus id");
						break;

		default:		print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
	}
	ErrorClear();
	print("");	print(" There is no error handlig → Exit()");
	Exit(0);
}