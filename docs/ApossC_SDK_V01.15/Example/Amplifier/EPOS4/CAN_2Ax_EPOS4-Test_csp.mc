/**
*	@brief		This test program shows the use of two EPOS4 in csp mode as CAN slave.
*	@detail		An EPOS4 is set up in csp mode. For this, the pdo must be defined correctly
*				and the respective modules must be assigned. Afterwards a generic Epos4
*				homing is called. If this is successfully executed, a small example
*				application starts. The Epos4 must already be configured via the EposStudio.
*
*	$Revision: 275 $
*
*	@example 	CAN_2Ax_EPOS4-Test_csp.mc
*
*/
#include "..\..\..\SDK\SDK_ApossC.mc"

// Parameters for the SDK function
#define C_AXIS1			0				// Axis module number
#define C_DRIVE_BUSID1	1				// The CAN drive busId

#define C_AXIS2			1				// Axis module number
#define C_DRIVE_BUSID2	2				// The CAN drive busId

#define C_PDO_NUMBER	1				// Used PDO number

#define C_AXISPOLARITY	0				// Definition of the polarity 0: Normal, 1: Inverse

// Encoder settings & axis user units (MACS)
#define C_AXIS_ENCRES 			4*1600						// Resolution of the encoder for position feed back in increments (quadcounts)
#define	C_AXIS_POSENCREV		1							// Number of revolutions of the motor
#define	C_AXIS_POSENCQC			C_AXIS_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	C_AXIS_POSFACT_Z		1							// Number of revolutions of the input shaft
#define	C_AXIS_POSFACT_N		1							// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	C_AXIS_FEEDREV			1							// Number of revolutions of the gear box output shaft
#define	C_AXIS_FEEDDIST			C_AXIS_ENCRES				// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter
#define C_AXIS_MAX_RPM			2000					// Maximum velocity in RPM
#define C_AXIS_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define C_AXIS_RAMPTYPE			RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define C_AXIS_RAMPMIN			800						// Maximum acceleration
#define C_AXIS_JERKMIN			1000					// Minimum time (ms) required before reaching the maximum acceleration
#define C_AXIS_TRACKERR			0						// There is also a following error on EPOS4, could be set to zero on the MACS

// Axis MACS control loop settings
// MACS position control is not used
#define	C_AXIS_KPROP			0
#define	C_AXIS_KINT				0
#define	C_AXIS_KDER				0
#define	C_AXIS_KILIM			0
#define	C_AXIS_KILIMTIME		0
#define	C_AXIS_BANDWIDTH		1000
#define	C_AXIS_FFVEL			1000
#define	C_AXIS_KFFAC			0
#define	C_AXIS_KFFDEC			0

// Set the bus ids of the all axis in the network
long axis[] = {C_AXIS1, C_AXIS2};
long busIds[] = {C_DRIVE_BUSID1, C_DRIVE_BUSID2};
long numberOfAxis = arraylen(busIds);


long main(void) {

    long i, homingState=0, retval, axisIndex;

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

	// set all slaves to PREOPERATIONAL by sending an NMT.
	SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 0;

	for(axisIndex = 0; axisIndex < numberOfAxis; axisIndex++ )
	{
		// initialising maxon drives
		sdkEpos4_SetupCanSdoParam(busIds[axisIndex], C_PDO_NUMBER, C_AXISPOLARITY, EPOS4_OP_CSP);

		// setup CANopen bus module for csp mode
		sdkEpos4_SetupCanBusModule(axis[axisIndex], busIds[axisIndex], C_PDO_NUMBER, EPOS4_OP_CSP);

		// setup virtual amplifier for csp mode
		sdkEpos4_SetupCanVirtAmp(axis[axisIndex], C_AXIS_MAX_RPM, EPOS4_OP_CSP);

		// setup irtual counter for csp mode
		sdkEpos4_SetupCanVirtCntin(axis[axisIndex], EPOS4_OP_CSP);

		// Movement parameters for the axis
		sdkSetupAxisMovementParam(	axis[axisIndex],
									C_AXIS_VELRES,
									C_AXIS_MAX_RPM,
									C_AXIS_RAMPTYPE,
									C_AXIS_RAMPMIN,
									C_AXIS_JERKMIN,
									C_AXIS_TRACKERR
									);

		// Definition of the user units
		sdkSetupAxisUserUnits(		axis[axisIndex],
									C_AXIS_POSENCREV,
									C_AXIS_POSENCQC,
									C_AXIS_POSFACT_Z,
									C_AXIS_POSFACT_N,
									C_AXIS_FEEDREV,
									C_AXIS_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	axis[axisIndex],
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
	}

	// start all slaves commanding them into OPERATIONAL.
	SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 1;

	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	for(axisIndex = 0; axisIndex < numberOfAxis; axisIndex++ )
	{
		// Homing setup
		print("\nEPOS4 Homing:");
		SdoWrite( busIds[axisIndex], EPOS4_HOMING_METHOD, 				0,   EPOS4_HOMING_CURRENT_N_SPEED);	// 0x6098 Set homing method to “-4" : Homing Method -4 (Current Threshold Negative Speed).”
		SdoWrite( busIds[axisIndex], EPOS4_HOMING_SPEEDS, 				1,   20);    						// Homing Speed / Speed for switch speed [velocity units]
		SdoWrite( busIds[axisIndex], EPOS4_HOMING_SPEEDS, 				2,   20);    						// Homing Speed / Speed for zero search [velocity units]
		SdoWrite( busIds[axisIndex], EPOS4_HOMING_ACCELERATION, 		0,   20);    						// Homing acceleration [acceleration units]
		SdoWrite( busIds[axisIndex], EPOS4_HOME_OFFSET_MOVE_DISTANCE, 	0,   6400);   						// Home offset move distance [position units]
		SdoWrite( busIds[axisIndex], EPOS4_HOME_POSITION, 				0,   0);   							// Home position [position units]
		SdoWrite( busIds[axisIndex], EPOS4_CURRENT_THRESHOLD_FOR_HOMING_MODE, 0,   1500);  				// Current threshold for homing mode [mA]

		// Disable MACS trackerror for homing
		AXE_PARAM(C_AXIS2,POSERR)=2000000000;
	}

	// Homing statemachine
	retval=0;

	while(! retval)
	{
		retval = 1;
		for(axisIndex = 0; axisIndex < numberOfAxis; axisIndex++ )
		{
			retval = retval && sdkEpos4_AxisHomingStart(axis[axisIndex], busIds[axisIndex], EPOS4_OP_CSP, homingState);

			// Homing error - Exit programm
			if(retval==-1)
			{
				print("Exit programm");
				Exit(0);
			}
		}
	}

	// Enable MACS trackerror for homing
	AXE_PARAM(C_AXIS1,POSERR)=C_AXIS_TRACKERR;
	AXE_PARAM(C_AXIS2,POSERR)=C_AXIS_TRACKERR;

	print("");
	print("-----------------------------------------------------------");
	print("                Movement in CSP Mode                       ");
	print("----------------------------------------------------------- \n");

	Vel(C_AXIS1, 50,C_AXIS2, 50);
	Acc(C_AXIS1, 30,C_AXIS2, 50);
	Dec(C_AXIS1, 30,C_AXIS2, 50);

	AxisControl(C_AXIS1, ON, C_AXIS2, ON);

	for(i=10;i>=0;i--)
	{
		print("Start, move to target position");
		AxisPosAbsStart(C_AXIS1, 20000, C_AXIS2, 20000);

		AxisWaitReached(C_AXIS1,C_AXIS2);
		print("Target position is reached \n");
		print("Start, back to start position");

		AxisPosAbsStart(C_AXIS1, 0,C_AXIS2, 0);
		AxisWaitReached(C_AXIS1,C_AXIS2);
		print("Start position is reached");
		print(i, " repetitions to go \n");
	}

	AxisControl(C_AXIS1, OFF,C_AXIS2, OFF);

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
		case F_AMP:		if(	axeNbr==C_AXIS1 || axeNbr==C_AXIS2 )
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