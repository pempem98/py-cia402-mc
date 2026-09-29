/**
*	@brief		This test program shows the use a MiniMACS6 as DS402 in cst mode as CAN slave.
*	@detail		A MiniMACS6 is used as DS402 slave. This is operated in the CST mode. For this the DS402
*				application must be loaded on the MiniMACS6. The application must have the minimum version V3.03.
*				The motor setup must be placed in the application of the MiniMACS6. This program is the application for the master.
*				Any MACS controller with at least 4 axes can be used as master. The CAN bus must be set up correctly with two 120 Ohm
*				resistors. The baud rate must be 1 MBaud on both devices. After setting the parameters, a power cycle is required.
*
*				In the program first the correct PDO mapping is made on the slave. Then the axis is set up and a homing is performed.
*
*	$Revision: 271 $
*
*	@example 	CAN_4Ax_MiniMACS6_cst.mc
*
*/
#include "..\..\..\SDK\SDK_ApossC.mc"

// Parameters for the SDK function
#define M_AX_0			0				// Axis module number - DS402 Master
#define M_AX_1			1				// Axis module number - DS402 Master
#define M_AX_2			2				// Axis module number - DS402 Master
#define M_AX_3			3				// Axis module number - DS402 Master

#define S_AX_0			0				// Axis module number - DS402 Slave
#define S_AX_1			1				// Axis module number - DS402 Slave
#define S_AX_2			2				// Axis module number - DS402 Slave
#define S_AX_3			3				// Axis module number - DS402 Slave

#define DRIVE_BUSID1	3				// The CAN drive busId of the MiniMACS6

#define PDO_S_AX_0		1				// Used PDO number
#define PDO_S_AX_1		2				// Used PDO number
#define PDO_S_AX_2		3				// Used PDO number
#define PDO_S_AX_3		4				// Used PDO number

// Encoder settings & axis user units (Master)
#define AXIS_ENCRES 		4*256					// Resolution of the encoder for position feed back in increments (quadcounts)
#define	AXIS_POSENCREV		1						// Number of revolutions of the motor
#define	AXIS_POSENCQC		AXIS_ENCRES				// Number of quadcounts in POSENCREV revolutions
#define	AXIS_POSFACT_Z		1						// Number of revolutions of the input shaft
#define	AXIS_POSFACT_N		1						// Number of revolutions of the output shaft in POSFACT_Z revolutions of the input shaft
#define	AXIS_FEEDREV		1						// Number of revolutions of the gear box output shaft
#define	AXIS_FEEDDIST		360						// Distance travelled (in user units) in FEEDREV revolutions of the gear box output shaft [mm]

// Axis Movement Parameter (Master)
#define AXIS_MAX_RPM		5500					// Maximum velocity in RPM
#define AXIS_VELRES			100						// Velocity resolution, Scaling used for the velocity and acceleration/deceleration commands, default
#define AXIS_RAMPTYPE		RAMPTYPE_JERKLIMITED	// Defines the ramptype
#define AXIS_RAMPMIN		20000					// Maximum acceleration
#define AXIS_JERKMIN		1000					// Minimum time (ms) required before reaching the maximum acceleration
#define AXIS_TRACKERR		0						// There is also a following error on DS402 Slave, could be very high ond the MACS

// Motor settings for actual torque feedback → VIRTAMP_PROCESS(0,PO_VIRTAMP_CURRENT)
#define SLAVE_TORQUE_CONST		15700				// Represents the motor’s torque constant uNm/A
#define SLAVE_MOT_RATED_TORQUE	135					// Motor rated torque mNm

// Axis MACS control loop settings, Master position control is not used
#define	AXIS_KPROP			0
#define	AXIS_KINT			0
#define	AXIS_KDER			0
#define	AXIS_KILIM			0
#define	AXIS_KILIMTIME		0
#define	AXIS_BANDWIDTH		1000
#define	AXIS_FFVEL			1000
#define	AXIS_KFFAC			0
#define	AXIS_KFFDEC			0

// Define Cia402 Objects
#define DS402_AXESOFFSET	0x800

long main(void) {

	// local variables
    long i, retval;
    long homingStateAx_0=0,homingStateAx_1=0,homingStateAx_2=0,homingStateAx_3=0;

	print("-------------------------------------------------------------------------");
	print(" Test application CANopen Master - MiniMACS6 DS402 Slave");
	print("-------------------------------------------------------------------------");

	ErrorClear();
	AmpErrorClear(AXALL);

	InterruptSetup(ERROR, ErrorHandler);

	//----------------------------------------------------------------
	//	Device Setup
	//
	//	For the operation of 4 axes via Can the baud rate must be set
	//	to 1 MBaud. In addition, the CANSYNCTIME must be set to 2 ms.
	//	As a rule of thumb 3axis synchronization via CAN must happen
	// 	with 1Mbaud and synctime 1 ms.
	//----------------------------------------------------------------

	if(GLB_PARAM(CANBAUD)!=88 || GLB_PARAM(CANSYNCTIMER)!=2)
	{
		print("New Baudrate: 1MBaud");
		GLB_PARAM(CANBAUD)=88;
		GLB_PARAM(CANSYNCTIMER)=2;
		CanOpenRestart();
		Delay(500);
	}

	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------

	// set all slaves to PREOPERATIONAL by sending an NMT.
	SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 0;

	// initialising maxon drives
	sdkMiniMACS6_SetupCanSdoParam(DRIVE_BUSID1, PDO_S_AX_0, S_AX_0, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanSdoParam(DRIVE_BUSID1, PDO_S_AX_1, S_AX_1, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanSdoParam(DRIVE_BUSID1, PDO_S_AX_2, S_AX_2, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanSdoParam(DRIVE_BUSID1, PDO_S_AX_3, S_AX_3, MINIMACS6_OP_CST);

	// setup CANopen bus module for csp mode
	sdkMiniMACS6_SetupCanBusModule(M_AX_0, DRIVE_BUSID1, PDO_S_AX_0, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanBusModule(M_AX_1, DRIVE_BUSID1, PDO_S_AX_1, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanBusModule(M_AX_2, DRIVE_BUSID1, PDO_S_AX_2, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanBusModule(M_AX_3, DRIVE_BUSID1, PDO_S_AX_3, MINIMACS6_OP_CST);

	// setup virtual amplifier for csp mode
	sdkMiniMACS6_SetupCanVirtAmp(M_AX_0, AXIS_MAX_RPM, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanVirtAmp(M_AX_1, AXIS_MAX_RPM, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanVirtAmp(M_AX_2, AXIS_MAX_RPM, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanVirtAmp(M_AX_3, AXIS_MAX_RPM, MINIMACS6_OP_CST);

	// setup irtual counter for csp mode
	sdkMiniMACS6_SetupCanVirtCntin(M_AX_0, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanVirtCntin(M_AX_1, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanVirtCntin(M_AX_2, MINIMACS6_OP_CST);
	sdkMiniMACS6_SetupCanVirtCntin(M_AX_3, MINIMACS6_OP_CST);

	// start all slaves commanding them into OPERATIONAL.
	SYS_PROCESS(SYS_CANOM_MASTERSTATE) = 1;

	// Setup axsi parameter (Master)
	for (i = M_AX_0; i <= M_AX_3; i++)
	{
		// Movement parameters for the axis
		sdkSetupAxisMovementParam(	i,
									AXIS_VELRES,
									AXIS_MAX_RPM,
									AXIS_RAMPTYPE,
									AXIS_RAMPMIN,
									AXIS_JERKMIN,
									AXIS_TRACKERR
									);

		// Definition of the user units
		sdkSetupAxisUserUnits(		i,
									AXIS_POSENCREV,
									AXIS_POSENCQC,
									AXIS_POSFACT_Z,
									AXIS_POSFACT_N,
									AXIS_FEEDREV,
									AXIS_FEEDDIST
									);
		// Position control setup
		sdkSetupPositionPIDControlExt( 	i,
										AXIS_KPROP,
										AXIS_KINT,
										AXIS_KDER,
										AXIS_KILIM,
										AXIS_KILIMTIME,
										AXIS_BANDWIDTH,
										AXIS_FFVEL,
										AXIS_KFFAC,
										AXIS_KFFDEC
										);


		// Slave information for correct display of current torque
		SdoWrite( DRIVE_BUSID1, 0x6076 + DS402_AXESOFFSET*i, 	00,   	SLAVE_MOT_RATED_TORQUE);	// Set Motor rated torque
		SdoWrite( DRIVE_BUSID1, 0x28C0 + i, VIRTAMP_TORQUE_CONST,   	SLAVE_TORQUE_CONST);		// Set the motor’s torque constant

		print("Setup axis parameter: ", i);
	}
	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	// Homing setup
	print("\nDS402 MiniMACS6 Homing:");

	// The homing setup can also be configured on the DS402 slave.
	for (i = M_AX_0; i <= M_AX_3; i++)
	{
		SdoWrite( DRIVE_BUSID1, 0x6098 + DS402_AXESOFFSET*i, 	0,   37);	// Select "Home at actual position"
		SdoWrite( DRIVE_BUSID1, 0x607C + DS402_AXESOFFSET*i, 	0,   0);   	// Set homing offset to [uu]
	}

	// Homing statemachine
	retval=0;
	while(retval!=0x0F)
	{
		retval.i[0] = 	sdkMiniMACS6_AxisHomingStart(M_AX_0, DRIVE_BUSID1, MINIMACS6_OP_CST, homingStateAx_0);
		retval.i[1] =  	sdkMiniMACS6_AxisHomingStart(M_AX_1, DRIVE_BUSID1, MINIMACS6_OP_CST, homingStateAx_1);
		retval.i[2] =  	sdkMiniMACS6_AxisHomingStart(M_AX_2, DRIVE_BUSID1, MINIMACS6_OP_CST, homingStateAx_2);
		retval.i[3] =  	sdkMiniMACS6_AxisHomingStart(M_AX_3, DRIVE_BUSID1, MINIMACS6_OP_CST, homingStateAx_3);
	}

	// Disable MACS trackerror for cst mode
	AXE_PARAM(M_AX_0,POSERR)=0;

	print("");
	print("-----------------------------------------------------------");
	print("                Set torque in CST Mode                       ");
	print("----------------------------------------------------------- \n");

	AxisControl(M_AX_0, ON,M_AX_1, ON,M_AX_2, ON,M_AX_3, ON);

	for(i=10;i>=0;i--)
	{
		// The value is given in per thousand of “Motor rated torque”
		print("Set target torque → positive");
		AXE_PROCESS(M_AX_0,REG_USERREFCUR)= 500; // 50.0 torque%
		AXE_PROCESS(M_AX_1,REG_USERREFCUR)= 500; // 50.0 torque%
		AXE_PROCESS(M_AX_2,REG_USERREFCUR)= 500; // 50.0 torque%
		AXE_PROCESS(M_AX_3,REG_USERREFCUR)= 500; // 50.0 torque%

		Delay(4000);

		// The value is given in per thousand of “Motor rated torque”
		print("Set target torque → negative");
		AXE_PROCESS(M_AX_0,REG_USERREFCUR)= -500; // -50.0 torque%
		AXE_PROCESS(M_AX_1,REG_USERREFCUR)= -500; // -50.0 torque%
		AXE_PROCESS(M_AX_2,REG_USERREFCUR)= -500; // -50.0 torque%
		AXE_PROCESS(M_AX_3,REG_USERREFCUR)= -500; // -50.0 torque%

		Delay(4000);

		// The value is given in per thousand of “Motor rated torque”
		print("Set target torque → zero");
		AXE_PROCESS(M_AX_0,REG_USERREFCUR)= 0; // 0 torque%
		AXE_PROCESS(M_AX_1,REG_USERREFCUR)= 0; // 0 torque%
		AXE_PROCESS(M_AX_2,REG_USERREFCUR)= 0; // 0 torque%
		AXE_PROCESS(M_AX_3,REG_USERREFCUR)= 0; // 0 torque%

		Delay(2000);
		print(i, " repetitions to go \n");
	}

	AxisControl(M_AX_0, OFF);





	// Enable the axes
	AxisControl(M_AX_0, ON,M_AX_1, ON,M_AX_2, ON,M_AX_3, ON);


	if(VIRTAMP_PROCESS(M_AX_0,PO_VIRTAMP_STATUS).i[12]==1)
		print("Drive Ready");


	while(1)
	{
		print("Start, move to target position - 10 revolutions");
		AxisPosAbsStart(	M_AX_0, 10*AXIS_FEEDDIST,
							M_AX_1, 10*AXIS_FEEDDIST,
							M_AX_2, 10*AXIS_FEEDDIST,
							M_AX_3, 10*AXIS_FEEDDIST);
		AxisWaitReached(M_AX_0,M_AX_1,M_AX_2,M_AX_3);

		print("Target position is reached \n");
		print("Start, back to start position");

		AxisPosAbsStart(	M_AX_0, 0,
							M_AX_1, 0,
							M_AX_2, 0,
							M_AX_3, 0);
		AxisWaitReached(M_AX_0,M_AX_1,M_AX_2,M_AX_3);
		print("Start position is reached");
		print(i-1, " repetitions to go \n");
	}

	print("Program done, Axis OFF ");
	AxisControl(AXALL, OFF);

    return(0);
}

void ErrorHandler(void)
{
	long axeNbr 	= ErrorAxis();
	long errNbr		= ErrorNo();
	long errInfoNbr	= ErrorInfo();

    AxisControl(AXALL,OFF);

  	switch(errNbr)
	{
		case F_AMP:		if(	axeNbr==M_AX_0 ||axeNbr==M_AX_1 || axeNbr==M_AX_2 || axeNbr==M_AX_3 )
						{
							print("DS402 Slave Error");
							print("Error Axis: ", axeNbr ," ErrorCode: 0x", radixstr(SdoRead(DRIVE_BUSID1,0x603F,0x00),16));

							// Clear error on Slave Device

							Delay(5000);
							AmpErrorClear(axeNbr);
						}
						else
						{
							print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
						}
						break;

		case F_CANIO:	print("ErrorNo: ",errNbr," info: ",errInfoNbr);
						printf("SDO Abort Code %lX\n", SYS_PROCESS(SYS_CANOM_SDOABORT) );
						print("Check Can baudrate & Can bus id");
						break;

		default:		print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
	}

	ErrorClear();
	print("");	print(" There is no error handlig → Exit()");
	Exit(0);
}