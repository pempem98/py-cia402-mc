/**
*	@brief		This is a testprogram which demonstrates the use of the EPOS4 SDK
*	@detail		The drive will run back and forth when the program is started. It will also
*				record some test data. This data can be displayed by pressing the "Oscilloscope(Record)"
*				button after the program has run.
*
*	$Revision: 238 $
*
*	@example 	DeltaRobot_No_SM_EPOS4_ECAT.mc
*
*/
#include "..\..\..\SDK\SDK_ApossC.mc"

// Parameters for the SDK function
#define C_AXIS1	0						// Axis module number
#define C_AXIS2	1						// Axis module number
#define C_AXIS3	2						// Axis module number

#define C_DRIVE_BUSID1 1000001			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID2 1000002			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus
#define C_DRIVE_BUSID3 1000003			// The driveBusId is 1000000 plus the EtherCAT slave position in the bus

#define C_EC_CYCLE_TIME	1				// Cycletime in milliseconds
#define C_EC_OFFSET		1				// Shift offset

#define C_PDO_NUMBER	1				// Used PDO number
#define C_OP_MODE_CSP 	1				// Definition of the operation mode 1: CSP, 2: CSV

#define C_MOTOR_MAX_RPM		2000		// Maximum velocity in RPM

#define C_AXISPOLARITY		1			// Definition of the polarity 0: Normal, 1: Inverse

#define C_HOMING_START      1
#define C_HOMING_WAIT_DONE  2

// EPOS4 Parameter for homing methode
#define C_EPOS4_HOMING_SPEED						20						// Homing Speed / Speed for switch speed[rpm]
#define C_EPOS4_HOMING_POS_OFFSET					999						// Homing position offset [mA]
#define C_EPOS4_HOMING_CUR_THRESHOLD				2500					// Current threshold for homing mode [mA]

#define C_EPOS4_HOMING_METH_PRES_POS				37						// Homing to present position
#define C_EPOS4_HOMING_METH_CUR_THRESHOLD_POS		-3						// Current Threshold Positive Speed
#define C_EPOS4_HOMING_METH_CUR_THRESHOLD_NEG		-4						// Current Threshold Negative Speed
#define C_EPOS4_HOMING_METHODE C_EPOS4_HOMING_METH_CUR_THRESHOLD_POS		// Used homing methode

// EPOS4 operation mode
#define C_EPOS4_OP_MODE_HOMING 	6						// x6060 Operation mode ”6: Homing mode.”
#define C_EPOS4_OP_MODE_CSP		8						// x6060 Operation mode ”8: CSP.”

// Kinematics definition machine 1
#define C_MACH1_PARAM_0	  800.0   		// Length A is 80.0 mm.
#define C_MACH1_PARAM_1	 1200.0   		// Length B is 120.0 mm.
#define C_MACH1_PARAM_2	 1800.0   		// Length C is 180.0 mm.
#define C_MACH1_PARAM_3	  300.0   		// Length D is 30.0 mm.
#define C_MACH1_PARAM_4	 -10.0   	// The arm must not move higher than -10 degrees.
#define C_MACH1_PARAM_5	  90.0   	// The arm must not move lower than +10 degrees.

// Define work coordinate system
#define C_WORK1_SCALE 	10   		// Scale by 10 to convert from mm's to 1/10 of a mm.
#define C_WORK1_ORIENT	90   		// Coordinate system must be rotated CLOCKWISE by 90 degrees
#define C_WORK1_TRANS_X 0   		// Machine coordinate X offset (in micrometers) from home position to Work origin.
#define C_WORK1_TRANS_Y	0   		// Machine coordinate Y offset (in micrometers) from home position to Work origin.
#define C_WORK1_TRANS_Z	-2600.0		// Machine coordinate Y offset (in micrometers) from home position to Work origin.




// Global variable. Its also possible to set them as a local variable directly in the function.
// Only global variables can be used in a osziloscop or watchlist
long      	Robot;             	// Kinematics handle for the robot that is active.
transform 	WorkToMachine;     	// Work-to-Machine transformation.
dpoint 		TmpPoint;
dpoint 		TmpPointWork;
dpoint 		TmpPointMach;


// Declaration of local funcitons
long SetupAxisParam(long axis, long EncCpt, long MaxRpm, long MaxAcc);

long EncCpt = 6400/4; 			// Encoder resolution [cpt]
long MaxRpm = 2000; 			// Max. speed [rpm]
long MaxAcc = 1; 				// Max. acceleration [ms]: 0 -> MaxRpm

long main(void) {

	long slaveCount, i, homingState;
    homingState = C_HOMING_START;

	print("-----------------------------------------------------------");
	print(" Test application EtherCAT Master with 3 EPOS4 drive");
	print("-----------------------------------------------------------");


	ErrorClear();
	AxisControl(AXALL,ON);
	ECatMasterCommand(0x1000, 0);

	InterruptSetup(ERROR, ErrorHandler);

	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------

	slaveCount = sdkEtherCATMasterInitialize();
	print("slavecount: ",slaveCount);

	// initialising maxon drives
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID1, C_PDO_NUMBER, C_AXISPOLARITY, C_OP_MODE_CSP );
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID2, C_PDO_NUMBER, C_AXISPOLARITY, C_OP_MODE_CSP );
	sdkEpos4_SetupECatSdoParam(C_DRIVE_BUSID3, C_PDO_NUMBER, C_AXISPOLARITY, C_OP_MODE_CSP );

	for (i = 1; i <= 3; i++) {
	   sdkEtherCATSetupDC(i, C_EC_CYCLE_TIME, C_EC_OFFSET);    // Setup EtherCAT DC  (cycle_time [ms], offset [us]
    }

	// starting the ethercat
	sdkEtherCATMasterStart();

	// setup EtherCAT bus module for csp mode
	sdkEpos4_SetupECatBusModule(C_AXIS1, C_DRIVE_BUSID1, C_PDO_NUMBER, C_OP_MODE_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS2, C_DRIVE_BUSID2, C_PDO_NUMBER, C_OP_MODE_CSP);
	sdkEpos4_SetupECatBusModule(C_AXIS3, C_DRIVE_BUSID3, C_PDO_NUMBER, C_OP_MODE_CSP);

	// setup virtual amplifier for csp mode
	sdkEpos4_SetupECatVirtAmp(C_AXIS1, C_MOTOR_MAX_RPM, C_OP_MODE_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS2, C_MOTOR_MAX_RPM, C_OP_MODE_CSP);
	sdkEpos4_SetupECatVirtAmp(C_AXIS3, C_MOTOR_MAX_RPM, C_OP_MODE_CSP);

	// setup irtual counter for csp mode
	sdkEpos4_SetupECatVirtCntin(C_AXIS1, C_OP_MODE_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS2, C_OP_MODE_CSP);
	sdkEpos4_SetupECatVirtCntin(C_AXIS3, C_OP_MODE_CSP);

	// Setup MACS axis parameters
	SetupAxisParam(C_AXIS1, EncCpt, MaxRpm, MaxAcc);
	SetupAxisParam(C_AXIS2, EncCpt, MaxRpm, MaxAcc);
	SetupAxisParam(C_AXIS3, EncCpt, MaxRpm, MaxAcc);


	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------


	//----------------------------------------------------------------
	// Homing start
	//----------------------------------------------------------------

	switch (homingState) {

		case C_HOMING_START:
		   	print("\n -----------------------------------------------------------");
			print("                Homing Start. Methode: ", C_EPOS4_HOMING_METHODE);
			print("----------------------------------------------------------- \n");
		   	AxisControl(AXALL,OFF);
		   	//Delay(100);

		   	for (i = 0;  i < 3; i++) {

			//0x6040 Set “Control word, Bit4＝0 (0x000x): HALT.”
			AxisControl(i,ON);
			VIRTAMP_PARAM(i, VIRTAMP_CNTRLW_PWRONENP) = 0x0F;
			VIRTAMP_PARAM(i, VIRTAMP_CNTRLW_PWRONENN) = 0x0F;

			// Homing setup
			SdoWriten( (1000001+i), 0x6099, 1, 4,  C_EPOS4_HOMING_SPEED);    		// Homing Speed / Speed for switch speed[rpm]
			SdoWriten( (1000001+i), 0x30B0, 0, 4,  C_EPOS4_HOMING_POS_OFFSET);   	// homing position offset [mA]
			SdoWriten( (1000001+i), 0x30B2, 0, 2,  C_EPOS4_HOMING_CUR_THRESHOLD);  // Current threshold for homing mode [mA]
			SdoWriten( (1000001+i), 0x6098, 0, 1, C_EPOS4_HOMING_METHODE);   		// 0x6098 Set homing method to “37" : Homing to present position.”

			SdoWriten( (1000001+i), 0x6060, 0, 1, C_EPOS4_OP_MODE_HOMING);        	// x6060 Change operation mode to ”6: Homing mode.”
			print("mode of OP: ",(SdoRead(1000001,  0x6061, 0)),"  : ",(SdoRead(1000002,  0x6061, 0)),"  : ",(SdoRead(1000003,  0x6061, 0)));

			//0x6040 Set “Control word, Bit4＝1 (0x0010): Homing start.”
			VIRTAMP_PARAM(i, VIRTAMP_CNTRLW_PWRONENP) = 0x1F;
			VIRTAMP_PARAM(i, VIRTAMP_CNTRLW_PWRONENN) = 0x1F;
			AxisControl(i,ON);
		   }

		   	while (1) {
		   		printf("Status:   %lx   ;   %lx  ;  %lx \n",(SdoRead(1000001, 0x6041, 0)), (SdoRead(1000002, 0x6041, 0)),(SdoRead(1000003, 0x6041, 0)) );
			   	Delay(1000);

				   if ( (((SdoRead(1000001, 0x6041, 0)) & 0x600) == 0x600) &&
						(((SdoRead(1000002, 0x6041, 0)) & 0x600) == 0x600) &&
						(((SdoRead(1000003, 0x6041, 0)) & 0x600) == 0x600) ) {

					printf("Status:   %lx   ;   %lx  ;  %lx \n",(SdoRead(1000001, 0x6041, 0)), (SdoRead(1000002, 0x6041, 0)),(SdoRead(1000003,  0x6041, 0)) ) ;
					print("\n ALL Homing done");

					for (i = 0;  i < 3; i++) {
					   VIRTAMP_PARAM(i, VIRTAMP_CNTRLW_PWRONENP) = 0x0F;
					   VIRTAMP_PARAM(i, VIRTAMP_CNTRLW_PWRONENN) = 0x0F;
					   AxisControl(i,OFF);
					   Delay(4);
					   SdoWriten( (1000001+i), 0x6060, 0, 1, C_EPOS4_OP_MODE_CSP );        // 0x6060 Change operation mode to ”8: CSP mode.”
					   AxisControl(i,ON);
					   Delay(4);
					}
					break;
			   }
		   }

		   homingState = C_HOMING_WAIT_DONE;
		   break;

		case C_HOMING_WAIT_DONE:
		   break;
	}


	AxisControl(C_AXIS1,ON, C_AXIS2,ON,  C_AXIS3,ON);

	print("-----------------------------------------------------------");
	print("               Setup Delta Kinematics                      ");
	print("----------------------------------------------------------- \n");


	//Setup kinematicS
	Robot = sdkSetupDeltaKinematics(C_AXIS1,C_AXIS2,C_AXIS3, C_MACH1_PARAM_0, C_MACH1_PARAM_1, C_MACH1_PARAM_2, C_MACH1_PARAM_3, C_MACH1_PARAM_4, C_MACH1_PARAM_5);
	WorkToMachine = sdkSetupWorkCoordKinematics(WorkToMachine, C_WORK1_TRANS_X, C_WORK1_TRANS_Y, C_WORK1_TRANS_Z, C_WORK1_ORIENT, C_WORK1_SCALE);

	//Set a Work-to-Machine transformation that will be used with the selected kinematics
	KinematicsWorkToMc(Robot,WorkToMachine);

 	KinematicsSetup(ENABLE, Robot);
	print("Kinematics enabled");

	PathVel(Robot,10.0);
	PathAcc(Robot,10.0);
	PathDec(Robot,10.0);
	PathJerk(Robot,10.0);
	print("Set path speed");

	// Set move point to drive
	TmpPoint.x = 250;
   	TmpPoint.y = 0;
   	TmpPoint.z = 0;

	//----------------------------------------------------------------
	// Test simple kinematics
	//----------------------------------------------------------------


	// If set to 1 -> print kintematics tcp position
	if (1)
	{
		print("Axis -> Off. cordinate system can be tested");
		AxisControl(AXALL,OFF);
		while(1)
			{
			KinematicsCpoint(Robot, TmpPointWork, PATHCOORD_WORK);
			KinematicsCpoint(Robot, TmpPointMach, PATHCOORD_MACHINE);

			print("Work");
			print("X: ",TmpPointWork.x," Y: ",TmpPointWork.y);
			print("Machine");
			print("X: ",TmpPointMach.x," Y: ",TmpPointMach.y);
			Delay(500);
			}
	}
	// Else go to TmpPoint
	else
	{
		print("Got to TmpPoint in Machine Coord");
		PathMove(Robot, TmpPoint, PATHCOORD_MACHINE);		// Machine cordinate system
		AxisWaitReached(AXALL);

 		//PathStart(Robot, 0, 0, WorkToMachine, 0, PATHSTART_NORMAL);
		AxisControl(AXALL,OFF);
	}

    return(0);

} // main

// Setup MACS axis parameters
long SetupAxisParam(long axis, long EncCpt, long MaxRpm, long MaxAcc)
{
	// Ensure that this corresponds to the encoder and EPOS4 configuration!
	AXE_PARAM(axis,POSENCQC)  	= EncCpt*4;	 	// Set enc. resolution per turn [inc]
	AXE_PARAM(axis,POSENCREV) 	= 1; 			// Default: 1
	AXE_PARAM(axis,FEEDDIST)  	= 36000;	 	// Set feed resolution (= output) per turn [inc]
	AXE_PARAM(axis,FEEDREV)   	= 1; 			// Default: 1

	AXE_PARAM(axis, VELMAX) 	= MaxRpm; 		// Set max. velocity [rpm]
	AXE_PARAM(axis, RAMPMIN)  	= MaxAcc; 		// Set max. acceleration [ms] (0- > MaxRpm)

	AXE_PARAM(axis, KPROP)     	= 0;  			// Disable P-Gain of PID-Controller
	AXE_PARAM(axis, KDER)      	= 0;  			// Disable D-Gain of PID-Controller
	AXE_PARAM(axis, KINT)      	= 0;	 		// Disable I-Gain of PID-Controller

	AXE_PARAM(axis, POSERR)      = 2000000; 		// Set max following error [inc]
	return(1);
}


//////////////////////////////////////////////////////////////////////////
void ErrorHandler(void) {
	long ErrorAxe;
	ErrorAxe=ErrorAxis();

    print("Error ",ErrorNo()," errax: ",ErrorAxis() );
    if(ErrorNo() == 89) {
        printf("SDO Abort Code %lX", SYS_PROCESS(SYS_CANOM_SDOABORT) );
    }
    if (ErrorNo() == 8) {
		 print(" Trackerror   time: ",Time() );
		 print("ErrorAx: ",ErrorAxis() );
		 Delay(10);
		 // disable Master
		 ECatMasterCommand(0x1000, 0);
    }
    if(ErrorNo() == 40) {
        print("Error Axis: ", ErrorAxe ," ErrorCode: ", radixstr(SdoRead(1000000+ErrorAxe+1,0x603F,0x00),16));
    }
// 	ErrorClear();
    Delay(2000);
}


