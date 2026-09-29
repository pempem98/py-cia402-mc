/*********************************************************************
** StateMachine Main File
**
** This is the main file of the state machine.
**
** The error handling and various functions which must work in each state
** are processed here.
**
*********************************************************************/

/*********************************************************************
** Include SDK
*********************************************************************/
#include "StateMachine_Main.mh"
/*********************************************************************
** State Definitions
*********************************************************************/

SmState RobotMain
{
    SIG_INIT =
    {
		print("SM Robot: RobotMain - Init");
		/*
		** Set up the kinematics and get an kinematic handler
		*/
		Robot = sdkSetupDeltaKinematics( 	AXIS1,
											AXIS2,
											AXIS3,
											DELTA_A,
											DELTA_B,
											DELTA_C,
											DELTA_D,
											DELTA_MIN,
											DELTA_MAX);

		/*
		** Set up the Work-to-Machine transformation.  Note that since axis
		** user units are micrometers, Machine coordinate system units will
		** also be micrometers.
		*/
		WorkToMachine =	sdkSetupWorkCoordKinematics(	WorkToMachine,
														WCORD_ORIGIN_OFFSET_X,
														WCORD_ORIGIN_OFFSET_Y,
														WCORD_ORIGIN_OFFSET_Z,
														WCORD_ROT_XY,
														WCORD_SCALE);

		// Set a Work-to-Machine transformation that will be used with the selected kinematics
		KinematicsWorkToMc(Robot,WorkToMachine);

		// Force new homing
		data[HOMEDONE] = 0;

		return(SmTrans(->MotorsOn));
	}
	SIG_ERROR 		= ErrorHandler;
    SIG_UPDATE 		= UpdateDisplay;
    SIG_CMD_MOTOFF  = SmTrans(->MotorsOff);
    SIG_CMD_MOTON   = SmTrans(->MotorsOn);
    SIG_CMD_HOME    = SmTrans(->MotorsOn->Homing);

    SmState MotorsOff
	{
		SIG_ENTRY =
		{
			print("SM Robot: MotorsOff - Entry");
			USER_PARAM(USR_DSP_STATUS) = DSP_MOTOFF;
		}
	}

    SmState MotorsOn
	{
		SIG_INIT  =
		{
			print("SM Robot: MotorsOn - Init");
			if (data[HOMEDONE] == 0)
				return(SmTrans(->Homing));
			else
				return(SmTrans(->Running));
		}
        SIG_ENTRY =
        {
        	print("SM Robot: MotorsOn - Entry");
        	USER_PARAM(USR_DSP_STATUS) = DSP_MOTON;
			AxisControl(AXIS1, ON, AXIS2, ON, AXIS3, ON);
		}

        SIG_EXIT  =
        {
        	print("SM Robot: MotorsOn - Exit");
			AxisControl(AXALL, OFF);
		}

        /*
        ** The "Homing" state will home the robot.  When homing is
        ** complete, then the state will automatically be switched
        ** to the "Running" state.
        */
        SmState Homing
		{
			SIG_ENTRY =
			{
				print("SM Robot: Homing - Entry");
				USER_PARAM(USR_DSP_STATUS) = DSP_HOMING;
			}
			SIG_EXIT =
			{
				print("SM Robot: Homing - Exit");
				data[HOMEDONE] = 1;
			}
			// Here could be a homing routine
			SIG_IDLE =
			{
				DefOrigin(AXIS1,AXIS2,AXIS3);
				print("SM Robot: Homing - Done");
				return(SmTrans(Running));
			}
		}

        /*
        ** The "Running" state is entered once homing is complete.
        */
        SmState Running
		{
			SIG_ENTRY	=
			{
				print("SM Robot: Running - Entry");
				USER_PARAM(USR_DSP_STATUS) = DSP_RUNNING;

				print("SM Robot: Running - Kinematics enabled");
				KinematicsSetup(ENABLE, Robot);
			}

            SIG_EXIT    =
            {
            	print("SM Robot: Running - Exit");
            	print("SM Robot: Running - Kinematics disabled");
				KinematicsSetup(DISABLE, Robot);
			}
			SIG_FIXPNT   =
			{
				long fixptcode;
				print("SM Robot: SIG_FIXPNT");

				USER_PARAM(USR_DSP_FIXPNO) = event[PFGFIXPNT];
				print(" New Fixpoint:  Fixpoint = ",event[PFGFIXPNT]);

				fixptcode = GetSegmentCode(DimArray(USER_PARAM(USR_ARRNO)), USER_PARAM(USR_PATHNO), event[PFGFIXPNT]);
				print(" Fixpoint Code = ",fixptcode);
				data[NXTFIXPOINT] = event[PFGFIXPNT] + 1;
			}
            SIG_PATHSTOP =
            {
				long nxtfixptcode;
				print("SM Robot: SIG_PATHSTOP");

				print(" PathStop Event with event value ",event[PFGSTOPTYPE]);
				if (event[PFGSTOPTYPE] & SM_PATHSTOP_END)
				{
					print(" End of Path - DoSomeStuff xx");
				}
				else
				{
				   if (event[PFGSTOPTYPE] & SM_PATHSTOP_FIX)
					{
					   	if(data[NXTFIXPOINT] < GetSegmentCount(DimArray(USER_PARAM(USR_ARRNO)), USER_PARAM(USR_PATHNO)))
						{
						   nxtfixptcode = GetSegmentCode(DimArray(USER_PARAM(USR_ARRNO)), USER_PARAM(USR_PATHNO), data[NXTFIXPOINT]);
						   if (nxtfixptcode == 0)
						   {
								//print(" DoSomeStuff 0");
						   }
						   else
						   {
								//print(" DoSomeStuff xx");
						   }
						}
					}
				}
			}
            SIG_STDONE   = {print("SM Robot: SIG_STDONE");}
            SIG_MOVEDONE = {print("SM Robot: SIG_MOVEDONE");}
            SIG_CYCLE    =
            {
				double diff;
				print("SM Robot: SIG_CYCLE");
				print(" Path has cycled:  Wrap = ",event[PFGWRAP]);

				diff = TimeDiffus(timer) * 0.001;
				print(" Time to draw last cycle = ",diff," ms");
				USER_PARAM(USR_DSP_MOVTIME)=diff;
				timer = TimeHw();
				/*
				** Once the 10th cycle has started, then end the test.
				*/
				if (event[PFGWRAP] == USER_PARAM(USR_CYCLECMD))
			   	{
				   print(" End of path");
				   PathStop(Robot, -1);   // stop path immediately
			   	}
			}
			SIG_CMD_PATHSTART =
            {
            	print("SM Robot: SIG_CMD_PATHSTART");
            	USER_PARAM(USR_COMMAND) = 0;
            	timer = TimeHw();
            	// Make sure that the kinematics are ready for movement.
				if(AXE_PROCESS(0,PFG_AKTSTATE)==PGS_PATHIDLE||AXE_PROCESS(0,PFG_AKTSTATE)==PGS_PATHWAIT)
				{
					double Stx, Sty, Stz, Rotx, Roty, Rotz, Strchx, Strchy, Strchz, PathScale;
					double scale = (double) USER_PARAM(USR_SCALE);
					PathVel(Robot,  USER_PARAM(USR_PATHVEL));
					PathAcc(Robot,  USER_PARAM(USR_PATHACC));
					PathDec(Robot,  USER_PARAM(USR_PATHACC));
					PathJerk(Robot, USER_PARAM(USR_PATHJERK));

					Stx       = (double) USER_PARAM(USR_WORKPOSX) / scale;
					Sty       = (double) USER_PARAM(USR_WORKPOSY) / scale;
					Stz       = (double) USER_PARAM(USR_WORKPOSZ) / scale;
					Rotx      = (double) USER_PARAM(USR_PATHROTXYD) / scale;
					Roty      = (double) USER_PARAM(USR_PATHROTYZD) / scale;
					Rotz      = (double) USER_PARAM(USR_PATHROTZXD) / scale;
					Strchx    = (double) USER_PARAM(USR_PATHSTRCHX) / scale;
					Strchy    = (double) USER_PARAM(USR_PATHSTRCHY) / scale;
					Strchz    = (double) USER_PARAM(USR_PATHSTRCHZ) / scale;
					PathScale = (double) USER_PARAM(USR_PATHSCALE)  / scale;

					TransIdent     (PathToWork);                  		// The path curve (as defined in the .zbc file)
					TransScale     (PathToWork, PathScale);       		// Scale by 1/20 to convert from path units to mm's.
					TransRotateXY  (PathToWork, rad(Rotx));       		// Rotation in xy
					TransRotateYZ  (PathToWork, rad(Roty));       		// Rotation in yz
					TransRotateZX  (PathToWork, rad(Rotz));       		// Rotation in zx
					TransStretch   (PathToWork, Strchx, Strchy, Strchz);  // Stretching in x,y,z
					TransTranslate (PathToWork, Stx, Sty, Stz);   // Then Translate by X += Stx mm's and Y += Sty mm's.

					print(" DoPathStart → ArrayNo: ", USER_PARAM(USR_ARRNO), " PathNo: ", USER_PARAM(USR_PATHNO));
					DoPathStart(DimArray(USER_PARAM(USR_ARRNO)), USER_PARAM(USR_PATHNO), USER_PARAM(USR_PATHSTARTMP), USER_PARAM(USR_PATHSTTYPE));
				}
				else
				{
					print(" Path already run");
					SmPost(id, SIG_PATHSTOP);
				}
			}
			SIG_CMD_PATHCONT =
			{
			   print("SM Robot: SIG_CMD_PATHCONT - PathContinue");
			   USER_PARAM(USR_COMMAND) = 0;
			   PathContinue(Robot);
			}
            SIG_CMD_UPDATE_VEL =
            {
            	print("SM Robot: SIG_CMD_UPDATE_VEL - Update Path Velocity: ", USER_PARAM(USR_PATHVEL));
				USER_PARAM(USR_COMMAND) = 0;
				PathVel(Robot,  USER_PARAM(USR_PATHVEL));
				PathAcc(Robot,  USER_PARAM(USR_PATHACC));
				PathDec(Robot,  USER_PARAM(USR_PATHACC));
				PathJerk(Robot, USER_PARAM(USR_PATHJERK));
			}
            SIG_CMD_MACHINE_MOVE =
            {
				dpoint TmpPoint;
				double scale = (double) USER_PARAM(USR_SCALE);

				print("SM Robot: SIG_CMD_MACHINE_MOVE");
				print(" MachineCord: PathMove ");
				USER_PARAM(USR_COMMAND) = 0;
				USER_PARAM(USR_DSP_STATUS) = DSP_RUNNING_MMOVE;

				PathVel(Robot,  USER_PARAM(USR_MOVEVEL));
				PathAcc(Robot,  USER_PARAM(USR_MOVEACC));
				PathDec(Robot,  USER_PARAM(USR_MOVEACC));
				PathJerk(Robot, USER_PARAM(USR_MOVEJERK));

				TmpPoint.x = (double)USER_PARAM(USR_MACHPOSX) / scale;
				TmpPoint.y = (double)USER_PARAM(USR_MACHPOSY) / scale;
				TmpPoint.z = (double)USER_PARAM(USR_MACHPOSZ) / scale;

				PathMove(Robot, TmpPoint, PATHCOORD_MACHINE);
			}
            SIG_CMD_WORK_MOVE =
            {
				dpoint TmpPoint;
				double scale = (double) USER_PARAM(USR_SCALE);
				print("SM Robot: SIG_CMD_WORK_MOVE");
				print(" WorkCord: PathMove ");
				USER_PARAM(USR_COMMAND) = 0;
				USER_PARAM(USR_DSP_STATUS) = DSP_RUNNING_WMOVE;

				PathVel(Robot,  USER_PARAM(USR_MOVEVEL));
				PathAcc(Robot,  USER_PARAM(USR_MOVEACC));
				PathDec(Robot,  USER_PARAM(USR_MOVEACC));
				PathJerk(Robot, USER_PARAM(USR_MOVEJERK));

				TmpPoint.x = (double)USER_PARAM(USR_WORKPOSX) / scale;
				TmpPoint.y = (double)USER_PARAM(USR_WORKPOSY) / scale;
				TmpPoint.z = (double)USER_PARAM(USR_WORKPOSZ) / scale;

				PathMove(Robot, TmpPoint, PATHCOORD_WORK);
			}
			SIG_CMD_STOP     =
            {
            	print("SM Robot: SIG_CMD_STOP");
            	USER_PARAM(USR_COMMAND) = 0;
            	PathStop(Robot,-1);
            }
            SIG_CMD_TEST_1   = {print("SM Robot: SIG_CMD_TEST_1"); USER_PARAM(USR_COMMAND) = 0;}
            SIG_CMD_TEST_2   = {print("SM Robot: SIG_CMD_TEST_2"); USER_PARAM(USR_COMMAND) = 0;}
            SIG_CMD_TEST_3   = {print("SM Robot: SIG_CMD_TEST_3"); USER_PARAM(USR_COMMAND) = 0;}
            SIG_CMD_TEST_4   = {print("SM Robot: SIG_CMD_TEST_4"); USER_PARAM(USR_COMMAND) = 0;}
            SIG_CMD_TEST_5   = {print("SM Robot: SIG_CMD_TEST_5"); USER_PARAM(USR_COMMAND) = 0;}
		} // Running
	} // Motors on
} //  DeltaRobotMain

/*********************************************************************
** State Machine Definitions
*********************************************************************/
SmMachine Delta{ SMID_ROBOT,    // State machine ID.
                 RobotInit,     // Initialization function.
                 RobotMain, 	// Top-most state.
                 50,            // Event queue size.
                 ROBOT_DATA}    // Private data array size.

/*********************************************************************
** State Machine Initialization Functions
*********************************************************************/
long RobotInit(long id, long data[])
{
    /*
    ** Start by putting the pen up to make sure that it is out of the
    ** way while we are getting things set up (i.e. homing, etc.).
    **
    ** Note:    At the moment, don't do this with the Delta since it
    **       doesn't have a proper pen up/down function.  Also, this
    **       may be the wrong place for the Delta...since the axes
    **       have not yet even been initialized and the kinematics
    **       have not yet been set up.  We can't actually move the
    **       Delta yet!
    */
	print("SM Robot: RobotInit");

    /*
    ** Set the axis parameters.  Note that this defines axis user units
    ** to be micrometers.
    */
	SetupAxis(AXIS1);
	SetupAxis(AXIS2);
	SetupAxis(AXIS3);

    #if  MACHINE==MINIMACS6_MOTORS
		SetupMotor(AXIS1, 1);
		SetupMotor(AXIS2, 1);
		SetupMotor(AXIS3, 1);
	#else
		sdkSetupAxisSimulation(AXIS1);
		sdkSetupAxisSimulation(AXIS2);
		sdkSetupAxisSimulation(AXIS3);
	#endif

    /*
    ** Register for PFG events and other system events.
    */

    SmSystem(PFGEVENT, 0, SM_PFG_PATHFIXPNT,    Delta, SIG_FIXPNT);
    SmSystem(PFGEVENT, *, SM_PFG_PATHCYCLE,     Delta, SIG_CYCLE);
    SmSystem(PFGEVENT, 0, SM_PFG_PATHSTOP,      Delta, SIG_PATHSTOP);
    SmSystem(PFGEVENT, 0, SM_PFG_PATHSTARTDONE, Delta, SIG_STDONE);
    SmSystem(PFGEVENT, *, SM_PFG_PATHMOVEDONE,  Delta, SIG_MOVEDONE);

	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_MOTOFF,      SM_PARAM_EQUAL, Delta, SIG_CMD_MOTOFF);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_MOTON,       SM_PARAM_EQUAL, Delta, SIG_CMD_MOTON);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_HOME,        SM_PARAM_EQUAL, Delta, SIG_CMD_HOME);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_STOP,        SM_PARAM_EQUAL, Delta, SIG_CMD_STOP);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_UPDATE_VEL,    SM_PARAM_EQUAL, Delta, SIG_CMD_UPDATE_VEL);

	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_PATHSTART,   SM_PARAM_EQUAL, Delta, SIG_CMD_PATHSTART);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_PATHNEXT,    SM_PARAM_EQUAL, Delta, SIG_CMD_PATHNEXT);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_PATHCONT,    SM_PARAM_EQUAL, Delta, SIG_CMD_PATHCONT);

	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_MACHINE_MOVE,SM_PARAM_EQUAL, Delta, SIG_CMD_MACHINE_MOVE);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_WORK_MOVE, 	SM_PARAM_EQUAL, Delta, SIG_CMD_WORK_MOVE);

	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_TEST_1, 		SM_PARAM_EQUAL, Delta, SIG_CMD_TEST_1);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_TEST_2, 		SM_PARAM_EQUAL, Delta, SIG_CMD_TEST_2);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_TEST_3, 		SM_PARAM_EQUAL, Delta, SIG_CMD_TEST_3);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_TEST_4, 		SM_PARAM_EQUAL, Delta, SIG_CMD_TEST_4);
	SmParam(USER_PARAM_INDEX(USR_COMMAND), CMD_TEST_5, 		SM_PARAM_EQUAL, Delta, SIG_CMD_TEST_5);



	SmPeriod(UPDATE_PERIOD, Delta, SIG_UPDATE);
	SmInput(1,SM_INPUT_RISING, Delta, SIG_CMD_PATHSTART);

    SmSubscribe(id, SIG_ERROR); // Error Handler

    /*
    ** Set up the Path-to-Work transformation.  This is the transformation
    ** from path curve coordinates in the Path Editor to Work (i.e. paper)
    ** coordinates.  The Work coordinate system uses mm's and the square path
    ** curve is 1000 units on the long side.  We want the square to be about
    ** 50 mm's on the paper.  So one path curve unit will be 50/1000 = 0.05 mm.
    ** The starting point of the square will be at (50,50) on the paper.
    */

    TransIdent(PathToWork);         // The path curve (as defined in the .zbc file)
    TransScale(PathToWork, 0.05);   // Scale by 1/20 to convert from path units to mm's.

    return(0);
}

/*********************************************************************
** State Machine Functions
*********************************************************************/
long ErrorHandler(long id, long signal, long event[], long data[])
{
	long axeNbr 	= event[0];
	long errNbr		= event[1];
	long errInfoNbr	= event[2];

	switch(errNbr)
	{
		case F_GRENZE:	print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
						print("Axis Limit Reached Axis: ", axeNbr);
						print("NEGLIMIT: ", AXE_PARAM(axeNbr,NEGLIMIT), " POSLIMIT: ", AXE_PARAM(axeNbr,POSLIMIT), " ActPos: ", Apos(axeNbr));
						break;

		case F_UMIN:	print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
						print("Minimum voltage of the amplifier has been undershot: ", HWAMP_PROCESS(axeNbr,PO_HWAMP_VOLT1)*4);
						break;

		default:		print("ErrorNo: ",errNbr," info: ",errInfoNbr, " AxisNo: ", axeNbr);
	}
	ErrorClear();
	return(SmTrans(->MotorsOff));
}
long UpdateDisplay(long id, long signal, long event[], long data[])
{
    dpoint TmpPnt,CmdPnt;
    double curpos[3];
    double scale = (double)USER_PARAM(USR_SCALE);

    // Actual cmd position in work coordinates
    KinematicsCpoint(Robot, TmpPnt, PATHCOORD_WORK);
    CmdPnt = TmpPnt;

    USER_PARAM(USR_DSP_WRKPOSX) = TmpPnt.x * scale;
    USER_PARAM(USR_DSP_WRKPOSY) = TmpPnt.y * scale;
    USER_PARAM(USR_DSP_WRKPOSZ) = TmpPnt.z * scale;

    // Actual cmd position in machine coordinates
    KinematicsCpoint(Robot,TmpPnt,PATHCOORD_MACHINE);
    USER_PARAM(USR_DSP_MCHPOSX) = TmpPnt.x * scale;
    USER_PARAM(USR_DSP_MCHPOSY) = TmpPnt.y * scale;
    USER_PARAM(USR_DSP_MCHPOSZ) = TmpPnt.z * scale;

    // Actual in work coordinates
    curpos[0] = Apos(AXIS1);
    curpos[1] = Apos(AXIS2);
    curpos[2] = Apos(AXIS3);

    PathUuToMc(Robot,curpos,TmpPnt);
    TransInverse(WorkToMachine,TmpPnt);

    USER_PARAM(USR_DSP_WAPOSX)  = TmpPnt.x * scale;
    USER_PARAM(USR_DSP_WAPOSY)  = TmpPnt.y * scale;
    USER_PARAM(USR_DSP_WAPOSZ)  = TmpPnt.z * scale;

    // Actual tracking error in work coordinates
    TmpPnt.x = TmpPnt.x - CmdPnt.x;
    TmpPnt.y = TmpPnt.y - CmdPnt.y;
    TmpPnt.z = TmpPnt.z - CmdPnt.z;
    USER_PARAM(USR_DSP_WTRPOSX) = TmpPnt.x * scale;
    USER_PARAM(USR_DSP_WTRPOSY) = TmpPnt.y * scale;
    USER_PARAM(USR_DSP_WTRPOSZ) = TmpPnt.z * scale;

    return(SmHandled);
}