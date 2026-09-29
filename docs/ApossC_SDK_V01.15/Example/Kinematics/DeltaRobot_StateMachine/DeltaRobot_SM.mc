/**
*	@brief		This program shows the use of a delta kinematics with a StateMachine
*	@detail		About:\n
*				This program sets up a delta kinematics. Various StateMachines events are presented.
*				Individual positions in a work or machine coordinate system can be moved to.
*				Additionally it is shown how a path can be executed. The path can be adapted in the
*				configuration file Delta_Show.zbc. \n
*
*				Instructions:\n
*				This program serves as support for the implementation of a kinematics based state machine.
*				The program is not intended for training in the use of state machines. Information about
*				state machines can be found in the ApossIDE Help.\n
*
*				Simulation:\n
*				The program can be simulated. This means that no motor or encoder must be connected.
*				It is enough to connect to a controller and start the program. With the #define MACHINE can be
*				switched to real motors(MACHINE → MINIMACS6_MOTORS). Afterwards 3 maxon motors are set up on a
*				MiniMACS6. The file "Motors\Maxon_XYZ.mc" must be adapted before start according to the
*				configurations of the motors/encoder.\n
*
*				Run:\n
*				If the program is executed, the kinematics can be commanded with the help of the
*				"HMI_TestRoutine.zbm" file.
*
*	$Revision: 238 $
*
*	@example 	DeltaRobot_SM.mc
*
*/
#include <SysDef.mh>

//Define encoder simulation or real motors on a MiniMACS6  → The file "Motors\Maxon_XYZ.mc" must be adapted before start
#define MINIMACS6_MOTORS		1
#define SIMULATION				2

#define MACHINE		 			SIMULATION

/*********************************************************************
** Application Defines
*********************************************************************/
#include "include\UserInterface.mh"

/*********************************************************************
** Machine-specific Definitions
*********************************************************************/
#include "include\MachineSpecification.mh"

/*********************************************************************
** Global Variables
*********************************************************************/
#include "include\Global_Variables.mh"

/*********************************************************************
** Global Functions
*********************************************************************/
#include "include\General_Functions.mc"
#include "include\AxisSetup.mc"

/*
** Load the path arrays from the ".zbc" configuration file.
*/
#config "include\Delta_Show.zbc", CONFIGFLG_DIM

/*********************************************************************
** State Machine Declarations
*********************************************************************/
#include "include\StateMachine_Main.mc"

/*********************************************************************
** Main Application Program
*********************************************************************/

long main(void)
{
    /*
    ** Start by clearing any previous errors.
    */
    ErrorClear();

    /*
    ** Initialize the user parameters.  These are used to communicate
    ** with the Monitor dialog.
    */
    UserParamInit();

    /*
    ** Start the state machine.
    */
    print("Starting...");

    SmRun(Delta);

    print("Done.");

    return(0);
}