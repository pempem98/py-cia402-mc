/**
*	@brief		This test program shows the use of an DS402 slave in profile position mode (PPM).
*	@detail		A DS402 slave is set up in ppm mode.
*
*	$Revision: 261 $
*
*	@example 	CAN_1Ax_DS402_ProfilePositionMode-Test_ppm.mc
*
*/

#include "..\..\..\SDK\SDK_ApossC.mc"

// Set the node ids of the all nodes in the network
long busIds[] = {1, 4};
long numberOfBusIds = arraylen(busIds);

void errorHandler();
void faultMonitoring();

void posAbsStartForAllNodes(long pos);
void posRelStartForAllNodes(long distance);
void verifyAllNodesStateAchieved(long state);
void verfiyAllNodesTargetReached();

long main(void)
{
   	long handle;
   	long actState=0;
   	long idIndex;

	//Reset CAN Bus 1
	handle = DefCanOut(0, 2);
	CanOut(handle, 0, 0x8100);
	//Reset CAN Bus 2
	handle = DefCanOut(100000, 2);
	CanOut(handle, 0, 0x8100);

	//Wait until nodes have been resetted
	Delay(2000);
	ErrorClear();

	//set errorHandler for master
	InterruptSetup(ERROR, errorHandler);
	//Monitior the Fault bit in the status word of the slave, all 25ms
	InterruptSetup(PERIOD, faultMonitoring, 25);

	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		//Set the mode of operation to PPM
		sdkDS402_SetOpartionMode(busIds[idIndex], DS402_OP_PPM);

		actState = sdkDS402_GetActDriveState(busIds[idIndex]);
		if(actState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED)
		{
			sdkDS402_TransitionToState(busIds[idIndex], DS402_DRIVE_STATE_READY_TO_SWITCH_ON);
		}

		//Set acceleration, deceleration and velocity
		sdkDS402_SetProfileMovementParameter(busIds[idIndex], 1000, 1000, 500);

		//Set the deccelation for quick stop
		sdkDS402_SetQuickStopDeceleration(busIds[idIndex], 5000);
	}

	verifyAllNodesStateAchieved(DS402_DRIVE_STATE_READY_TO_SWITCH_ON);

	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	//Transition to drive state 'Operation enabled'
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		//Transition to drive state 'Operation enabled'
	 	sdkDS402_TransitionToState(busIds[idIndex], DS402_DRIVE_STATE_OPERATION_ENABLED);
	}

	verifyAllNodesStateAchieved(DS402_DRIVE_STATE_OPERATION_ENABLED);

	while(1)
	{
		//Start absolute positioning to pos 100000 qc
		posAbsStartForAllNodes(100000);
		//Wait until the target position has been reached
		verfiyAllNodesTargetReached();

		//Start relative positioning of -100000 qc
		posRelStartForAllNodes(-100000);
		//Wait until the target position has been reached
		verfiyAllNodesTargetReached();
	}

	return(1);
}

/**
* @brief	Start an absolute position movement
* @details	This function start an absolute position movement for all nodes.
*
*/
void posAbsStartForAllNodes(long pos)
{
	long idIndex;
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		//Start absolute positioning
		sdkDS402_PPM_PosAbsStart(busIds[idIndex], pos, DS402_PPM_IMMEDIATELY);
	}
}


/**
* @brief	Start a relative position movement
* @details	This function start a relative position movement for all nodes.
*
*/
void posRelStartForAllNodes(long distance)
{
	long idIndex;
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		//Start relative positioning of distance
		sdkDS402_PPM_PosRelStart(busIds[idIndex], distance, DS402_PPM_IMMEDIATELY);
	}
}

/**
* @brief	Verify if all nodes have reached the target state.
* @details	This function checks whether all nodes reached the specified target state.
*
*/
void verifyAllNodesStateAchieved(long state)
{
	long idIndex;
	long actState;
	long result;

	while(result != 1)
	{
		result = 1;
		for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
		{
			actState = sdkDS402_GetActDriveState(busIds[idIndex]);
			result = (actState == state) && result;
		}
	}
}

/**
* @brief	Verify if all nodes have reached the target velocity.
* @details	This function checks whether all nodes reached the specified target velocity.
*
*/
void verfiyAllNodesTargetReached()
{
	long idIndex;
	long result;

	print("Wait until all nodes target is reached");
	while(result != 1)
	{
		result = 1;
		for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
		{
			result = sdkDS402_PPM_TargetReached(busIds[idIndex]) && result;
		}
	}
}

/**
* @brief	Quick stop
* @details	This function executes a quick stop on all nodes.
*
*/
void quickStopAllNodes()
{
	long idIndex;
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		sdkDS402_QuickStop(busIds[idIndex]);
	}
}

/**
*	@brief 		Monitor the fault bit for all nodes
*	@details	This function monitors the fault bit of the status word of all nodes.
*				If on node the fault bit is set, the error handler gets called
*
*/
void faultMonitoring()
{
	long idIndex;
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		long statusWord = sdkDS402_ReadStatusWord(busIds[idIndex]);
		if(statusWord.i[DS402_SW_BIT_FAULT])
		{
			errorHandler();
		}
	}
}

/**
*	@brief     Error Handler
* 	@details   This function processes errors encountered by the master device.
*              If a fault bit is set on the slave device, it will reset this bit
*              to restore normal operation. This ensures that the system can
*              recover from errors and continue functioning smoothly.
*
*/
void errorHandler()
{

	long axeNbr 	= ErrorAxis();
	long errNbr		= ErrorNo();
	long errInfoNbr	= ErrorInfo();
	long statusWord, idIndex, sdoAbortCode;

	//Stop all nodes
    quickStopAllNodes();

    print("-----------------------------------------------------------");
    printf("Error %d: ",errNbr);
    sdkErrorPrint_ApossIdeErrorDescription(errNbr);
    print();
    print("...Info: ",errInfoNbr, " AxisNo: ", axeNbr);
    print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    print();

    switch(errNbr)
	{
		case F_CANIO:
			printf("SDO Abort Code 0x%lX: ", sdoAbortCode );
			sdkErrorPrint_SdoErrorDescription(SYS_PROCESS(SYS_CANOM_SDOABORT));
			print();
			print("Check Can baudrate & Can bus id");
	}

    if(axeNbr == 0xFF && errNbr != F_CANIO)
    {
    	//The error is not releated to a certain axis
		for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
		{
			statusWord = sdkDS402_ReadStatusWord(busIds[idIndex]);
			if(statusWord.i[DS402_SW_BIT_FAULT])
			{
				print();

				printf("Bus Id %d, Slave device is in fault state\n", busIds[idIndex]);
				if(statusWord.i[DS402_SW_PPM_BIT_FOLLWING_ERROR])
				{
					print("...Following Error");
				}


				//Reset the fault
				sdkDS402_ResetFault(busIds[idIndex]);
			}
		}
    }

	ErrorClear();
	print();
	print(" There is no error handlig → Exit()");
	print("-----------------------------------------------------------");
	Exit(0);
}
