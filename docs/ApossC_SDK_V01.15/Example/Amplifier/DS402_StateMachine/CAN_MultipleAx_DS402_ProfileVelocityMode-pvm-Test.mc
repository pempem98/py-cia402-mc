/**
*	@brief		This test program shows the use of an DS402 slave in profile velocity mode (PVM).
*	@detail		A DS402 slave is set up in pvm mode.
*
*	$Revision: 261 $
*
*	@example 	CAN_1Ax_DS402_ProfilePositionMode-Test_pvm.mc
*
*/

#include "..\..\..\SDK\SDK_ApossC.mc"

// Set the node ids of the all nodes in the network
long busIds[] = {1,4};
long numberOfBusIds = arraylen(busIds);

void errorHandler();
void faultMonitoring();

void startCvelForAllNodes(long vel);
void stopCvelForAllNodes();
void verifyAllNodesStateAchieved(long state);
void verfiyAllNodesTargetReached();
void haltAllNodes();
void quickStopAllNodes();


long main(void)
{
	long idIndex;
   	long handle, i;
   	long actState=0;

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
		print(numberOfBusIds);
		//Set the mode of operation to PVM
		sdkDS402_SetOpartionMode(busIds[idIndex], DS402_OP_PVM);

		//Get the actual DS402 state machnie state, and set it
		actState = sdkDS402_GetActDriveState(busIds[idIndex]);
		if(actState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED)
		{
			//change the state to 'Ready to switch on'
			sdkDS402_TransitionToState(busIds[idIndex], DS402_DRIVE_STATE_READY_TO_SWITCH_ON);
		}

		//Set acceleration, deceleration
		sdkDS402_SetProfileAcceleration(busIds[idIndex], 2000);
		sdkDS402_SetProfileDeceleration(busIds[idIndex], 2000);


		//Set the deccelation for quick stop
		sdkDS402_SetQuickStopDeceleration(busIds[idIndex], 5000);
	}

	verifyAllNodesStateAchieved(DS402_DRIVE_STATE_READY_TO_SWITCH_ON);

	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------

	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		//Transition to drive state 'Operation enabled'
	 	sdkDS402_TransitionToState(busIds[idIndex], DS402_DRIVE_STATE_OPERATION_ENABLED);
	}

	verifyAllNodesStateAchieved(DS402_DRIVE_STATE_OPERATION_ENABLED);

	for(i = 0; i < 2; i++)
	{
		startCvelForAllNodes(200);
		verfiyAllNodesTargetReached();

		Delay(1500);
		startCvelForAllNodes(100);
		verfiyAllNodesTargetReached();
		Delay(1500);

		//Stop
		haltAllNodes();
		Delay(500);

		//Set target velocity to 500rpm
		startCvelForAllNodes(500);
		verfiyAllNodesTargetReached();
		Delay(1500);

		//Stop continuous velocity movment
		stopCvelForAllNodes();
		Delay(500);
	}

	//Set target velocity to 1000rpm
	startCvelForAllNodes(1000);
	verfiyAllNodesTargetReached();

	Delay(2000);

	//Quick Stop
	quickStopAllNodes();

	return(1);
}

/**
* @brief	Set target velocity and start continuous velocity
* @details	This function sets target velocity and starts continuous velocity for all nodes.
*
*/
void startCvelForAllNodes(long vel)
{
	long idIndex;
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		//Set target velocity to vel
		sdkDS402_PVM_SetTargetVelocity(busIds[idIndex], vel);
		//Start continuous velocity movment
		sdkDS402_PVM_CvelStart(busIds[idIndex]);
	}
}

/**
* @brief	Stops continuous velocity
* @details	This function stops continuous velocity for all nodes.
*
*/
void stopCvelForAllNodes()
{
	long idIndex;
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		//Stop continuous velocity movment
		sdkDS402_PVM_CvelStop(busIds[idIndex]);
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
* @brief	Quick stop
* @details	This function executes a halt on all nodes.
*
*/
void haltAllNodes()
{
	long idIndex;
	for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
	{
		sdkDS402_Halt(busIds[idIndex]);
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
* @brief	Verify if all nodes have reached the target velocity.
* @details	This function checks whether all nodes reached the specified target velocity.
*
*/
void verfiyAllNodesTargetReached()
{
	long idIndex;
	long result;

	while(result != 1)
	{
		result = 1;
		for(idIndex = 0; idIndex < numberOfBusIds; idIndex++)
		{
			result = sdkDS402_PVM_TargetReached(busIds[idIndex]) && result;
		}
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
	long statusWord, modeOfOperation, idIndex, sdoAbortCode;

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