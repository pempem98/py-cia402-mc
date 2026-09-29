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



// Bus settings
#define SLAVE_CAN_BUSID				1

long waitTransitionToState(long busId, long targetState, long timeout);
void errorHandler();
void faultMonitoring();

long main(void)
{
   	long handle, i;
   	long actState=0;

   	//Reset CAN Bus 1
	handle = DefCanOut(0, 2);
	CanOut(handle, 0, 0x8100);
	//Reset CAN Bus 2
	handle = DefCanOut(100000, 2);
	CanOut(handle, 0, 0x8100);

	//Wait until the node has been resetted
	Delay(2000);
	ErrorClear();

	//set errorHandler for master
	InterruptSetup(ERROR, errorHandler);
	//Monitior the Fault bit in the status word of the slave, all 25ms
	InterruptSetup(PERIOD, faultMonitoring, 25);

	//----------------------------------------------------------------
	// Application Setup
	//----------------------------------------------------------------

	//Set the mode of operation to PVM
	sdkDS402_SetOpartionMode(SLAVE_CAN_BUSID, DS402_OP_PVM);

	//Get the actual DS402 state machnie state, and set it
	actState = sdkDS402_GetActDriveState(SLAVE_CAN_BUSID);
	if(actState == DS402_DRIVE_STATE_SWITCH_ON_DISABLED)
	{
		//change the state to 'Ready to switch on', and verify the the state was achieved with infinite timeout (-1)
		sdkDS402_WaitTransitionToState(SLAVE_CAN_BUSID, DS402_DRIVE_STATE_READY_TO_SWITCH_ON, -1);
	}

	//Set acceleration, deceleration
	sdkDS402_SetProfileAcceleration(SLAVE_CAN_BUSID, 2000);
    sdkDS402_SetProfileDeceleration(SLAVE_CAN_BUSID, 2000);


	//Set the deccelation for quick stop
	sdkDS402_SetQuickStopDeceleration(SLAVE_CAN_BUSID, 5000);

	//----------------------------------------------------------------
	// End of Application Setup
	//----------------------------------------------------------------


	//Transition to drive state 'Operation enabled'
	sdkDS402_WaitTransitionToState(SLAVE_CAN_BUSID, DS402_DRIVE_STATE_OPERATION_ENABLED, -1);
	sdkDS402_Print_StatusWord(SLAVE_CAN_BUSID);

	Delay(1000);

	for(i = 0; i < 2; i++)
	{
		//Set target velocity to 200rpm
		sdkDS402_PVM_SetTargetVelocity(SLAVE_CAN_BUSID, 200);
		//Start continuous velocity movment
		sdkDS402_PVM_CvelStart(SLAVE_CAN_BUSID);
		//Wait until target velocity is reached
		sdkDS402_PVM_WaitTargetReached(SLAVE_CAN_BUSID, -1);
		Delay(1500);

		//Set target velocity to 200rpm
		sdkDS402_PVM_SetTargetVelocity(SLAVE_CAN_BUSID, 100);
		//Start continuous velocity movment
		sdkDS402_PVM_CvelStart(SLAVE_CAN_BUSID);
		//Wait until target velocity is reached
		sdkDS402_PVM_WaitTargetReached(SLAVE_CAN_BUSID, -1);
		Delay(1500);

		//Stop
		sdkDS402_Halt(SLAVE_CAN_BUSID);
		Delay(500);

		//Set target velocity to 500rpm
		sdkDS402_PVM_SetTargetVelocity(SLAVE_CAN_BUSID, 500);
		//Start continuous velocity movment
		sdkDS402_PVM_CvelStart(SLAVE_CAN_BUSID);
		//Wait until target velocity is reached
		sdkDS402_PVM_WaitTargetReached(SLAVE_CAN_BUSID, -1);
		Delay(1500);

		//Stop continuous velocity movment
		sdkDS402_PVM_CvelStop(SLAVE_CAN_BUSID);
		Delay(500);
	}

	//Set target velocity to 1000rpm
	sdkDS402_PVM_SetTargetVelocity(SLAVE_CAN_BUSID, 1000);
	//Start continuous velocity movment
	sdkDS402_PVM_CvelStart(SLAVE_CAN_BUSID);
	//Wait until target velocity is reached
	sdkDS402_PVM_WaitTargetReached(SLAVE_CAN_BUSID, -1);
	Delay(2000);

	//Quick Stop
	sdkDS402_QuickStop(SLAVE_CAN_BUSID);

	return(1);
}

/**
*	@brief 		Monitor the fault bit
*	@details	This function monitors the fault bit of the status word.
*				If on node the fault bit is set, the error handler gets called
*
*/
void faultMonitoring()
{
	long statusWord = sdkDS402_ReadStatusWord(SLAVE_CAN_BUSID);
	if(statusWord.i[DS402_SW_BIT_FAULT])
	{
		errorHandler();
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
    sdkDS402_QuickStop(SLAVE_CAN_BUSID);

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
		statusWord = sdkDS402_ReadStatusWord(SLAVE_CAN_BUSID);
		if(statusWord.i[DS402_SW_BIT_FAULT])
		{
			print();

			printf("Bus Id %d, Slave device is in fault state\n", SLAVE_CAN_BUSID);
			if(statusWord.i[DS402_SW_PPM_BIT_FOLLWING_ERROR])
			{
				print("...Following Error");
			}


			//Reset the fault
			sdkDS402_ResetFault(SLAVE_CAN_BUSID);
		}
    }

	ErrorClear();
	print();
	print(" There is no error handlig → Exit()");
	print("-----------------------------------------------------------");
	Exit(0);
}
