/**
*	@brief		This test program shows the possibility to open an Ethernet UDP server
*	@detail		A UDP server will be opened. The port can be set by the define MY_PORT_NUM.
*				The program receives data from the client by means of an interrupt and
*				can process these in a routine.
*				In addition, a cyclic message is sent to the client in the CYCLE_TIME cycle.
*				UDP communication requires a firmware Version 9.9.00 or hihger.
*
*				1. 	set the IP address using ApossIDE:
*					Controller →  	Global/ Axis → IP Adress
*				2. 	power cycle the controller after successfully writing the parameters.
*				3.	compile and start the application
*				4.	start a UDP client (computer and MACS must be in the same subnet)
*
*	$Revision: 257 $
*
*	@example 	EthernetUDPSocket_OpenServer.mc
*
*/

#include "..\..\..\SDK\SDK_ApossC.mc"
#include <SysDef.mh>

// Parameters for the SDK function
#define MY_PORT_NUM      3000	// Define TCP Port number
#define CYCLE_TIME	     1000	// ms → 0 to disable

// Local functions
void EthernetSendData_sendCycle(void);
void FunctionEthernetHandler(void);

// global variables
long server_handle;
long status,status_old;

long main(void)
{
	long ip = SYS_INFO(SYS_IP_ADDRESS_CONTROLLER);
;
	// All errors are cleared at the beginning of the program.
    ErrorClear();

    print("EthernetOpenServer (UDP) with port: ",MY_PORT_NUM);
    print("IP address of this Server is: ",((ip)&0xFF),".",((ip>>8)&0xFF),".",((ip>>16)&0xFF),".",((ip>>24)&0xFF));

    // opening an ethernet server in case of a failure the program will be terminated
    server_handle = EthernetOpenServer(PROT_UDP, MY_PORT_NUM);
    print(" returned handler 'server_handle' : ",server_handle);
    if(server_handle<0)
    {
    	print("Exit program");
    	Exit(0);
    }

	// Wating for connection
	print("Wating for connection");
	while(3 != EthernetGetConnectionStatus(server_handle));

	// Start ethernet interrupts
	InterruptSetup(ETHERNET, FunctionEthernetHandler, server_handle);
	InterruptSetup(PERIOD, EthernetSendData_sendCycle, CYCLE_TIME);

	while (1)
	{
		status = EthernetGetConnectionStatus(server_handle);
		if(status != status_old)
		{
			sdkEthernetPrintConnectionStatus(status);
			status_old=status;
			Delay(1);
		}
	}

    return(0);
}

////////////////////////////////////////////////////////
void FunctionEthernetHandler(void)
{
	long result = 0;
	wchar receiveArray[50]="";
	arrayset(receiveArray,0,50);

    result = EthernetReceiveTelegram(server_handle, receiveArray);
    print("we received a UDP Telegram at time: ", Time(),"  size: ",result," data: ",receiveArray);

    return;
}


////////////////////////////////////////////////////////
void EthernetSendData_sendCycle(void)
{
	long result;
	wchar sendArray[40] = "";

	sprintf(sendArray, "UDP Server says Hello time: %d\n",Time());
	result = EthernetSendTelegram(server_handle, sendArray, arraylen(sendArray));
	print("SendCyclic time: ",Time());
	if (result != 0) {
		sdkEthernetPrintGeneralResult(result);
	}

    return;
}
