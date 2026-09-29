/**
*	@brief		This test program shows the possibility to open an Ethernet UDP client
*	@detail		A UDP client will be used. The port and the Ip-Adresse can be set by a define itself.
*				The program receives data from the server by means of an interrupt and
*				can process these in a routine.
*				In addition, a cyclic message is sent to the client in the CYCLE_TIME cycle.
*				UDP communication requires a firmware Version 9.9.00 or hihger.
*
*				1. 	start a UDP server (computer and MACS must be in the same subnet)
*				2. 	set the IP address using ApossIDE:
*					Controller →  	Global/ Axis → IP Adress
*				3. 	power cycle the controller after successfully writing the parameters.
*				4.	compile and start the application

*
*	$Revision: 257 $
*
*	@example 	EthernetUDPSocket_OpenClient.mc
*
*/

#include "..\..\..\SDK\SDK_ApossC.mc"
#include <SysDef.mh>

// Parameters for the example
#define SERVER_IP_ADDRESS   	192.168.1.3	// IP address of the server socket.
#define SERVER_PORT_NUM     	3000		// Port of the server socket.
#define CYCLE_TIME	     		200			// ms → 0 to disable

// Local functions
void EthernetSendData_sendCycle(void);
void FunctionEthernetHandler(void);

// global variables
long client_handle;
long status, status_old;


//////////////////////////////////////////////////////////////////////////////
long main(void)
{
	long ip = SERVER_IP_ADDRESS;
	// All errors are cleared at the beginning of the program.
    ErrorClear();

    // opening an ethernet client in case of a failure the program will be terminated
	print("EthernetOpenClient (UDP) to IP: ",((ip)&0xFF),".",((ip>>8)&0xFF),".",((ip>>16)&0xFF),".",((ip>>24)&0xFF),"  and Port: ",SERVER_PORT_NUM);
	client_handle = EthernetOpenClient(PROT_UDP, SERVER_IP_ADDRESS, SERVER_PORT_NUM);   //  ipAddress, portNumber);
	ip = SYS_INFO(SYS_IP_ADDRESS_CONTROLLER);
	print(" IP address of this client is: ",((ip)&0xFF),".",((ip>>8)&0xFF),".",((ip>>16)&0xFF),".",((ip>>24)&0xFF) );

    print(" returned handler 'client_handle' : ",client_handle);
    if(client_handle<0)
    {
    	sdkEthernetPrintGeneralResult(client_handle);
    	print("Exit program");
    	Exit(0);
    }

	// Wating for connection
	print("Wating for connection");
	while(3 != status)
	{
		status = EthernetGetConnectionStatus(client_handle);
		if(status != status_old)
		{
			sdkEthernetPrintConnectionStatus(status);
			status_old=status;
			Delay(1);
		}
	}

	// Start interrupts
	InterruptSetup(ETHERNET, FunctionEthernetHandler, client_handle);
	InterruptSetup(PERIOD, EthernetSendData_sendCycle, CYCLE_TIME);

	while (1) {
		status = EthernetGetConnectionStatus(client_handle);
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

    result = EthernetReceiveTelegram(client_handle, receiveArray);
    print("we received a UDP Telegram at time: ", Time(),"  size: ",result," data: ",receiveArray);

    return;
}


////////////////////////////////////////////////////////
void EthernetSendData_sendCycle(void)
{
	long result;
	wchar sendArray[40] = "";

	sprintf(sendArray, "UDP Client says Hello time: %d\n",Time());
	result = EthernetSendTelegram(client_handle, sendArray, arraylen(sendArray));
	print("SendCyclic time: ",Time());
	if (result != 0) {
		sdkEthernetPrintGeneralResult(result);
	}
    return;
}