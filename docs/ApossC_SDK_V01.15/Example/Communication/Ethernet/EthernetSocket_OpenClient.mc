/**
*	@brief		This test program shows the possibility to open an Ethernet client
*	@detail		A TCP client will be used. The port and the Ip-Adresse can be set by a define itself.
*				The program receives data from the server by means of an interrupt and
*				can process these in a routine. The MACS controller responds in this program with
*				a "sendResponse". In addition, a cyclic message is sent to the client in the CYCLE_TIME cycle.
*
*				1. 	download & install Hercules for a first test
*					https://www.hw-group.com/software/hercules-setup-utility
*				2. 	set the IP address using ApossIDE:
*					Controller →  	Global/ Axis → IP Adress
*				3. 	power cycle the controller after successfully writing the parameters.
*				4.	compile and start the application
*				5.	start Herclues and connect in TCP server mode (computer must be in the same subnet)
*
*	$Revision: 202 $
*
*	@example 	EthernetSocket_OpenClient.mc
*
*/

#include "..\..\..\SDK\SDK_ApossC.mc"
#include <SysDef.mh>

// Parameters for the example
#define SERVER_IP_ADDRESS   	192.168.1.81	// IP address of the server socket.
#define SERVERT_PORT_NUM     	3000			// Port of the server socket.
#define CYCLE_TIME	     		1000			// ms → 0 to disable

// Local functions
void EthernetSendData_sendCycle(void);
void FunctionEthernetHandler(void);

// global variables
long client_handle;
long status, status_old;

//////////////////////////////////////////////////////////////////////////////
long main(void)
{
	// All errors are cleared at the beginning of the program.
    ErrorClear();

    // opening an ethernet client in case of a failure the program will be terminated
	print("EthernetOpenClient with IP: ",SERVER_IP_ADDRESS," and Port: ",SERVERT_PORT_NUM);
	client_handle = EthernetOpenClient(PROT_TCP, SERVER_IP_ADDRESS, SERVERT_PORT_NUM);   //  ipAddress, portNumber);

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
	long result;
	wchar receiveArray[50]="";
	wchar sendResponse[]="sendResponse";

    arrayset(receiveArray,0,result);

    result = EthernetReceiveTelegram(client_handle, receiveArray);
    print("\nwe received a Ethernet Telegram at time: ", Time());
    print("num of received data: ",result);
	print("receiveArray: ", receiveArray);

	result = sdkEthernetSendString(sendResponse, client_handle);
	sdkEthernetPrintGeneralResult(result);

    return;
}

void EthernetSendData_sendCycle(void)
{
	long result;
	wchar sendArray[30] = "";

	sprintf(sendArray, "EthernetSendData_sendCycle\n");
	result = sdkEthernetSendString(sendArray, client_handle);
	sdkEthernetPrintGeneralResult(result);

    return;
}