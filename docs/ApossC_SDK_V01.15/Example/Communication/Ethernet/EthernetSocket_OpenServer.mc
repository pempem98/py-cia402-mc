/**
*	@brief		This test program shows the possibility to open an Ethernet server
*	@detail		A TCP server will be opened. The port can be set by a define itself.
*				The program receives data from the client by means of an interrupt and
*				can process these in a routine. The MACS controller responds in this program with
*				a "sendResponse". In addition, a cyclic message is sent to the client in the CYCLE_TIME cycle.
*
*				1. 	download & install Hercules for a first test
*					https://www.hw-group.com/software/hercules-setup-utility
*				2. 	set the IP address using ApossIDE:
*					Controller →  	Global/ Axis → IP Adress
*				3. 	power cycle the controller after successfully writing the parameters.
*				4.	compile and start the application
*				5.	start Herclues and connect in TCP client mode (computer must be in the same subnet)
*
*	$Revision: 198 $
*
*	@example 	EthernetSocket_OpenServer.mc
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
	// All errors are cleared at the beginning of the program.
    ErrorClear();

    print("EthernetOpenServer with port: ",MY_PORT_NUM);

    // opening an ethernet server in case of a failure the program will be terminated
    server_handle = EthernetOpenServer(PROT_TCP, MY_PORT_NUM);
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
	long result;
	wchar receiveArray[50]="";
	wchar sendResponse[]="sendResponse";

    arrayset(receiveArray,0,result);

    result = EthernetReceiveTelegram(server_handle, receiveArray);
    print("\nwe received a Ethernet Telegram at time: ", Time());
    print("num of received data: ",result);
	print("receiveArray: ", receiveArray);

	result = sdkEthernetSendString(sendResponse, server_handle);
	sdkEthernetPrintGeneralResult(result);

    return;
}

void EthernetSendData_sendCycle(void)
{
	long result;
	wchar sendArray[30] = "";

	sprintf(sendArray, "EthernetSendData_sendCycle\n");
	result = sdkEthernetSendString(sendArray, server_handle);
	sdkEthernetPrintGeneralResult(result);

    return;
}
