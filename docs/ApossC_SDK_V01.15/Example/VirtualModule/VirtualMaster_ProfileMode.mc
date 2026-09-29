#include <SysDef.mh>

#include "..\..\SDK\SDK_ApossC.mc"

#define VIRTUAL_MASTER	0
#define VIRTUAL_AXIS	0

long main(void)
{

	// Setup an virtual master in profile mode
	sdkSetupVirtualMasterMode(VIRTUAL_MASTER, VIRTMAST_MODE_PROFILE);

	// Setup an virtual axis for simulation
	sdkSetupAxisSimulation(VIRTUAL_AXIS);

	// Set the virtual master VIRTUAL_MASTER as master of VIRTUAL_AXIS
	sdkSetupVirtualMasterAxisLink(VIRTUAL_MASTER, VIRTUAL_AXIS);

	// The "inputs" are converted from increments to quadcounts.
	sdkSetupVirtualMasterScale(VIRTUAL_MASTER,4,1);

	// Set the profile of the virtual master
	// Acc/ Dec = 10 qc / ms
	sdkSetVirtualMasterProfile(VIRTUAL_MASTER, 10, 10);

	// The axis must be on and ready for synchronisation
	AxisControl(VIRTUAL_AXIS,ON);
	SyncPos(VIRTUAL_AXIS);

	// Start the virtual master with a velocity of 10
	sdkStartVirtualMasterProfile(VIRTUAL_MASTER, 10);

	while(1)
	{
		sdkInfoPrintAxesPos();
		Delay(2000);
	}

	// Stops the virtual master -> Set velocity to zero
	sdkStopVirtualMasterProfile(VIRTUAL_MASTER);

	// Disable the virtual master module
	sdkSetupVirtualMasterMode(VIRTUAL_MASTER, VIRTMAST_MODE_DISABLED);

    return(0);
}