/**
*	@brief		Program for testing signal generator function on the MiniMACS6.
*	@details	A special encoder connection is necessary. Line A can be directly connected between
*				the TX and the RX port. But the B Signal will be generated on the I Line of the TX encoder.
*               This has to be connected to the B signal of the RX port.
*               The index signal will be generated on a digital output. This will be Digout 1 for TX port 0,
*               Digout 2 for TX port 1, and so on. This Digout signal has to be connected to Digin 1, to
*               trigger the index capture on the RX port.
*	$Revision: 213 $
*
*
*/

#include <SysDef.mh>

#include "..\..\SDK\SDK_ApossC.mc"
#include "..\..\SDK\Encoder\SDK_SignalGenerator.mc"

#define AXIS 					0		// Axis module number
#define ENCPORT_TX				0		// Encoder port number used for signal generator
#define ENCPORT_RX				1		// Encoder port number used for reception of the signal
#define INDEX_DISTANCE			10000

long startRecording(long axis);

long main(void)
{
	sdkSetupIncEncoder(AXIS,
	                   ENCPORT_RX,
	                   2000,
	                   1, 				//digital input latch type
	                   1, 				//digital input no
	                   HWLATCH_SLOPE_RISING);

	sdkSigGenOpenloopSetupMiniMACS6(ENCPORT_TX,
							INDEX_DISTANCE
							);


	sdkSigGenOpenloopAcceleration(ENCPORT_TX, 100000);
	sdkSigGenOpenloopDeceleration(ENCPORT_TX, 50000);

	Delay(1);

	DefOrigin(AXIS);

	startRecording(AXIS);

	//start master and ramp up
	sdkSigGenOpenloopVelocity(ENCPORT_TX, 100000);

	Delay(2500);

	sdkSigGenEnable(ENCPORT_TX, 0);
	Delay(100);
	sdkSigGenEnable(ENCPORT_TX, 1);

	//ramp down to 0
	sdkSigGenOpenloopVelocity(ENCPORT_TX, 0);

	Delay(3000);

	//reset index generator
	sdkSigGenReset(ENCPORT_TX);

	//start master
	sdkSigGenOpenloopVelocity(ENCPORT_TX, -100000);

	Delay(2500);

	sdkSigGenEnable(ENCPORT_TX, 0);
	Delay(100);
	sdkSigGenEnable(ENCPORT_TX, 1);

	//stop master
	sdkSigGenOpenloopVelocity(ENCPORT_TX, 0);

	Delay(2500);

	sdkSigGenEnable(ENCPORT_TX, 0);

	Delay(100);

	RecordStop(0, 0);


	while(1)
	{
		sdkInfoPrintAxesPos();
		Delay(500);
	}
    return(0);
}

long startRecording(long axis)
{
	//set up oscilloscope
	RecordTime(1);
	RecordDest(DYNMEM);
	RecordIndex(AXE_PROCESS_INDEX(axis, REG_ACTPOS),
				AXE_PROCESS_INDEX(axis, REG_ZERO),
				AXE_PROCESS_INDEX(axis, REG_IAVEL),
				VIRTMAST_PROCESS_INDEX(axis, PO_VIRTMAST_VEL));
	RecordType(0);
	RecordStart(0);
}