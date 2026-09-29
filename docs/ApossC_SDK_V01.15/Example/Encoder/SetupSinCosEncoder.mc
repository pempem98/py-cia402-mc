/**
*	@brief		Program for reading a SinCos encoder
*	$Revision: 244 $
*
*/

#include <SysDef.mh>

#include "..\..\SDK\SDK_ApossC.mc"

#define AXIS 					0						// Axis module number
#define AXIS_ENCPORT 			3						// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
#define AXIS_ENCRES 			5000					// Resolution of the encoder for position feed back in increments (quadcounts)
#define AXIS_ENC_LATCHTYPE 		1						// Defines the latch type: Digital input
#define AXIS_ENC_LATCHPARAM 	1						// Latch on digital input 1
#define AXIS_ENC_LATCHSLOPE 	HWLATCH_SLOPE_RISING	// Defines the slope of the trigger signal (Default 1)

long main(void)
{
	sdkSetupSinCosEncoder(	AXIS,
							AXIS_ENCPORT,
							AXIS_ENCRES*256,
							AXIS_ENC_LATCHTYPE,
							AXIS_ENC_LATCHPARAM,
							AXIS_ENC_LATCHSLOPE
							);

	while(1)
	{
		sdkInfoPrintAxesPos();
		Delay(2000);
	}
    return(0);
}