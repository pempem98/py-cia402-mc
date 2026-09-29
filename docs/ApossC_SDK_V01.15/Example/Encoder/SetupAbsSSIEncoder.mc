/**
*	@brief		Program for reading an absolute SSI encoder
*	$Revision: 239 $
*
*/

#include <SysDef.mh>

#include "..\..\SDK\SDK_ApossC.mc"

#define AXIS 						0					// Axis module number
#define AXIS_ENCPORT 				0					// Encoder port number. Usually, module instance 0 is connected to X1 and so on. Please refer to product manual
#define AXIS_ENCRES 				4096				// Resolution of the encoder for position feed back in increments (quadcounts)
#define AXIS_ENC_CLOCK_FREQ			2000000				// Depending on the requirements of the sensor
#define AXIS_ENC_DATALENGTH 		13					// Databit length of endat position
#define AXIS_ENC_FAST_UPDATE		0					// 1 if SSI is used for commutation
#define AXIS_ENC_CODED 				0					// 1 if data coding is binary otherwise it is gray coded.
#define AXIS_ENC_LATCH_BITMASK		1					// Bitmask to check the latchvalue

long main(void)
{
	sdkSetupAbsSSIEncoder(	AXIS,
							AXIS_ENCPORT,
							AXIS_ENCRES,
							AXIS_ENC_CLOCK_FREQ,
							AXIS_ENC_FAST_UPDATE,
							AXIS_ENC_DATALENGTH,
							AXIS_ENC_CODED,
							AXIS_ENC_LATCH_BITMASK
							);

	while(1)
	{
		sdkInfoPrintAxesPos();
		Delay(500);
	}
    return(0);
}