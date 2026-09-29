/**
*	@brief		Program for controlling a single ECi40 motor from Maxon.
*	@detail		This program controls an ECi40 motor from maxon. An incremental encoder with a resolution of 1024 qc is used. \n
*				The purpose of this program is to show how to set up a motor, an encoder and the axis using the SDK. \n
*				For this purpose the following must be configured: \n
*				- Encoder \n
*				- Amplifier \n
*				- ControllerMode and controller (position, velocity, current) \n
*				- Axis parameters (user units, direction, limits) \n
*
*				It is recommended to implement the setup of a motor outside the main program in an external *.mh file.
*
*	$Revision: 271 $
*
*/

#include <SysDef.mh>
#include "..\..\..\..\..\SDK\SDK_ApossC.mc"
#include "Maxon_XYZ.mh"

void SetupMotor(long axis, long direction)
{
	// Encoder setup
	sdkSetupSinCosEncoder(		axis,
								axis,
								C_MOTOR_ENCRES,
								C_MOTOR_ENC_LATCHTYPE,
								C_MOTOR_ENC_LATCHPARAM,
								C_MOTOR_ENC_LATCHSLOPE
								);
	// Amplifier setup
	HWAMP_PARAM(axis,HWAMP_HALL_ALIGNMENT) = 4;
	sdkSetupAmpHallPmsmMotor(	axis,
								C_MOTOR_CONTROLMODE,
								C_MOTOR_POLEPAIRS,
								C_MOTOR_MAXCUR,
								C_MOTOR_ENCRES,
								C_MOTOR_MAX_RPM,
								-1
								);
	// Current control setup
	sdkSetupCurrentPIControl( 	axis,
								C_MOTOR_CURKPROP,
								C_MOTOR_CURKINT,
								C_MOTOR_CURKILIM
								);
	// Velocity control setup
	sdkSetupVelocityPIControl( 	axis,
								C_MOTOR_VELKPROP,
								C_MOTOR_VELKINT,
								C_MOTOR_VELKILIM
								);
	// Movement parameters for the axis
	sdkSetupAxisMovementParam(	axis,
								C_MOTOR_VELRES,
								C_MOTOR_MAX_RPM,
								C_MOTOR_RAMPTYPE,
								C_MOTOR_RAMPMIN,
								C_MOTOR_JERKMIN,
								C_MOTOR_POSERR
								);
	// Set the direction of the axis
	sdkSetupAxisDirection(		axis,
								direction);
	// Position control setup
	sdkSetupPositionPIDControlExt( 	axis,
									C_MOTOR_KPROP,
									C_MOTOR_KINT,
									C_MOTOR_KDER,
									C_MOTOR_KILIM,
									C_MOTOR_KILIMTIME,
									C_MOTOR_BANDWIDTH,
									C_MOTOR_FFVEL,
									C_MOTOR_KFFAC,
									C_MOTOR_KFFDEC
									);


}
