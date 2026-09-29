/*********************************************************************
** Axis setup
*
* 	The axis must be set up specifically for the kinematics.
*	The user units of the axis must be in 1/100 degrees.
*
*********************************************************************/
/*********************************************************************
** Includes
*********************************************************************/
#include "..\..\..\..\SDK\SDK_ApossC.mc"
#include "AxisSetup.mh"

long SetupAxis(long axis)
{
    //Defines parameters which are required for the calculation of ramps and velocities
	sdkSetupAxisMovementParam(	axis,
								C_AXIS_VELRES,
								C_AXIS_MAX_RPM,
								C_AXIS_RAMPTYPE,
								C_AXIS_RAMPMIN,
								C_AXIS_JERKMIN,
								C_MOTOR_POSERR
								);



	// Definition of the user units
	sdkSetupAxisUserUnits(		axis,
								C_AXIS_POSENCREV,
								C_AXIS_POSENCQC,
								C_AXIS_POSFACT_Z,
								C_AXIS_POSFACT_N,
								C_AXIS_FEEDREV,
								C_AXIS_FEEDDIST
								);

	//	Follow the CAM profile without taking ACC, DEC, and VEL limitations into account
    AXE_PARAM(axis,SYNCTYPE) 	= 2;

    Vel(axis,1);
    Acc(axis,1);
    Dec(axis,1);

    print(" Setup axis ",axis);

    return(1);
}