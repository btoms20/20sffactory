//
//  ParallelRobotKinematics.cpp
//  
//
//  Created by Brandon Toms on 8/4/21.
//

#include "ParallelRobotKinematics.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Storage/MassStorage.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/DDA.h>
//#include <Kinematics.h>

#include <limits>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(ParallelRobotKinematics, __VA_ARGS__)


constexpr ObjectModelTableEntry ParallelRobotKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members angles
	{ "name", 				OBJECT_MODEL_FUNC(self->GetName(true)), ObjectModelEntryFlags::none },

	// 1. kinematics members lengths
	{ "distalArmLength",	OBJECT_MODEL_FUNC(self->distalArmLength, 3), ObjectModelEntryFlags::none },
	{ "proximalArmLength",	OBJECT_MODEL_FUNC(self->proximalArmLength, 3), ObjectModelEntryFlags::none },

	// 2. angle limits and endstop positions
	{ "axisBaseLimitMax",		OBJECT_MODEL_FUNC(self->baseLimits[1], 3),  ObjectModelEntryFlags::none },
	{ "axisBaseLimitMin",		OBJECT_MODEL_FUNC(self->baseLimits[0], 3),  ObjectModelEntryFlags::none },
	{ "axisDistalLimitMax",		OBJECT_MODEL_FUNC(self->psiLimits[1], 3),   ObjectModelEntryFlags::none },
	{ "axisDistalLimitMin",		OBJECT_MODEL_FUNC(self->psiLimits[0], 3),   ObjectModelEntryFlags::none },
	{ "axisProximalLimitMax",	OBJECT_MODEL_FUNC(self->thetaLimits[1], 3), ObjectModelEntryFlags::none },
	{ "axisProximalLimitMin",	OBJECT_MODEL_FUNC(self->thetaLimits[0], 3), ObjectModelEntryFlags::none },
//	{ "endStopAngleProximal",	OBJECT_MODEL_FUNC(self->endStopAngles[0], 3), ObjectModelEntryFlags::none },
//	{ "endStopAngleDistal",		OBJECT_MODEL_FUNC(self->endStopAngles[1], 3), ObjectModelEntryFlags::none },
//	{ "endStopAngleBase",		OBJECT_MODEL_FUNC(self->endStopAngles[2], 3), ObjectModelEntryFlags::none },
};


// number of groups, number of entries for each group:
constexpr uint8_t ParallelRobotKinematics::objectModelTableDescriptor[] = { 3, 1, 2, 6 };

DEFINE_GET_OBJECT_MODEL_TABLE(ParallelRobotKinematics)

#endif

// Assume this is a 2 axis parallel arm robot with a rotational base as the 3rd axis.
ParallelRobotKinematics::ParallelRobotKinematics() noexcept
    : Kinematics(KinematicsType::parallelRobot, SegmentationType(true, true, true)),
      proximalArmLength(DefaultProximalArmLength), distalArmLength(DefaultDistalArmLength), xOffset(0.0), yOffset(0.0), zOffset(0.0)
{
    thetaLimits[0] = DefaultMinTheta;
    thetaLimits[1] = DefaultMaxTheta;
    psiLimits[0] = DefaultMinPsi;
    psiLimits[1] = DefaultMaxPsi;
    baseLimits[0] = DefaultMinBase;
    baseLimits[1] = DefaultMaxBase;
    toolOffsetY = DefaultToolOffsetY;
    toolOffsetZ = DefaultToolOffsetZ;
    Recalc();
}

// Return the name of the current kinematics
const char *ParallelRobotKinematics::GetName(bool forStatusReport) const noexcept
{
    return "ParallelRobot";
}

// Calculate theta, psi and the new arm mode from a target position.
// If the position is not reachable because it is out of radius limits, set theta and psi to NaN and return false.
// Otherwise set theta and psi to the required values and return true if they are in range.
// Note: theta and psi are now returned in degrees.
bool ParallelRobotKinematics::CalculateThetaAndPsi(const float machinePos[], bool isCoordinated, float& theta, float& psi, float& base) const noexcept
{
    // Theta is the angle of our proximal arm (lower shank) with 0 degrees being upright, positive rotations are forward in the Y direction. Negative rotations are therefor backwards, towards the endstops.
    //  - Theta is a direct link to our proximal arm
    // Psi is the angle of our distal arm (uppershank) and is dependent on that of the proximal arm
    
    // The arm is constrained along the YZ plane. The x axis is exposed/reachable via the rotation of the base.
    // Calc top down X,Y hypotenous (including any x and y offsets)
    const float hypotenousTop = fastSqrtf( fsquare(machinePos[X_AXIS] + xOffset) + fsquare(machinePos[Y_AXIS] + yOffset) ) - toolOffsetY;
    
    // If the hypotenous and z position match our caches then we can reuse the previous calculations.
    if (hypotenousTop == cachedHypotenousTop && machinePos[Z_AXIS] == cachedZ) {
        cachedX = machinePos[X_AXIS];
        cachedY = machinePos[Y_AXIS];
        cachedZ = machinePos[Z_AXIS];
        return true;
    }
    
    if (hypotenousTop > maxRadius) {
    	// Unreachable (hypotenousTop is longer than our combined shank length)
    	return false;
    }

    // Calc base rotation (including any x and y offsets)
    // Base rotation is 0 at +Y Axis
    base = atan2f( (machinePos[X_AXIS] + xOffset) , (machinePos[Y_AXIS] + yOffset) ) * RadiansToDegrees;
    //float rotBase2 = acosf( machinePos[X_AXIS] / hypTop ) // We could also use the calculated hypotenous instead...
    
    // Ensure our base angle is reachable
    if (base < baseLimits[0] || base > baseLimits[1]) {
        return false;
    }
    
    // Calc side view hypotenousTop & Z -> hypotenousSide (including any z axis offsets)
    const float hypotenousSide = fastSqrtf( fsquare(hypotenousTop) + fsquare(machinePos[Z_AXIS] + zOffset - toolOffsetZ) );
    if (hypotenousSide > maxRadius) {
        // Unreachable (hypotenousSide is longer than our combined shank length)
    	return false;
    }
    const float hypotenousSideSquared = fsquare(hypotenousSide);
    
    // Calculate psi in radians (psi is our distal angle (upper arm / shank))
    psi = -acosf( (proximalArmLengthSquared + distalArmLengthSquared - hypotenousSideSquared) / (twoPD) );
    
    // Calculate theta in radians (theta is our proximal angle (lower arm / shank))
    if ( (machinePos[Z_AXIS] + zOffset - toolOffsetZ) > 0 ) {
        theta = acosf( (machinePos[Z_AXIS] + zOffset - toolOffsetZ) / hypotenousSide ) - acosf( (proximalArmLengthSquared - distalArmLengthSquared + hypotenousSideSquared) / (2 * proximalArmLength * hypotenousSide) );
    } else {
        theta = PI - asinf( hypotenousTop / hypotenousSide ) - acosf( (proximalArmLengthSquared - distalArmLengthSquared + hypotenousSideSquared) / (2 * proximalArmLength * hypotenousSide) );
    }
    
    // Adjust psi and convert to degrees
    psi = (psi + theta) * RadiansToDegrees;
    
    // Convert theta to degrees
    theta = theta * RadiansToDegrees;
    
    // Ensure our angles are reachable
    if (psi < psiLimits[0] || psi > psiLimits[1] || theta < thetaLimits[0] || theta > thetaLimits[1]) {
        return false;
    }
    
    // TODO: Ensure Psi isn't within 30deg of our theta else we'll collide
    if ((theta - psi) < 30 || (theta - psi) > 135) {
        debugPrintf("WARNING! Psi <-> Theta collision detected. psi = %.2f, theta = %.2f\n", (double)psi, (double)theta);
        return false;
    }
    
    //debugPrintf("Calculated Psi = %.2f, Theta = %.2f, Base = %.2f\n", (double)psi, (double)theta, (double)base);

    // Save the original and transformed coordinates so that we don't need to calculate them again if we are commanded to move to this position
    cachedProximalAngle = theta;
    cachedDistalAngle = psi;
    cachedBaseAngle = base;
    cachedHypotenousTop = hypotenousTop;
    cachedX = machinePos[X_AXIS];
    cachedY = machinePos[Y_AXIS];
    cachedZ = machinePos[Z_AXIS];
    
    return true;
}

// Convert Cartesian coordinates to motor coordinates, returning true if successful
// In the following, theta is the proximal arm angle relative to the X axis, psi is the distal arm angle relative to the proximal arm
bool ParallelRobotKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
    float theta, psi, base;
    // If x,y and z match our cached positions, use our cached theta and psi
    if (machinePos[X_AXIS] == cachedX && machinePos[Y_AXIS] == cachedY && machinePos[Z_AXIS] == cachedZ) {
        
        theta = cachedProximalAngle;
        psi = cachedDistalAngle;
        base = cachedBaseAngle;
        
    } else {
        
        if (!CalculateThetaAndPsi(machinePos, isCoordinated, theta, psi, base)) {
            return false;
        }
        
    }
    
    //debugPrintf("psi = %.2f, theta = %.2f\n", psi * RadiansToDegrees, theta * RadiansToDegrees);

    //Set the motor positions based on our inverse kinematic calculations
    // - Note: stepsPerMm is actually stepsPerDegree
    motorPos[X_AXIS] = lrintf(base * stepsPerMm[X_AXIS]);  //Sets the base rotation
    motorPos[Y_AXIS] = lrintf(theta * stepsPerMm[Y_AXIS]); //Sets the lower shank (proximal arm) rotation
    motorPos[Z_AXIS] = lrintf(psi * stepsPerMm[Z_AXIS]);   //Sets the upper shank (distal arm) rotation

    // Transform any additional axes linearly
    for (size_t axis = XYZ_AXES; axis < numVisibleAxes; ++axis) {
        motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
    }
    
    return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// For Scara, the X and Y components of stepsPerMm are actually steps per degree angle.
// motorPos is in steps
// stepsPerMm is in steps/degree
void ParallelRobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
    // This assumes that the Robot Arm uses a rotational base (where should we place our datum)
    const float base = ((float)motorPos[X_AXIS]/stepsPerMm[X_AXIS]);
    // Y Axis motor is bound to our Proximal (Lower Shank) arm
    const float theta = ((float)motorPos[Y_AXIS]/stepsPerMm[Y_AXIS]);
    // Z Axis motor is bound to our Distal (Upper Shank) arm
    const float psi = ((float)motorPos[Z_AXIS]/stepsPerMm[Z_AXIS]);

    // Cache the current values so that a Z probe at this position won't fail due to rounding error when transforming the XY coordinates back
    cachedProximalAngle = theta;
    cachedDistalAngle = psi;
    cachedBaseAngle = base;
    
    const float thetaSin = sinf(theta * DegreesToRadians),
                thetaCos = cosf(theta * DegreesToRadians),
                psiSin   = sinf(  psi * DegreesToRadians),
                psiCos   = cosf(  psi * DegreesToRadians),
                baseSin  = sinf( base * DegreesToRadians),
                baseCos  = cosf( base * DegreesToRadians);
    
    // yOffset is planar to arm kinematics
    const float hypotenousTop = (proximalArmLength * thetaSin) - (distalArmLength * psiSin) + yOffset + toolOffsetY;
    cachedHypotenousTop = hypotenousTop;
    
    cachedX = machinePos[X_AXIS] = hypotenousTop * baseSin;
    cachedY = machinePos[Y_AXIS] = hypotenousTop * baseCos;
    cachedZ = machinePos[Z_AXIS] = (proximalArmLength * thetaCos) - (distalArmLength * psiCos) + zOffset + toolOffsetZ;
    
    // Convert any additional axes linearly
    for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive) {
        machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
    }
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters that affect the geometry. Set 'error' true if there was an error, otherwise leave it alone.
bool ParallelRobotKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) /*override*/
{
    if (mCode == 669)
    {
        const bool seenNonGeometry = TryConfigureSegmentation(gb);  // configure optional segmentation

        bool seen = false;
        gb.TryGetFValue('P', proximalArmLength, seen);
        gb.TryGetFValue('D', distalArmLength, seen);
        gb.TryGetFValue('X', xOffset, seen);
        gb.TryGetFValue('Y', yOffset, seen);
        gb.TryGetFValue('Z', zOffset, seen);
        gb.TryGetFValue('H', toolOffsetY, seen);
        gb.TryGetFValue('I', toolOffsetZ, seen);
        if (gb.TryGetFloatArray('A', 2, thetaLimits, reply, seen))
        {
            error = true;
            return true;
        }
        if (gb.TryGetFloatArray('B', 2, psiLimits, reply, seen))
        {
            error = true;
            return true;
        }
        if (gb.TryGetFloatArray('C', 2, baseLimits, reply, seen))
        {
            error = true;
            return true;
        }
        //gb.TryGetFValue('R', requestedMinRadius, seen);

        if (seen)
        {
            Recalc();
        }
        else if (!seenNonGeometry && !gb.Seen('K'))
        {
            reply.printf("Kinematics is Parallel Robot Arm, proximal length %.2fmm range %.1f" DEGREE_SYMBOL " to %.1f" DEGREE_SYMBOL
                            ", distal length %.2fmm range %.1f" DEGREE_SYMBOL " to %.1f" DEGREE_SYMBOL ", base from %.1f" DEGREE_SYMBOL " to %.1f" DEGREE_SYMBOL
							", bed origin (%.1f, %.1f), seg/sec %.1f, min seg length %.2fmm, Ty:%.1f, Tz:%.1f",
                            (double)proximalArmLength, (double)thetaLimits[0], (double)thetaLimits[1],
                            (double)distalArmLength, (double)psiLimits[0], (double)psiLimits[1],
                            (double)baseLimits[0], (double)baseLimits[1],
                            (double)xOffset, (double)yOffset,
                            (double)GetSegmentsPerSecond(), (double)GetMinSegmentLength(),
							(double)toolOffsetY, (double)toolOffsetZ);
        }
        return seen;
    }
    return false;
}

// Return true if the specified XY position is reachable by the print head reference point, ignoring M208 limits.
bool ParallelRobotKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes, bool isCoordinated) const noexcept
{
    if (axes.IsBitSet(X_AXIS) && axes.IsBitSet(Y_AXIS) && axes.IsBitSet(Z_AXIS))
    {
        // See if we can transform the position
        float coords[3] = {axesCoords[X_AXIS], axesCoords[Y_AXIS], axesCoords[Z_AXIS]};
        float theta, psi, base;
        if (!CalculateThetaAndPsi(coords, isCoordinated, theta, psi, base))
        {
            return false;
        }
    }
    axes.ClearBit(X_AXIS);
    axes.ClearBit(Y_AXIS);
    axes.ClearBit(Z_AXIS);
    return true;
    //return Kinematics::IsReachable(axesCoords, axes, isCoordinated); // I think this is the scara asking the leadscrew if the z height is reachable...
}

// Limit the Cartesian position that the user wants to move to, returning true if any coordinates were changed
LimitPositionResult ParallelRobotKinematics::LimitPosition(float finalCoords[], const float * null initialCoords,
                                                    size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
    // First limit all axes according to M208
    bool limited = applyM208Limits && Kinematics::LimitPositionFromAxis(finalCoords, 0, numVisibleAxes, axesToLimit);

    // Now check whether the arms can reach the final position
    //float theta, psi, base;
    //if (!CalculateThetaAndPsi(finalCoords, isCoordinated, theta, psi, base)) {
    	// Then we can't reach the position, lets find the culprit and determine how close we can get...
    	//if (cachedHypotenousTop > )
    //}

    return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void ParallelRobotKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	// We should calculate the EE at our minimum limit angles
    positions[X_AXIS] = -xOffset;
    positions[Y_AXIS] = -yOffset;
    positions[Z_AXIS] = -zOffset;
    for (size_t i = Z_AXIS + 1; i < numAxes; ++i)
    {
        positions[i] = 0.0;
    }
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap ParallelRobotKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
//    // If both X and Y have been specified then we know the positions of both arm motors, otherwise we don't
//    if ((g92Axes & XyAxes) != XyAxes)
//    {
//        g92Axes &= ~XyAxes;
//    }
    return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap ParallelRobotKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
//    if (axesMoving.Intersects(XyAxes))
//    {
//        axesMoving |= XyAxes;
//    }
    disallowMovesBeforeHoming = true;
    return axesMoving;
}

size_t ParallelRobotKinematics::NumHomingButtons(size_t numVisibleAxes) const noexcept
{
#if HAS_MASS_STORAGE
    const Platform& platform = reprap.GetPlatform();
    if (!platform.SysFileExists(HomeProximalFileName))
    {
        return 0;
    }
    if (!platform.SysFileExists(HomeDistalFileName))
    {
        return 1;
    }
    if (!platform.SysFileExists(HomeBaseFileName))
    {
        return 2;
    }
#endif
    return numVisibleAxes;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap ParallelRobotKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
    // Ask the base class which homing file we should call first
    const AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);

    if (ret.IsEmpty())
    {
        // Change the returned name if it is X or Y
        if (StringEqualsIgnoreCase(filename.c_str(), "homex.g"))
        {
            filename.copy(HomeBaseFileName);
        }
        if (StringEqualsIgnoreCase(filename.c_str(), "homey.g"))
        {
            filename.copy(HomeProximalFileName);
        }
        else if (StringEqualsIgnoreCase(filename.c_str(), "homez.g"))
        {
            filename.copy(HomeDistalFileName);
        }

#if HAS_MASS_STORAGE
        // Some SCARA printers cannot have individual axes homed safely. So if the user doesn't provide the homing file for an axis, default to homeall.
        if (!reprap.GetPlatform().SysFileExists(filename.c_str()))
        {
            filename.copy(HomeAllFileName);
        }
#endif
    }
    return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool ParallelRobotKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
    // If crosstalk causes the axis motor concerned to affect other axes then must terminate the entire move
    //return (axis == X_AXIS && (crosstalk[0] != 0.0 || crosstalk[1] != 0.0))
    //    || (axis == Y_AXIS && crosstalk[2] != 0.0);
    return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate().
void ParallelRobotKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
{
    switch (axis)
    {
    case X_AXIS:    // base axis homing switch
        {
            const float hitPoint = (highEnd) ? baseLimits[1] : baseLimits[0];
            dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis); // -90deg * 35 steps/deg ->
            //dda.SetDriveCoordinate(lrintf(0), axis);
        }
        break;

    case Y_AXIS:    // proximal joint homing switch
        {
            const float hitPoint = (highEnd) ? thetaLimits[1] : thetaLimits[0];
            dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis); //-51.8deg * 35 steps/deg
            //dda.SetDriveCoordinate(lrintf(0), axis);
        }
        break;

    case Z_AXIS:    // distal joint homing switch
        {
            const float hitPoint = (highEnd) ? psiLimits[1] : psiLimits[0];
            dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis); //-120deg * 35 steps/deg
            //dda.SetDriveCoordinate(lrintf(0), axis);
        }
        break;

    default:        // Additional axis
        {
            const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
            dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
        }
        break;
    }
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void ParallelRobotKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept
{
    //Do we limit yz factor then use that to limit xh factor??

//    // For now we limit the speed in the XY plane to the lower of the X and Y maximum speeds, and similarly for the acceleration.
//    // Limiting the angular rates of the arms would be better.
//    const float xyFactor = fastSqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
//    if (xyFactor > 0.01)
//    {
//        const Platform& platform = reprap.GetPlatform();
//        const float maxSpeed = min<float>(platform.MaxFeedrate(X_AXIS), platform.MaxFeedrate(Y_AXIS));
//        const float maxAcceleration = min<float>(platform.Acceleration(X_AXIS), platform.Acceleration(Y_AXIS));
//        dda.LimitSpeedAndAcceleration(maxSpeed/xyFactor, maxAcceleration/xyFactor);
//    }
}

// Return true if the specified axis is a continuous rotation axis
bool ParallelRobotKinematics::IsContinuousRotationAxis(size_t axis) const noexcept
{
    return false;
    //return (axis < 2 && supportsContinuousRotation[axis]) || Kinematics::IsContinuousRotationAxis(axis);
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap ParallelRobotKinematics::GetLinearAxes() const noexcept
{
    return AxesBitmap();
    //return (crosstalk[1] == 0.0 && crosstalk[2] == 0.0) ? AxesBitmap::MakeFromBits(Z_AXIS) : AxesBitmap();
}

// Recalculate the derived parameters
void ParallelRobotKinematics::Recalc() noexcept
{
    proximalArmLengthSquared = fsquare(proximalArmLength);
    distalArmLengthSquared = fsquare(distalArmLength);
    twoPD = proximalArmLength * distalArmLength * 2;

    minRadius = max<float>(fastSqrtf(proximalArmLengthSquared + distalArmLengthSquared
                            + twoPD * min<float>(cosf(psiLimits[0] * DegreesToRadians), cosf(psiLimits[1] * DegreesToRadians))) * 1.005,
                            requestedMinRadius);
    minRadiusSquared = fsquare(minRadius);

    //const float minAngle = min<float>(fabsf(psiLimits[0]), fabsf(psiLimits[1])) * DegreesToRadians;
    //maxRadius = fastSqrtf(proximalArmLengthSquared + distalArmLengthSquared); //+ (twoPD * cosf(minAngle)));
    const float minAngle = 35 * DegreesToRadians;
    maxRadius = (cosf(minAngle) * proximalArmLength) + (cosf(minAngle) * distalArmLength);

    maxRadius *= 0.995;

    cachedX = cachedY = cachedZ = std::numeric_limits<float>::quiet_NaN();        // make sure that the cached values won't match any coordinates
    cachedProximalAngle = cachedDistalAngle = cachedBaseAngle = std::numeric_limits<float>::quiet_NaN();
}

// End
