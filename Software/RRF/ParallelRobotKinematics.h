//
//  ParallelRobotKinematics.h
//  
//
//  Created by Brandon Toms on 8/4/21.
//

#ifndef ParallelRobotArm_h
#define ParallelRobotArm_h

#include "Kinematics.h"

// Can we use the SCARA implementation but swap the X & Y axes for Y & Z. Then incorporate the X axis in via RoundBed/Polar Kinematics or something

// Standard setup for SCARA machines assumed by this firmware
// The X motor output drives the proximal arm joint, unless remapped using M584
// The Y motor output drives the distal arm joint, unless remapped using M584
// Forward motion of a motor rotates the arm anticlockwise as seen from above (use M569 to reverse it)
// Theta is the angle of the proximal arm joint from the reference position.
// At theta = 0 the proximal arm points in the Cartesian +X direction
// Phi is the angle of the distal arm relative to the Cartesian X axis. Therefore the angle of the distal arm joint is (phi - theta).
// The M92 steps/mm settings for X and Y are interpreted as steps per degree of theta and phi respectively.

class ParallelRobotKinematics : public Kinematics
{
public:
    // Constructors
	ParallelRobotKinematics() noexcept;

    // Overridden base class functions. See Kinematics.h for descriptions.
    const char *GetName(bool forStatusReport) const noexcept override;
    bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
    bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
    void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
    bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes, bool isCoordinated) const noexcept override;
    LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
    void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
    size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override;
    const char* HomingButtonNames() const noexcept override { return "PDZUVWABC"; }
    HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
    AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
    AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
    AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
    bool QueryTerminateHomingMove(size_t axis) const noexcept override;
    void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
    void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
    bool IsContinuousRotationAxis(size_t axis) const noexcept override;
    AxesBitmap GetLinearAxes() const noexcept override;

protected:
    DECLARE_OBJECT_MODEL

private:
    static constexpr float DefaultProximalArmLength = 140.0; // Lower Shank Length
    static constexpr float DefaultDistalArmLength = 140.0;   // Upper Shank Length
    static constexpr float DefaultMinTheta = -50;            // Minimum proximal joint angle (-52.75 endstop location)
    static constexpr float DefaultMaxTheta = 90.0;           // Maximum proximal joint angle
    static constexpr float DefaultMinPsi = -115.0;           // Minimum distal joint angle (-118.7 endstop location)
    static constexpr float DefaultMaxPsi = 35.0;             // Maximum distal joint angle
    static constexpr float DefaultMinBase = -90.0;           // Minimum base joint angle (-95.6 endstop location)
    static constexpr float DefaultMaxBase = 90.0;            // Maximum base joint angle
    static constexpr float PI = 3.14159274101257324219;      // PI
    static constexpr float DefaultToolOffsetY = 0;
    static constexpr float DefaultToolOffsetZ = 0;

    // Combination Arm Homing (our proximal joint can be limited by the distal, and vice versa. If we drive both arms backwards (towards their endstop) at the same rate & time, then we don't need to worry about collisions)
    // static constexpr conts char *HomeArmFileName = "homearm.g"
    static constexpr const char *HomeProximalFileName = "homeproximal.g";
    static constexpr const char *HomeDistalFileName = "homedistal.g";
    static constexpr const char *HomeBaseFileName = "homebase.g";

    void Recalc() noexcept;
    bool CalculateThetaAndPsi(const float machinePos[], bool isCoordinated, float& theta, float& psi, float& base) const noexcept;

    // Primary parameters
    float proximalArmLength;                         // Lower Shank Length
    float distalArmLength;                           // Upper Shank Length
    float thetaLimits[2];                            // Min and max Proximal joint angles
    float psiLimits[2];                              // Min and max Distal joint angles
    float baseLimits[2];							 // Min and max Base angles
    float xOffset;                                   // Where bed X=0 is relative to the proximal joint
    float yOffset;                                   // Where bed Y=0 is relative to the proximal joint
    float zOffset;                                   // Where bed Z=0 is relative to the proximal joint
    float requestedMinRadius;                        // Requested minimum radius
    float toolOffsetY;								 // This is the horizontal tool offset on the YZ Plane (arm axis)
    float toolOffsetZ;								 // This is the vertical tool offset on the YZ Plane (arm axis)

    // Derived parameters
    float minRadius;
    float maxRadius;
    float minRadiusSquared;
    float proximalArmLengthSquared;
    float distalArmLengthSquared;
    float twoPD;

    // State variables
    mutable float cachedX, cachedY, cachedZ;
    mutable float cachedHypotenousTop; // sqrt(proximal^2 + distal^2) - Note: We have to cache the hypotenous because the IK is dependent on Hypotenous Length and Z Height
    mutable float cachedProximalAngle, cachedDistalAngle, cachedBaseAngle;
};


#endif /* ThreeAxisParallelRobotArm_h */
