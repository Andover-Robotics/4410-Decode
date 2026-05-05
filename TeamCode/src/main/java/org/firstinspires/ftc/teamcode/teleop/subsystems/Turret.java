package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.MainTeleop;
import org.firstinspires.ftc.teamcode.util.LinearInterpolation;

import java.util.ArrayList;

@Config
public class Turret {
    private final MotorEx motor;
    private final PIDController largeErrorController;
    private final PIDController smallErrorController;
    public PIDController activeController;
    private final ElapsedTime timer = new ElapsedTime();

    public Shooter shooter;

    public static boolean shooterActive = true, positionTracking = true;
//    public static double goalX = 62;
//    public static double goalY = 60;

    public static double POS_TRACK_X = 0;
    public static double POS_TRACK_Y = 0;
    public static double TURRET_OFFSET_BACK_IN = 1; // inches back from robot center
    public static double rapidFireDistanceThresholdIn = 122;
    public static double rapidFireSleepScalePerIn = 0.004;
    public static double
            largeP = 0.0062, largeI = 0, largeD = 0.00035,
            smallP = 0.008 , smallI = 0, smallD = 0.0005,
            errorThresholdDeg = 650, manualPower = 0,
            targetVelK = 0.0036, targetAccelK = 0.00000,
            staticF = 0.05, staticFErrorDeg = 0.50;

    public static double feedforwardPower, velFFPower, accelFFPower, staticFFPower;

    private double tolerance = 1, powerMin = 0.05, degsPerTick = 360.0 / (145.1 * 104.0/10.0), ticksPerRev = 360 / degsPerTick;

    public static final double[] SHOOTER_DISTANCE_IN = {
            30, 32.5, 35, 37.5, 40, 42.5, 45, 47.5, 50, 52.5, 55, 57.5, 60, 62.5, 65, 67.5, 70, 72.5, 75, 77.5, 80, 82.5, 85, 87.5, 90, 92.5, 95, 97.5, 100, 102.5, 105, 107.5, 110, 112.5, 115, 117.5, 120, 122.5, 125, 127.5, 130, 132.5, 135, 137.5, 140, 142.5, 145, 147.5, 150.0, 152.5, 155};
    public static final double[] SHOOTER_RPM = {
            2650, 2655, 2656, 2660, 2665, 2670, 2690, 2700, 2750, 2800, 2890, 2970, 3040, 3100, 3130, 3180, 3220, 3250, 3310, 3350, 3375, 3400, 3420, 3510, 3550, 3585, 3620, 3655, 3710, 3755, 3790, 3820, 3885, 3935, 4000, 4010, 4020, 4040, 4090, 4130, 4190, 4240, 4260, 4290, 4330, 4350, 4380, 4410, 4440, 4470, 4510};
    public static final double[] SHOOTER_HOOD_ANGLE_DEG = {
            35.000, 35.000, 35.000, 35.469, 35.938, 36.406, 36.875, 37.344, 37.813, 38.281, 38.750, 39.219, 39.688, 40.156, 40.625, 41.094, 41.563, 42.031, 42.500, 42.969, 43.438, 43.906, 44.375, 44.844, 45.313, 45.781, 46.250, 46.719, 47.188, 47.656, 48.125, 48.594, 49.063, 49.531, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000, 50.000
    };

    private final LinearInterpolation rpmInterpolator;
    private final LinearInterpolation hoodAngleInterpolator;

    public double power, lastTime, setPoint = 0, pos = 0, highLimit = 210, lowLimit = -160, trackingTarget, runToTargetAngle;
    private double previousTargetTicks = 0, previousTargetVelDegPerSec = 0;
    private int cachedPositionTicks = 0;

    public static double shooterRpm = 0, trackingDistance, pureDistance, error, currentPosDegs;

    public ArrayList<Double> txArr, tyArr;

    public static boolean velComp = true, shooterOverride = false, deadzone = false;

    public Pose2d pose;
    public PoseVelocity2d velocity;
    private final SRSHubs srsHubs;

    public Turret(OpMode opMode, SRSHubs srsHubs) {
        this.srsHubs = srsHubs;
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        largeErrorController = new PIDController(largeP, largeI, largeD);
        smallErrorController = new PIDController(smallP, smallI, smallD);
        largeErrorController.setTolerance(0);
        smallErrorController.setTolerance(0);
        largeErrorController.setSetPoint(0);
        smallErrorController.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        shooter = new Shooter(opMode, srsHubs);
        rpmInterpolator = new LinearInterpolation(SHOOTER_DISTANCE_IN, SHOOTER_RPM);
        hoodAngleInterpolator = new LinearInterpolation(SHOOTER_DISTANCE_IN, SHOOTER_HOOD_ANGLE_DEG);

        timer.reset();
        lastTime = timer.seconds();
        txArr = new ArrayList<>(0);
        tyArr = new ArrayList<>(0);
    }

    public void enableFullAuto(boolean on) {
        enableAutoAim(on);
        enableShooter(on);
    }

    public void enableAutoAim(boolean on) {
        enablePositionTracking(on);
    }

    public void enableShooter(boolean enable) {
        shooterActive = enable;
    }

    public void enablePositionTracking(boolean enable) {
        positionTracking = enable;
    }

    public boolean shooterInRange() {
        return shooterActive && shooter.inRange();
    }

    public void runToAngle(double angle) {
        if (getPositionDegs() > 0 && ((angle + 360) % 360) > highLimit ) {
            angle = angle - 360;
        } else if (angle < lowLimit) {
            angle = angle + 360;
        } else if (getPositionDegs() > 0 && angle > lowLimit) {
            angle = (angle + 360) % 360;
        }
        angle = Math.min(Math.max(lowLimit, angle), highLimit);
        runToTargetAngle = angle;
        int t = (int) ((angle) / degsPerTick);
        runTo(t);
    }

    private void runTo(int t) { //takes in ticks
        setPoint = t;
    }

    public void runManual(double manual) {
        if (manual > powerMin || manual < -powerMin) {
            manualPower = manual;
        } else {
            manualPower = 0;
        }
    }

    // Normalize to [-180, 180)
    private static double normDeg(double a) {
        return ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    /**
     * Aim the turret at a fixed field point using the robot's live pose.
     *  - Field/robot headings are CCW positive.
     *  - Turret encoder angles are CW positive.
     *  - Turret zero is 180° (backwards) from robot-forward.
     *
     * steps:
     *  1) fieldAngleCCW = atan2(dy, dx)
     *  2) relToRobotCCW = fieldAngleCCW - robotHeadingCCW
     *  3) turretTargetCW = -relToRobotCCW + TURRET_ZERO_CW_OFFSET
     */
    private double aimAtGlobalPoint(double targetX, double targetY) {
        pose = Bot.storedPose;

        // Robot heading in radians (CCW+)
        double headingRad = pose.heading.log();

        // Turret position in field frame:
        // "back" = negative X in robot frame, rotated into field frame
        double turretX = pose.position.x - TURRET_OFFSET_BACK_IN * Math.cos(headingRad);
        double turretY = pose.position.y - TURRET_OFFSET_BACK_IN * Math.sin(headingRad);

        // Vector from turret to target in field frame
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        velocityCompensation(dx, dy);

        // Field-bearing CCW to target
        double fieldAngleCCW = Math.toDegrees(Math.atan2(POS_TRACK_Y, POS_TRACK_X)); // CCW+

        // Robot heading CCW
        double robotHeadingCCW = Math.toDegrees(headingRad); // CCW+

        // Robot-relative CCW angle to target
        double relToRobotCCW = normDeg(fieldAngleCCW - robotHeadingCCW);

        // Turret is CW-positive and zero is backwards, so:
        // 0° turret = 180° robot-relative CCW
//        double turretTargetCW = normDeg(-relToRobotCCW + 180);

        double turretTargetCW = normDeg(-relToRobotCCW);

        pureDistance = Math.sqrt(dx*dx + dy*dy);

        // Tracking distance from turret to goal
        trackingDistance = Math.sqrt(POS_TRACK_X * POS_TRACK_X + POS_TRACK_Y * POS_TRACK_Y);

        return turretTargetCW;
    }

    public void velocityCompensation(double dx, double dy) {
        double currentTurretDegs = pos * degsPerTick;
        if (currentTurretDegs < highLimit - 5 && currentTurretDegs > lowLimit + 5) {
            double time = calculateTime(dx, dy);
            velocity = Bot.drive.localizer.getPoseVelocity();
//        double dispX = velocity.linearVel.x * time;
//        double dispY = velocity.linearVel.y * time;
//        POS_TRACK_X = dx + dispX;
//        POS_TRACK_Y = dy + dispY;
            double heading = pose.heading.log();

            // Convert robot-centric velocity to field frame
            double velocityXField = velocity.linearVel.x * Math.cos(heading) - velocity.linearVel.y * Math.sin(heading);
            double velocityYField = velocity.linearVel.x * Math.sin(heading) + velocity.linearVel.y * Math.cos(heading);

            // Offset the target opposite the robot's drift so that the added launch
            // velocity from the robot's motion lands on the goal.
            double dispX = velocityXField * time;
            double dispY = velocityYField * time;
            if (velComp) {
                POS_TRACK_X = dx - dispX;
                POS_TRACK_Y = dy - dispY;
            } else {
                POS_TRACK_X = dx;
                POS_TRACK_Y = dy;
            }
        }
    }

    public void setVelComp(boolean i) {
        velComp = i;
    }

    public double calculateTime(double dx, double dy) {
        // Constants
        double G = 386.09;                 // in/s^2 (gravity in inches)
        double heightDisplacement = 26.0;  // inches (Δz)
        double launchAngleAboveHorizDeg = 90 - shooter.getHoodAngle();  // (90 degrees - actual shooter angle) -> makes the angle relative to horizontal plane
        double launchAngleRad = Math.toRadians(launchAngleAboveHorizDeg);

        // Horizontal distance (XY plane)
        double R = Math.sqrt(dx * dx + dy * dy);

        // t^2 = (2/g) * (R * tan(theta) - Δz)
        double term = R * Math.tan(launchAngleRad) - heightDisplacement;
        double tSquared = (2.0 / G) * term;

        if (tSquared <= 0) {
            // No ballistic solution (target too low for given angle)
            return 0;  // or any fallback (0 means “no adjustment”)
        }

        double t = Math.sqrt(tSquared);
        return t;
    }

    public void periodic() {
        power = 0;
        PoseVelocity2d poseVelocity = Bot.drive.localizer.getPoseVelocity();
        double robotSpeedInPerSec = 0.0;
        if (poseVelocity != null) {
            robotSpeedInPerSec = Math.hypot(poseVelocity.linearVel.x, poseVelocity.linearVel.y);
        }
        shooter.setRobotVelocityInPerSec(robotSpeedInPerSec);

        cachedPositionTicks = motor.getCurrentPosition();
        pos = cachedPositionTicks;
        double now = timer.seconds();
        double deltaTime = Math.max(1e-3, now - lastTime);
        currentPosDegs = getPositionDegs();

//        // position tracking mode
        if (positionTracking) {
            trackingTarget = aimAtGlobalPoint(Bot.targetPose.x, Bot.targetPose.y);
            runToAngle(trackingTarget);
////            runToAngle(aimAtGlobalPoint(goalX, goalY));
            double errorDeg = Math.abs((setPoint - pos) * degsPerTick);
            activeController = smallErrorController;
            activeController.setPID(smallP, smallI, smallD);
            activeController.setSetPoint(setPoint);
//
            double targetVelDegPerSec = ((setPoint - previousTargetTicks) * degsPerTick) / deltaTime;
//
            velFFPower = targetVelK * targetVelDegPerSec;

            staticFFPower = 0;
            if (Math.abs(activeController.getPositionError() * degsPerTick) > staticFErrorDeg) {
                staticFFPower = staticF * Math.signum(activeController.getPositionError());
            }

            feedforwardPower = velFFPower + staticFFPower;
            power = (activeController.calculate(pos) + feedforwardPower);

            previousTargetTicks = setPoint;
            previousTargetVelDegPerSec = targetVelDegPerSec;
            double overlap = (highLimit - lowLimit) - 360;
            deadzone = (getPositionDegs()) > (highLimit - overlap - 10) || (getPositionDegs()) < ((lowLimit + overlap + 13));
        } else {
            power = manualPower;
            previousTargetTicks = setPoint;
            previousTargetVelDegPerSec = 0;
        }

        error = activeController.getPositionError() * degsPerTick;

        double maxPower = 1;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        shooterRpm = rpmInterpolator.interpolate(trackingDistance);
        double hoodAngleDeg = hoodAngleInterpolator.interpolate(trackingDistance);

        if (MainTeleop.manualTurret) {
            shooterRpm = 3000;
        }

        if (shooterActive && !shooterOverride) {
            shooter.setVelocity(shooterRpm);
            shooter.setHoodAngleDeg(hoodAngleDeg);
        } else if (!shooterActive) {
            if (shooter.getPower() != 0) {
                shooter.setPower(0);
            }
        }
        shooter.periodic(); //todo small spikes

        motor.set(power);
        lastTime = now;
    }

    public void setShooterVelocity(double rpm) {
        shooter.setVelocity(rpm);
    }

    public void setShooterOverride(boolean override) {
        shooterOverride = override;
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }

    public static double getRapidShootSleep(double baseSleepSeconds) {
        double extraDistance = Math.max(0.0, trackingDistance - rapidFireDistanceThresholdIn);
        return baseSleepSeconds + (extraDistance * rapidFireSleepScalePerIn);
    }

    public int getPosition() {
        return cachedPositionTicks;
    }

    public double getPositionDegs() {
        return getPosition() * degsPerTick;
    }

    public double getErrorDegs() {
        return (activeController.getPositionError()) * degsPerTick;
    }

    public double getPositionTicks() {
        return getPosition();
    }

    public double getTargetTicks() {
        return setPoint;
    }

    public double getTargetDegs() {
        return setPoint * degsPerTick;
    }

    public double getPower() {
        return power;
    }


    public PIDController getLargeController() {
        return largeErrorController;
    }
    public PIDController getSmallController() {
        return smallErrorController;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
