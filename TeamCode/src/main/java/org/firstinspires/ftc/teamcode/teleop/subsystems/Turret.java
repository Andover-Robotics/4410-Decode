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
    public static double rapidFireDistanceThresholdIn = 95;
    public static double rapidFireSleepScalePerIn = 0.0032;
    public static double
            largeP = 0.009, largeI = 0, largeD = 0.0003,
            smallP = 0.018 , smallI = 0, smallD = 0.0003,
            errorThresholdDeg = 4, manualPower = 0,
            targetVelK = 0.0032, targetAccelK = 0.00000;

    public static double feedforwardPower, velFFPower, accelFFPower;

    private double tolerance = 1, powerMin = 0.05, degsPerTick = 360.0 / (145.1 * 104.0/10.0), ticksPerRev = 360 / degsPerTick;

    public static final double[] SHOOTER_DISTANCE_IN = {
            30.0, 32.5, 35.0, 37.5, 40.0, 42.5, 45.0, 47.5, 50.0, 52.5,
            55.0, 57.5, 60.0, 62.5, 65.0, 67.5, 70.0, 72.5, 75.0, 77.5,
            80.0, 82.5, 85.0, 87.5, 90.0, 92.5, 95.0, 97.5, 100.0, 102.5,
            105.0, 107.5, 110.0, 112.5, 115.0, 117.5, 120.0, 122.5, 125.0,
            127.5, 130.0, 132.5, 135.0, 137.5, 140.0, 142.5, 145.0, 147.5, 150
    };

    public static final double[] SHOOTER_RPM = {
            2825, 2845, 2860, 2880, 2890, 2900, 2915, 2935, 3000, 3050,
            3150, 3200, 3250, 3280, 3310, 3320, 3390, 3440, 3480, 3525,
            3565, 3610, 3650, 3700, 3750, 3790, 3840, 3870, 3910, 3950,
            3980, 3990, 4020, 4040, 4080, 4110, 4140, 4180, 4220, 4260,
            4300, 4340, 4370, 4400, 4430, 4470, 4500, 4530, 4570
//
//            ,2825, 2845, 2860, 2880, 2890, 2900, 2915, 2935, 3000, 3050, old inter
//            3150, 3200, 3250, 3280, 3310, 3340, 3390, 3430, 3470, 3510,
//            3550, 3590, 3640, 3690, 3740, 3740, 3800, 3850, 3900, 3940,
//            3980, 4010, 4050, 4080, 4135, 4190, 4220, 4205, 4210, 4235,
//            4250, 4270, 4300, 4315, 4360, 4390, 4425
    };

    public static final double[] SHOOTER_HOOD_ANGLE_DEG = {
            32.500, 32.500, 32.500, 33.023, 33.545, 34.068, 34.591, 35.114, 35.636, 36.159,
            36.682, 37.205, 37.727, 38.250, 38.773, 39.295, 39.818, 40.341, 40.864, 41.386,
            41.909, 42.432, 42.955, 43.477, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000,
            44.000, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000,
            44.000, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000, 44.000
    };

    private double lastVXField = 0.0, lastVYField = 0.0;
    private double lastTimeSec = Double.NaN;

    private double aXFieldFilt = 0.0, aYFieldFilt = 0.0;

    // Tuning knobs
    private static final double ACCEL_ALPHA = 0.20;   // accel low-pass (0..1)
    private static final double T_PRELAUNCH_SEC = 0.20; // "now" -> actual release delay (in seconds)
    private static final double ACCEL_CLAMP = 130.0;  // B) clamp accel to ±130 in/s^2


    private final LinearInterpolation rpmInterpolator;
    private final LinearInterpolation hoodAngleInterpolator;

    public double power, lastTime, setPoint = 0, pos = 0, highLimit = 220, lowLimit = -150;
    private double previousTargetTicks = 0, previousTargetVelDegPerSec = 0;
    private int cachedPositionTicks = 0;

    public static double shooterRpm = 0, trackingDistance, pureDistance;

    public ArrayList<Double> txArr, tyArr;

    public static boolean velComp = true, shooterOverride = false;

    public Pose2d pose;
    public PoseVelocity2d velocity;

    public Turret(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        largeErrorController = new PIDController(largeP, largeI, largeD);
        smallErrorController = new PIDController(smallP, smallI, smallD);
        largeErrorController.setTolerance(tolerance);
        smallErrorController.setTolerance(tolerance);
        largeErrorController.setSetPoint(0);
        smallErrorController.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        shooter = new Shooter(opMode);
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
        if (angle > highLimit) {
            angle = angle - 360;
        } else if (angle < lowLimit) {
            angle = angle + 360;
        }
        angle = Math.min(Math.max(lowLimit, angle), highLimit);
        int t = (int) ((angle) / degsPerTick);
        runTo(t);
    }

    private void runTo(int t) { //takes in ticks
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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
        final double G = 386.09;                 // in/s^2 (gravity in inches)
        final double heightDisplacement = 26.0;  // inches (Δz)
        final double launchAngleAboveHorizDeg = 52.0;  // (90 degrees - actual shooter angle) -> makes the angle relative to horizontal plane
        final double launchAngleRad = Math.toRadians(launchAngleAboveHorizDeg);

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
//

    public void periodic() {
        power = 0;
        cachedPositionTicks = motor.getCurrentPosition();
        pos = cachedPositionTicks;
        double now = timer.seconds();
        double deltaTime = Math.max(1e-3, now - lastTime);

        // position tracking mode
        if (positionTracking) {
            runToAngle(aimAtGlobalPoint(Bot.targetPose.x, Bot.targetPose.y));
//            runToAngle(aimAtGlobalPoint(goalX, goalY));
            double errorDeg = Math.abs((setPoint - pos) * degsPerTick);
            activeController = errorDeg > errorThresholdDeg ? largeErrorController : smallErrorController;
            if (activeController == largeErrorController) {
                activeController.setPID(largeP, largeI, largeD);
            } else {
                activeController.setPID(smallP, smallI, smallD);
            }
            activeController.setSetPoint(setPoint);

            double targetVelDegPerSec = ((setPoint - previousTargetTicks) * degsPerTick) / deltaTime;
            double targetAccelDegPerSec2 = (targetVelDegPerSec - previousTargetVelDegPerSec) / deltaTime;

            velFFPower = targetVelK * targetVelDegPerSec;
            accelFFPower = targetAccelK * targetAccelDegPerSec2;

            feedforwardPower = velFFPower + accelFFPower;
            power = activeController.calculate(pos) + feedforwardPower;

            previousTargetTicks = setPoint;
            previousTargetVelDegPerSec = targetVelDegPerSec;
        } else {
            power = manualPower;
            previousTargetTicks = setPoint;
            previousTargetVelDegPerSec = 0;
        }

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
            shooter.setPower(0);
        }
        shooter.periodic();

        motor.set(power * 13.5 / Bot.getBatteryVoltage());
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


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
