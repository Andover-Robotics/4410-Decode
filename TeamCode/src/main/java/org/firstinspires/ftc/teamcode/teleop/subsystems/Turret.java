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
    public static double rapidFireDistanceThresholdIn = 250;
    public static double rapidFireSleepScalePerIn = 0.009584479 ;
    public static double
            largeP = 0.006, largeI = 0, largeD = 0.0003,
            smallP = 0.017 , smallI = 0, smallD = 0.0004,
            errorThresholdDeg = 4, manualPower = 0;

    private double tolerance = 1, powerMin = 0.05, degsPerTick = 360.0 / (145.1 * 104.0/10.0), ticksPerRev = 360 / degsPerTick;
    public static double shooterLowF = -4383.53086, shooterLowG = -0.00733324, shooterLowH = 1.81436, shooterLowI = 8284.3436;
    public static double shooterMidF = -4440.0, shooterMidG = -0.0072, shooterMidH = 1.85, shooterMidI = 8340.0;
    public static double shooterHighF = -4500.0, shooterHighG = -0.0071, shooterHighH = 1.9, shooterHighI = 8400.0;
    private double shooterF = shooterLowF, shooterG = shooterLowG, shooterH = shooterLowH, shooterI = shooterLowI;

    public double power, lastTime, setPoint = 0, pos = 0, highLimit = 235, lowLimit = -135;

    public static double shooterRpm = 0, trackingDistance, pureDistance;

    public ArrayList<Double> txArr, tyArr;

    private boolean velComp = true, shooterOverride = false;

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
        if (getPositionDegs() < highLimit - 10 && getPositionDegs() > lowLimit + 10) {
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
        final double launchAngleAboveHorizDeg = 48.0;  // (90 degrees - actual shooter angle) -> makes the angle relative to horizontal plane
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

    public void periodic() {
        power = 0;
        pos = getPosition();
        // Early-out: position tracking mode
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
            power = activeController.calculate(pos);
        } else {
            power = manualPower;
        }

        double maxPower = 1;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        shooterRpm = shooterF * Math.sqrt(Math.abs(shooterG * trackingDistance + shooterH)) + shooterI; //Math.sqrt(shooterA * (distance) + shooterC);

        if (MainTeleop.manualTurret) {
            shooterRpm = 3000;
        }

        if (shooterActive) {
            shooter.periodic();
            if (!shooterOverride) {
                shooter.setVelocity(shooterRpm);
            }
        } else {
            shooter.setPower(0);
        }

        motor.set(power);
    }

    public void setShooterVelocity(double rpm) {
        shooter.setVelocity(rpm);
    }

    public void useLowAngleRegression() {
        setShooterRegression(shooterLowF, shooterLowG, shooterLowH, shooterLowI);
    }

    public void useMidAngleRegression() {
        setShooterRegression(shooterMidF, shooterMidG, shooterMidH, shooterMidI);
    }

    public void useHighAngleRegression() {
        setShooterRegression(shooterHighF, shooterHighG, shooterHighH, shooterHighI);
    }

    private void setShooterRegression(double f, double g, double h, double i) {
        shooterF = f;
        shooterG = g;
        shooterH = h;
        shooterI = i;
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
        return motor.getCurrentPosition();
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
}
