package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDController controller;
    private final ElapsedTime timer = new ElapsedTime();

    private IMU imu;

    private Limelight3A limelight;
    public LLResult llResult;

    public Shooter shooter;

    public static boolean aprilTracking = true, imuFollow = true, shooterActive = true, obelisk = false, positionTracking = true;

    public static double POS_TRACK_X = Bot.goalPose.x;
    public static double POS_TRACK_Y = Bot.goalPose.y;
    public static double TURRET_OFFSET_BACK_IN = 2.5; // inches back from robot center

    public static double p = 0.0115, i = 0, d = 0.0005, p2 = 0.008, i2 = 0, d2 = 0.00033, manualPower = 0, dA = 149, wraparoundTime = 0.35, timerTolerance = 0.15, distanceOffset = 3, llRearOffsetInches = 14;
    private double tolerance = 5, powerMin = 0.05, degsPerTick = 360.0 / (145.1 * 104.0/10.0), ticksPerRev = 360 / degsPerTick, shooterA = 197821.985, shooterC = 1403235.28, shooterF = -5135.52, shooterG = -0.00678212, shooterH = 1.6709, shooterI = 9199.73374;

    public double txAvg, tyAvg, power, lastTime, setPoint = 0, pos = 0, highLimit = 185, lowLimit = -185, highLimitTicks = highLimit / degsPerTick, lowLimitTicks = lowLimit/degsPerTick;

    public static double tx, ty, distance, tAngle, tOffset, shooterRpm = 0, avgCount = 8, trackingDistance;
    public static YawPitchRollAngles orientation;

    public int startingOffset = 0;

    public ArrayList<Double> txArr, tyArr;

    private boolean isManual = false, wraparound = false;

    public enum Motif {
        GPP,
        PGP,
        PPG
    }

    public static Motif motif;

    public Turret(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDController(p, i, d);
        controller.setTolerance(tolerance);
        controller.setSetPoint(0);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        // initialize limelight
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        shooter = new Shooter(opMode);

        timer.reset();
        lastTime = timer.seconds();
        startingOffset = 45 * ((Bot.isBlue())? -1 : 1);
        txArr = new ArrayList<>(0);
        tyArr = new ArrayList<>(0);
        POS_TRACK_X = Bot.goalPose.x;
        POS_TRACK_Y = Bot.goalPose.y;
    }

    public void setPipeline(int i) {
        limelight.pipelineSwitch(i);
        /*
            0 is blue alliance
            1 is red alliance
            2 is obelisk tracking
         */
    }

    public void trackRedAlliance() {
        setPipeline(1);
//        POS_TRACK_X = Math.abs(POS_TRACK_X) * -1;
        obelisk = false;
    }

    public void trackBlueAlliance() {
        setPipeline(0);
//        POS_TRACK_X = Math.abs(POS_TRACK_X);
        obelisk = false;
    }

    public void trackObelisk() {
        setPipeline(2);
        obelisk = true;
    }

    public void enableFullAuto(boolean on) {
        enableAutoAim(on);
        enableShooter(on);
    }

    public void enableAutoAim(boolean on) {
        enablePositionTracking(on);
        enableAprilTracking(on);
        enableImuFollow(on);
    }

    public void enableAprilTracking(boolean enable) {
        aprilTracking = enable;
        if (enable) {
            if (Bot.isBlue()) {
                trackBlueAlliance();
            } else {
                trackRedAlliance();
            }
        }
    }

    public void enableShooter(boolean enable) {
        shooterActive = enable;
    }

    public void enableImuFollow(boolean enable) {
        imuFollow = enable;
    }

    public void enablePositionTracking(boolean enable) {
        positionTracking = enable;
    }

    public void runToAngle(double angle) {
        if (angle > highLimit) {
            angle = angle - 360;
            wraparound = true;
        } else if (angle < lowLimit) {
            angle = angle + 360;
            wraparound = true;
        } else {
            wraparound = false;
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
            isManual = true;
            manualPower = manual;
        } else {
            manualPower = 0;
            isManual = false;
        }
    }

    public void filtertx() {
        txArr.add(tx);
        if (txArr.size() > avgCount) {
            txArr.remove(0);
        }
        double tempTxAvg = 0;
        for (double i : txArr) {
            tempTxAvg += i;
        }
        txAvg = tempTxAvg / txArr.size();
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
    private void aimAtGlobalPoint(double targetX, double targetY) {
        Pose2d pose = Bot.drive.localizer.getPose();

        // Robot heading in radians (CCW+)
        double headingRad = pose.heading.log();

        // Turret position in field frame:
        // "back" = negative X in robot frame, rotated into field frame
        double turretX = pose.position.x - TURRET_OFFSET_BACK_IN * Math.cos(headingRad);
        double turretY = pose.position.y - TURRET_OFFSET_BACK_IN * Math.sin(headingRad);

        // Vector from turret to target in field frame
        double dx = targetX - turretX;
        double dy = targetY - turretY;

        // Field-bearing CCW to target
        double fieldAngleCCW = Math.toDegrees(Math.atan2(dy, dx)); // CCW+

        // Robot heading CCW
        double robotHeadingCCW = Math.toDegrees(headingRad); // CCW+

        // Robot-relative CCW angle to target
        double relToRobotCCW = normDeg(fieldAngleCCW - robotHeadingCCW);

        // Turret is CW-positive and zero is backwards, so:
        // 0° turret = 180° robot-relative CCW
        double turretTargetCW = normDeg(-relToRobotCCW + 180);

        // Tracking distance from turret to goal
        trackingDistance = Math.sqrt(dx * dx + dy * dy);

        runToAngle(turretTargetCW);
    }

    public void periodic() {
        if (Bot.isBlue()) {
            if (Bot.isFar()) {
                startingOffset = -135;
            } else {
                startingOffset = 0;
            }
        } else {
            if (Bot.isFar()) {
                startingOffset = 135;
            } else {
                startingOffset = 0;
            }
        }
        power = 0;
        orientation = imu.getRobotYawPitchRollAngles();
        llResult = limelight.getLatestResult();
        pos = getPosition();
        controller.setPID(p, i, d);

        if (!obelisk){
            // Early-out: position tracking mode
            if (positionTracking) {
                controller.setPID(p2, i2, d2);
                aimAtGlobalPoint(POS_TRACK_X, POS_TRACK_Y);
                controller.setSetPoint(setPoint);

                if (isManual || manualPower != 0) {
                    power = manualPower;
                } else {
                    pos = getPosition();
                    power = controller.calculate(pos);
                }
                power = Math.max(-1, Math.min(1, power));

                updateShooter();

                motor.set(power);
                return;
            }

            if ((llResult != null && llResult.isValid() && aprilTracking) && (!wraparound || (wraparound == (timer.seconds() - lastTime > wraparoundTime)))) { //checks if LL valid, and its not in the middle of wrapping around
                wraparound = false;
                tx = llResult.getTx();
                ty = llResult.getTy();
                filtertx();
                runToAngle(getPositionDegs() + ty);// not using avg here (kept as in original)
                controller.setPID(p, i, d);
                lastTime = timer.seconds();
            } else if ((timer.seconds() - lastTime) > (wraparoundTime + timerTolerance) || !aprilTracking) {
                if (imuFollow) {
                    runToAngle(getHeading() + startingOffset);
                    controller.setPID(p2, i2, d2);
                }
            }
            controller.setSetPoint(setPoint);

            if (isManual || manualPower != 0) {
                power = manualPower;
            } else {
                power = controller.calculate(pos);
            }

            double maxPower = 1;
            power = Math.max(-maxPower, Math.min(maxPower, power));

            updateShooter();
        } else {
            if (llResult != null && llResult.isValid() && llResult.getFiducialResults() != null && !llResult.getFiducialResults().isEmpty()) {
                int id = llResult.getFiducialResults().get(0).getFiducialId();
                if (id == 21) {
                    motif = Motif.GPP;
                } else if (id == 22) {
                    motif = Motif.PGP;
                } else if (id == 23) {
                    motif = Motif.PPG;
                }
            }
        }

        motor.set(power);
    }

    private void updateShooter() {
        tAngle = getPositionDegs() - getHeading() - startingOffset;
        tOffset = llRearOffsetInches * Math.cos(Math.toRadians(tAngle));
        distance = (29.5 - 17) / Math.tan(Math.toRadians(25 - txAvg)) - distanceOffset + tOffset;
        shooterRpm = shooterF * Math.sqrt(Math.abs(shooterG * trackingDistance + shooterH)) + shooterI; //Math.sqrt(shooterA * (distance) + shooterC);

        if (shooterActive) {
            shooter.setVelocity(shooterRpm);
            shooter.periodic();
        } else {
            shooter.setPower(0);
        }
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public double getPositionDegs() {
        return getPosition() * degsPerTick;
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

    public double getHeading() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getRoll() {
        orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getRoll(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

}
