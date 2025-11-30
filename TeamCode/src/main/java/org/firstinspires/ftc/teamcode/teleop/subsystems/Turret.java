package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teleop.MainTeleop;

import java.util.ArrayList;

@Config
public class Turret {
    private final MotorEx motor;
    private PIDController controller;
    private final ElapsedTime timer = new ElapsedTime();

    private IMU imu;

    private Limelight3A limelight;
    public LLResult llResult;
    public static Pose3D llBotPose = new Pose3D(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));

    public Shooter shooter;

    public static boolean aprilTracking = true, imuFollow = true, shooterActive = true, obelisk = false, positionTracking = true;
//    public static double goalX = 65;
//    public static double goalY = 60;

    public static double POS_TRACK_X = 0;
    public static double POS_TRACK_Y = 0;
    public static double llxRLOffset = 120, llyRLOffset = 108.5;
    public static double TURRET_OFFSET_BACK_IN = 3.25; // inches back from robot center
    public static double p = 0.0115, i = 0, d = 0.0005, p2 = 0.008, i2 = 0, d2 = 0.0004, manualPower = 0, dA = 149, wraparoundTime = 0.35, timerTolerance = 0.15, distanceOffset = 3, llRearOffsetInches = 14;
    private double tolerance = 5, powerMin = 0.05, degsPerTick = 360.0 / (145.1 * 104.0/10.0), ticksPerRev = 360 / degsPerTick, shooterA = 197821.985, shooterC = 1403235.28, shooterF=-4096.01855, shooterG = -0.00809392, shooterH = 1.81342, shooterI = 7854.91759;

    public double txAvg, tyAvg, power, lastTime, setPoint = 0, pos = 0, highLimit = 185, lowLimit = -185, highLimitTicks = highLimit / degsPerTick, lowLimitTicks = lowLimit/degsPerTick;

    public static double tx, ty, distance, tAngle, tOffset, shooterRpm = 0, avgCount = 8, trackingDistance, pureDistance;
    public static YawPitchRollAngles orientation;

    public int startingOffset = 0;

    public ArrayList<Double> txArr, tyArr;

    private boolean isManual = false, wraparound = false;
    private boolean velComp = true;

    public Pose2d pose;
    public PoseVelocity2d velocity;

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
    }


    public void enableShooter(boolean enable) {
        shooterActive = enable;
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
        pose = Bot.drive.localizer.getPose();

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
        double turretTargetCW = normDeg(-relToRobotCCW + 180);

        pureDistance = Math.sqrt(dx*dx + dy*dy);

        // Tracking distance from turret to goal
        trackingDistance = Math.sqrt(POS_TRACK_X * POS_TRACK_X + POS_TRACK_Y * POS_TRACK_Y);

        return turretTargetCW;
    }

    public void velocityCompensation(double dx, double dy) {
        if (getPositionDegs() < 175 && getPositionDegs() > -175) {
            double time = calculateTime(dx, dy);
            velocity = Bot.drive.localizer.update();
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
        final double heightDisplacement = 23.0;  // inches (Δz)
        final double launchAngleAboveHorizDeg = 51.0;  // (90 degrees - actual shooter angle) -> makes the angle relative to horizontal plane
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
        controller.setPID(p, i, d);

        if (!obelisk) {
            // Early-out: position tracking mode
            if (positionTracking) {
                controller.setPID(p2, i2, d2);
                runToAngle(aimAtGlobalPoint(Bot.targetPose.x, Bot.targetPose.y));
//                runToAngle(aimAtGlobalPoint(goalX, goalY));
            }

            controller.setSetPoint(setPoint);

            if (isManual || manualPower != 0) {
                power = manualPower;
            } else {
                power = controller.calculate(pos);
            }

            double maxPower = 1;
            power = Math.max(-maxPower, Math.min(maxPower, power));

            shooterRpm = shooterF * Math.sqrt(Math.abs(shooterG * trackingDistance + shooterH)) + shooterI; //Math.sqrt(shooterA * (distance) + shooterC);

            if (MainTeleop.manualTurret) {
                shooterRpm = 3000;
            }

            if (shooterActive) {
                shooter.periodic();
                shooter.setVelocity(shooterRpm);
            } else {
                shooter.setPower(0);
            }

            // LIMELIGHT RELOCALIZATION
            limelight.updateRobotOrientation(Math.toDegrees(Bot.drive.localizer.getPose().heading.log()));
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    llBotPose = result.getBotpose_MT2();
                }
                //odom x = llx + 120
                //odom y = lly + 105
            }
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

    public void relocalizeBotPose() {
        Bot.drive.localizer.setPose(new Pose2d(llBotPose.getPosition().toUnit(DistanceUnit.INCH).x + llxRLOffset, llBotPose.getPosition().toUnit(DistanceUnit.INCH).y + llyRLOffset, Math.toRadians(llBotPose.getOrientation().getYaw())));
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

}
