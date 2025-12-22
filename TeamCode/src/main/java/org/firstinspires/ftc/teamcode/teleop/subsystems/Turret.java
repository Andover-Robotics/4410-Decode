package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teleop.MainTeleop;

import java.util.ArrayList;

@Config
public class Turret {

    private final MotorEx motor;
    private final IMU imu;
    public Shooter shooter;

    private PIDController controller;
    private final ElapsedTime timer = new ElapsedTime();

    public static boolean positionTracking = true;
    public static boolean shooterActive = true;

    private boolean isManual = false;
    private boolean velComp = true;

    public static double
            p = 0.0115, i = 0, d = 0.0005,
            p2 = 0.0075, i2 = 0, d2 = 0.0002,
            manualPower = 0,
            TURRET_OFFSET_BACK_IN = 3.25;

    private final double
            tolerance = 1,
            powerMin = 0.05,
            degsPerTick = 360.0 / (145.1 * 104.0 / 10.0),
            shooterF = -4096.01855,
            shooterG = -0.00809392,
            shooterH = 1.81342,
            shooterI = 7854.91759;

    public double
            power = 0,
            setPoint = 0,
            pos = 0,
            highLimit = 185,
            lowLimit = -185;

    public static double
            POS_TRACK_X = 0,
            POS_TRACK_Y = 0,
            trackingDistance = 0,
            pureDistance = 0,
            shooterRpm = 0;

    public static YawPitchRollAngles orientation;

    public Pose2d pose;
    public PoseVelocity2d velocity;

    public Turret(OpMode opMode) {

        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(p, i, d);
        controller.setTolerance(tolerance);

        imu = opMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        shooter = new Shooter(opMode);

        timer.reset();
    }

    public void enableFullAuto(boolean on) {
        positionTracking = on;
        shooterActive = on;
    }

    public void enablePositionTracking(boolean on) {
        positionTracking = on;
    }

    public void enableShooter(boolean on) {
        shooterActive = on;
    }

    public void runManual(double manual) {
        if (Math.abs(manual) > powerMin) {
            manualPower = manual;
            isManual = true;
        } else {
            manualPower = 0;
            isManual = false;
        }
    }

    private static double normDeg(double a) {
        return ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    public void runToAngle(double angle) {
        angle = Math.min(Math.max(lowLimit, angle), highLimit);
        setPoint = angle / degsPerTick;
    }

    private double aimAtGlobalPoint(double targetX, double targetY) {

        pose = Bot.drive.localizer.getPose();
        double headingRad = pose.heading.log();

        double turretX = pose.position.x - TURRET_OFFSET_BACK_IN * Math.cos(headingRad);
        double turretY = pose.position.y - TURRET_OFFSET_BACK_IN * Math.sin(headingRad);

        double dx = targetX - turretX;
        double dy = targetY - turretY;

        velocityCompensation(dx, dy);

        double fieldAngleCCW = Math.toDegrees(Math.atan2(POS_TRACK_Y, POS_TRACK_X));
        double robotHeadingCCW = Math.toDegrees(headingRad);
        double relToRobotCCW = normDeg(fieldAngleCCW - robotHeadingCCW);

        pureDistance = Math.hypot(dx, dy);
        trackingDistance = Math.hypot(POS_TRACK_X, POS_TRACK_Y);

        return normDeg(-relToRobotCCW);
    }

    private void velocityCompensation(double dx, double dy) {

        double time = calculateTime(dx, dy);
        velocity = Bot.drive.localizer.update();
        double heading = pose.heading.log();

        double vx = velocity.linearVel.x * Math.cos(heading)
                - velocity.linearVel.y * Math.sin(heading);
        double vy = velocity.linearVel.x * Math.sin(heading)
                + velocity.linearVel.y * Math.cos(heading);

        if (velComp) {
            POS_TRACK_X = dx - vx * time;
            POS_TRACK_Y = dy - vy * time;
        } else {
            POS_TRACK_X = dx;
            POS_TRACK_Y = dy;
        }
    }

    private double calculateTime(double dx, double dy) {

        final double G = 386.09;
        final double dz = 23.0;
        final double angle = Math.toRadians(51.0);

        double R = Math.hypot(dx, dy);
        double term = R * Math.tan(angle) - dz;

        if (term <= 0) return 0;
        return Math.sqrt((2.0 / G) * term);
    }

    public void periodic() {

        pos = getPosition();
        orientation = imu.getRobotYawPitchRollAngles();

        if (positionTracking) {
            controller.setPID(p2, i2, d2);
            runToAngle(aimAtGlobalPoint(
                    Bot.targetPose.x,
                    Bot.targetPose.y
            ));
            controller.setSetPoint(setPoint);
            power = controller.calculate(pos);
        } else {
            power = manualPower;
        }

        power = Math.max(-1, Math.min(1, power));

        shooterRpm = shooterF * Math.sqrt(Math.abs(shooterG * trackingDistance + shooterH)) + shooterI;
        if (MainTeleop.manualTurret) shooterRpm = 3000;

        if (shooterActive) {
            shooter.periodic();
            shooter.setVelocity(shooterRpm);
        } else {
            shooter.setPower(0);
        }

        motor.set(power);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public double getPositionDegs() {
        return getPosition() * degsPerTick;
    }

    public void resetEncoder() {
        motor.resetEncoder();
    }
}