package org.firstinspires.ftc.teamcode.auto.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystems.SRSHubs;
import org.firstinspires.ftc.teamcode.util.SRSHub;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        }

    public static Params PARAMS = new Params();

    private SRSHubs srshubs;
    public SRSHub.GoBildaPinpoint pinpoint;
    public static double xOffset = -3.03, yOffset = -5.9;
    public static PoseVelocity2d robotPosVel;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(SRSHubs srshubs, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        pinpoint = srshubs.getPinpoint();

        srshubs.getPinpointHub().runCommand(new SRSHub.GoBildaPinpoint.ResetIMUCommand(1));

        txWorldPinpoint = initialPose;
        robotPosVel = new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    public void recalibrateIMU() {
        srshubs.getPinpointHub().runCommand(new SRSHub.GoBildaPinpoint.ResetIMUCommand(1));
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d getPoseVelocity() {
        return robotPosVel;
    }

    @Override
    public PoseVelocity2d update() {
//        pinpoint.update();
//        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
//            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
//            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
//            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);
//            robotPosVel = new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//
//            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
//        }
        if (!pinpoint.disconnected) {
            txPinpointRobot = new Pose2d(pinpoint.xPosition / 25.4, pinpoint.yPosition / 25.4, pinpoint.hOrientation);
            Vector2d worldVelocity = new Vector2d(pinpoint.xVelocity / 25.4, pinpoint.yVelocity / 25.4);
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);
            robotPosVel = new PoseVelocity2d(robotVelocity, pinpoint.hVelocity);

            return robotPosVel;
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
