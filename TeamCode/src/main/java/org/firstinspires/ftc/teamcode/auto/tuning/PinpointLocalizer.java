package org.firstinspires.ftc.teamcode.auto.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SRSHub;
import org.firstinspires.ftc.teamcode.util.SRSHubSensorLayout;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
    }

    public static Params PARAMS = new Params();

    public static double xOffset = -3.03, yOffset = -5.9;
    public static PoseVelocity2d robotPosVel;

    private final SRSHub srsHubPinpoint;
    private final SRSHub.GoBildaPinpoint pinpoint;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        SRSHubSensorLayout.ensureInitialized(hardwareMap, xOffset, yOffset);
        srsHubPinpoint = SRSHubSensorLayout.getPinpointHub();
        pinpoint = SRSHubSensorLayout.pinpoint;

        srsHubPinpoint.runCommand(new SRSHub.GoBildaPinpoint.ResetIMUCommand(SRSHubSensorLayout.PINPOINT_BUS));

        txWorldPinpoint = initialPose;
        robotPosVel = new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    public void updateOffsets() {
        // Offsets are configured in SRSHubSensorLayout.pinpoint initialization.
    }

    public void recalibrateIMU() {
        srsHubPinpoint.runCommand(new SRSHub.GoBildaPinpoint.ResetIMUCommand(SRSHubSensorLayout.PINPOINT_BUS));
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
        updateOffsets();
        srsHubPinpoint.update();

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
