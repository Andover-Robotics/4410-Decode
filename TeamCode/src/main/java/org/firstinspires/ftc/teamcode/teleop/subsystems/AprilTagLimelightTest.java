package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
@TeleOp(name = "LimeLightTest")
public class AprilTagLimelightTest extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        //limelight.pipelineSwitch(8); // april tag #__ pipeline
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();

        telemetry.addLine("Loop running");

        if (llResult == null) {
            telemetry.addLine("llResult is null");
        } else {
            Pose3D botpose = llResult.getBotpose();
            telemetry.addData("Valid?", llResult.isValid());
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botpose.toString());
            telemetry.addData("Yaw", botpose.getOrientation().getYaw());
        }

        telemetry.update();
    }

}
