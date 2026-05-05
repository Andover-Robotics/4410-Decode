package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@TeleOp(name = "Drive + FarRamp Snapshots", group = "Test")
public class SnapshotTaker extends LinearOpMode {

    public static int snapshotCounter = 1;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Bot.instance = null;
        Bot bot = Bot.getInstance(this);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        bot.enableFullAuto(true);
        bot.enableShooter(false);
        bot.setTargetFarRampPose();
        Bot.drive.localizer.setPose(Pos.initialFarBluePose);

        while (!isStarted() && !isStopRequested()) {
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.switchAlliance();
                bot.setTargetFarRampPose();
                if (Bot.getAlliance() == Bot.allianceOptions.BLUE_ALLIANCE) {
                    bot.limelight.trackBlueRamp();
                } else {
                    bot.limelight.trackRedRamp();
                }
            }

            telemetry.addLine("Driving-only snapshot opmode");
            telemetry.addData("Alliance (B to switch)", Bot.getAlliance());
            telemetry.addData("Target Pose", "farRampPose");
            telemetry.addData("Next Snapshot (A)", String.format("%03d", snapshotCounter));
            telemetry.update();
        }


        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.switchAlliance();
                bot.setTargetFarRampPose();
                if (Bot.getAlliance() == Bot.allianceOptions.BLUE_ALLIANCE) {
                    bot.limelight.trackBlueRamp();
                } else {
                    bot.limelight.trackRedRamp();
                }
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                @SuppressLint("DefaultLocale") String fileName = String.format("%03d", snapshotCounter);
                bot.limelight.takeSnapshot(fileName);
                snapshotCounter++;
            }

            bot.driveRobotCentric(gp1.getLeftY(), -gp1.getLeftX(), -gp1.getRightX(), 1.0);
            bot.periodic();

            telemetry.addData("Alliance", Bot.getAlliance());
            telemetry.addData("Target Pose", "farRampPose");
            telemetry.addData("Next Snapshot", String.format("%03d", snapshotCounter));
            telemetry.update();
        }
    }
}