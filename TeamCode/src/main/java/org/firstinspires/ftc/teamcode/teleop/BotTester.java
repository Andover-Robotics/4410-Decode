package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Limelight;

@TeleOp(name = "Bot Tester", group = "Competition")
public class BotTester extends LinearOpMode {

    private Limelight limelight;
    private GamepadEx gp1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        limelight = new Limelight(this);
        gp1 = new GamepadEx(gamepad1);

        limelight.trackNumBalls();

        while (!isStarted() && !isStopRequested()) {
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                limelight.trackBlueAlliance();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                limelight.trackRedAlliance();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                limelight.trackObelisk();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                limelight.trackNumBalls();
            }

            telemetry.addLine("Limelight-only BotTester (no motors/servos/sensors)");
            telemetry.addLine("A: Blue pipeline");
            telemetry.addLine("B: Red pipeline");
            telemetry.addLine("X: Obelisk pipeline");
            telemetry.addLine("Y: Ball detection pipeline");
            telemetry.addData("Obelisk Mode", limelight.isObelisk());
            telemetry.addData("Ball Detection", limelight.isBallDetection());
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                limelight.setBallDetection(!limelight.isBallDetection());
            }

            limelight.periodic();

            telemetry.addData("Obelisk Mode", limelight.isObelisk());
            telemetry.addData("Ball Detection", limelight.isBallDetection());
            telemetry.addData("Artifacts Detected", limelight.numArtifactsDetected());
            telemetry.update();
        }
    }
}
