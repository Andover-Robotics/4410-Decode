package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "MainTeleop", group = "Competition")
public class MainTeleop extends LinearOpMode {

    private Bot bot;
    private GamepadEx gp1, gp2;
    private List<Action> runningActions = new ArrayList<>();

    private boolean useStoredPose = true;

    public static boolean stallIntake = true;
    public static boolean manualTurret = false;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        bot.enableFullAuto(true);
        bot.setTargetGoalPose();

        while (!isStarted()) {

            gp1.readButtons();
            gp2.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.switchAlliance();
                useStoredPose = false;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.switchStartingPos();
                useStoredPose = false;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                useStoredPose = !useStoredPose;
            }

            telemetry.addData("Alliance", Bot.getAlliance());
            telemetry.addData("Starting Pos", Bot.getStartingPos());
            telemetry.addData("Stored Pose", useStoredPose);
            telemetry.update();
        }

        if (!useStoredPose) {
            if (Bot.isFar()) {
                Bot.drive.localizer.setPose(
                        Bot.isBlue() ? Pos.initialFarBluePose : Pos.initialFarRedPose
                );
            } else {
                Bot.drive.localizer.setPose(
                        Bot.isBlue() ? Pos.initialCloseBluePose : Pos.initialCloseRedPose
                );
            }
        } else {
            Bot.useStoredPose();
        }

        while (opModeIsActive()) {

            gp1.readButtons();
            gp2.readButtons();

            /* ===== TURRET MODES ===== */

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.enableFullAuto(true);
                manualTurret = false;
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.enableFullAuto(false);
                bot.turret.enablePositionTracking(true);
                manualTurret = false;
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.enableFullAuto(false);
                manualTurret = true;
            }

            if (manualTurret) {
                bot.turret.runManual(gp2.getLeftX());
            }

            /* ===== SHOOTER ===== */

            bot.turret.enableShooter(
                    gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2
            );

            /* ===== UPDATE ===== */

            bot.periodic();
            drive();

            telemetry.addData("Turret Deg", bot.turret.getPositionDegs());
            telemetry.addData("Target RPM", Turret.shooterRpm);
            telemetry.addData("Distance", Turret.trackingDistance);
            telemetry.update();
        }
    }

    private void drive() {

        double speed = 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        speed = Math.max(0, speed);

        Bot.drive.setDrivePowers(
                new PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(
                                speed * gp1.getLeftY(),
                                speed * -gp1.getLeftX()
                        ),
                        speed * -gp1.getRightX()
                )
        );
    }
}