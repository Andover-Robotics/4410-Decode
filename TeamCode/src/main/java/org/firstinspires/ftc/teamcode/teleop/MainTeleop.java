package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "MainTeleop")
public class MainTeleop extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    private GamepadEx gp1, gp2;
    private Thread thread;
    private List<Action> runningActions = new ArrayList<>();

    public static boolean stallIntake = true, manualTurret = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.enableFullAuto(true);

        // Initialize bot
//        bot.stopMotors();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.resetHeading();
            }

            if (!bot.shooting) {
                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                    bot.intake.intake();
                } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    bot.intake.reverse();
                } else {
                    if (stallIntake) {
                        bot.intake.storage();
                    } else {
                        bot.intake.stop();
                    }
                }
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                stallIntake = !stallIntake;
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { //imu follow only
                bot.enableFullAuto(false);
                bot.turret.enableImuFollow(true);
                manualTurret = false;
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { //everything!
                bot.enableFullAuto(true);
                manualTurret = false;
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { //auto aim only
                bot.enableFullAuto(false);
                bot.turret.enableAprilTracking(true);
                manualTurret = false;
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { //disable all
                bot.enableFullAuto(false);
                manualTurret = true;
            }

            if (manualTurret) {
                bot.turret.runManual(gp2.getLeftX());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.resetEncoder();
            }

            if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                bot.turret.enableShooter(true);
            } else {
                bot.turret.enableShooter(false);
            }

            if (gp2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.intake.openGate();
            }

            if (gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.intake.closeGate();
            }

            if (gp2.getButton(GamepadKeys.Button.A) && !bot.shooting) {
                runningActions.add(bot.shootOne());
            }

            if (gp2.getButton(GamepadKeys.Button.B) && !bot.shooting) {
                runningActions.add(bot.shootThree());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.lift.enableClosedLoop(!bot.lift.isClosedLoopEnabled());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                bot.lift.liftUp();
            }



            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.lift.balance();
            }

            bot.periodic();
            // DRIVE
            drive();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

//            // TELEMETRY
            telemetry.addData("alliance", Bot.alliance);
            telemetry.addData("starting pos", Bot.startingPos);

            telemetry.addData("\ntx", Turret.tx);
            telemetry.addData("ty", Turret.ty);
            telemetry.addData("correct distance", Turret.distance);
            telemetry.addData( "tag angle", Turret.tAngle);
            telemetry.addData("tOffset", Turret.tOffset);
            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());
            telemetry.addData("\nPower", bot.turret.shooter.getPower());
            telemetry.addData("auto target rpm", Turret.shooterRpm);
            telemetry.addData("filtered rpm", bot.turret.shooter.getFilteredRPM());

            telemetry.addData("\nLeft Climb Position", bot.lift.getLeftEncContinuousDeg());
            telemetry.addData("Right Climb Position", bot.lift.getRightEncContinuousDeg());
            telemetry.addData("Climb Loop?", bot.lift.isClosedLoopEnabled());
            telemetry.addData("Left Power", bot.lift.leftPower);
            telemetry.addData("Right Power", bot.lift.rightPower);
            telemetry.addData("Left Climb Target", bot.lift.leftTargetDeg);
            telemetry.addData("Right Climb Target", bot.lift.rightTargetDeg);
            telemetry.addData("Offset", bot.lift.offset);
            telemetry.addData("Roll", Turret.orientation.getRoll(AngleUnit.DEGREES));
            telemetry.update();

        }
    }

    // Driving
    private void drive() { // Robot centric, drive multiplier default 1
        driveSpeed = driveMultiplier - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();
        Vector2d driveVector = new Vector2d(gp1.getLeftX(), gp1.getLeftY()),
                turnVector = new Vector2d(gp1.getRightX(), 0);
        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed
        );
    }
}