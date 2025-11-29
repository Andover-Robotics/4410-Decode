package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

import java.lang.*;
import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Bot Tester", group = "Utility")
public class BotTester extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1;
    private GamepadEx gp1, gp2;
    private boolean fieldCentric, intakeCancel, clipCancel;
    private Thread thread;
    private List<Action> runningActions = new ArrayList<>();

    private int degTarget = 0;
    public static boolean runTurret = false;
    public static double rpm = 4800;
    public static boolean shoot = false, stallIntake = true, fullIntake = false;
    public static boolean manualRPM = false;
    public static boolean autoPos = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        // Initialize bot
//        bot.stopMotors();

        while (!isStarted()) {

            gp1.readButtons();
            gp2.readButtons();

            TelemetryPacket packet = new TelemetryPacket();

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.resetEncoder();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.switchAlliance();
                autoPos = false;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.switchStartingPos();
                autoPos = false;
            }

            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
            telemetry.addData("STARTING POSITION (B)", Bot.getStartingPos());
            telemetry.addData("AUTO POSITION", autoPos);

            telemetry.update();
        }

        if (!autoPos) {
            if (Bot.isFar()) {
                if (Bot.isBlue()) {
                    Bot.drive.localizer.setPose(Pos.initialFarBluePose);
                } else {
                    Bot.drive.localizer.setPose(Pos.initialFarRedPose);
                }
            } else {
                if (Bot.isBlue()) {
                    Bot.drive.localizer.setPose(Pos.initialCloseBluePose);
                } else {
                    Bot.drive.localizer.setPose(Pos.initialCloseRedPose);
                }
            }
        }



        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();


            if (gp1.wasJustPressed(GamepadKeys.Button.A)) { //everything!
                runTurret = true;
                bot.enableFullAuto(true);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) { //disable all
                runTurret = false;
                bot.enableFullAuto(false);
            }

            if (!bot.shooting) {
                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                    bot.intake.intake();
                } else {
                    if (stallIntake) {
                        bot.intake.storage();
                    } else {
                        if (fullIntake) {
                            bot.intake.intake();
                        } else {
                            bot.intake.stop();
                        }
                    }
                }
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                stallIntake = !stallIntake;
            }

            if (runTurret){
                bot.turret.periodic();
            }

            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 || shoot) {
                bot.turret.enableShooter(true);
                bot.turret.shooter.setVelocity(rpm);
            } else {
                bot.turret.enableShooter(false);
            }

            if (gp1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.intake.openGate();
            }

            if (gp1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.intake.closeGate();
            }

            if (gp2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                runningActions.add(bot.shootOne());
            }

            if (gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                runningActions.add(bot.shootThree());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.lift.enableClosedLoop(!bot.lift.isClosedLoopEnabled());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                bot.lift.liftUp();
            }

            bot.lift.periodic();
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
            telemetry.addData("alliance", Bot.getAlliance());
            telemetry.addData("starting pos", Bot.getStartingPos());

            telemetry.addData("\nPose", Bot.drive.localizer.getPose());
            telemetry.addData("Velocity", Bot.drive.localizer.update());
            telemetry.addData("Goal Distance", Turret.trackingDistance);

            telemetry.addData("\ntx", Turret.tx);
            telemetry.addData("ty", Turret.ty);

            telemetry.addData("\ntx", Turret.tx);
            telemetry.addData("ty", Turret.ty);
            telemetry.addData( "tag angle", Turret.tAngle);
            telemetry.addData("tOffset", Turret.tOffset);
            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());
            telemetry.addData("\nPower", bot.turret.shooter.getPower());
            telemetry.addData("correct distance", Turret.distance);
            telemetry.addData("regression distance", Turret.distance - Turret.tOffset);
            telemetry.addData("manual target rpm", rpm);
            telemetry.addData("auto target rpm", Turret.shooterRpm);
            telemetry.addData("filtered rpm", bot.turret.shooter.getFilteredRPM());

            telemetry.addData("\nLeft Climb Encoder", bot.lift.getLeftEncAbsDeg());
            telemetry.addData("Right Climb Encoder", bot.lift.getRightEncAbsDeg());
            telemetry.addData("Left Climb Pos", bot.lift.getLeftEncContinuousDeg());
            telemetry.addData("Right Climb Pos", bot.lift.getRightEncContinuousDeg());
            telemetry.addData("Climb Loop?", bot.lift.isClosedLoopEnabled());
            telemetry.addData("Left Power", bot.lift.leftPower);
            telemetry.addData("Right Power", bot.lift.rightPower);

//            if (bot.turret.llResult != null)
//                telemetry.addData("Limelight Result", bot.turret.llResult);

//            telemetry.addData("On?", shoot);

            telemetry.update();
            Bot.drive.localizer.update();

        }
    }

    // Driving
    private void drive() { // Robot centric, drive multiplier default 1
        driveSpeed = driveMultiplier - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
//        bot.fixMotors();
        com.arcrobotics.ftclib.geometry.Vector2d driveVector = new com.arcrobotics.ftclib.geometry.Vector2d(-gp1.getLeftX(), -gp1.getLeftY());
//                turnVector = new com.arcrobotics.ftclib.geometry.Vector2d(-gp1.getRightX(), 0);
//        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
//                driveVector.getY() * driveSpeed,
//                turnVector.getX() * driveSpeed
//        );

        Bot.drive.setDrivePowers(new PoseVelocity2d(new com.acmerobotics.roadrunner.Vector2d(driveSpeed * gp1.getLeftY(),driveSpeed * -gp1.getLeftX()),driveSpeed * -gp1.getRightX()));
    }
}