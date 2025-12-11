package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "MainTeleop", group = "Competition")
public class MainTeleop extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1 ;
    private GamepadEx gp1, gp2;
    private Thread thread;
    private List<Action> runningActions = new ArrayList<>();
    private boolean useStoredPose = true;

    NormalizedRGBA colors;

    public static boolean stallIntake = true, manualTurret = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.enableFullAuto(true);
        bot.setTargetFarAutoGoal();

        // Initialize bot
//        bot.stopMotors();

//        waitForStart();

        while (!isStarted()) {

            gp1.readButtons();
            gp2.readButtons();

            TelemetryPacket packet = new TelemetryPacket();

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.resetEncoder();
            }

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

            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
            telemetry.addData("STARTING POSITION (B)", Bot.getStartingPos());
            telemetry.addData("STORED POSITION", useStoredPose);

            telemetry.update();
        }

        if (!useStoredPose) {
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
        } else {
            Bot.useStoredPose();
        }

        bot.intake.closeGate();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();

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

            // CLIMB

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.lift.enableClosedLoop(!bot.lift.isClosedLoopEnabled());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                bot.lift.liftUp();
            }

            // TURRET

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { //everything!
                bot.enableFullAuto(true);
                manualTurret = false;
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { //position tracking
                bot.enableFullAuto(false);
                bot.turret.enablePositionTracking(true);
                manualTurret = false;
            }
            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { //no tracking
                bot.enableFullAuto(false);
                manualTurret = true;
            }

            // SHOOTING

            if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                bot.turret.enableShooter(true);
            } else {
                bot.turret.enableShooter(false);
            }

            if (gp2.getButton(GamepadKeys.Button.A) && !bot.shooting) {
                runningActions.add(bot.shootOne());
            }

            if (gp2.getButton(GamepadKeys.Button.B) && !bot.shooting) {
                runningActions.add(bot.shootThree());
            }

            // FAILSAFES

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.switchAlliance();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.relocalizeBotPose();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.resetPose();
            }

            if (gp2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.intake.openGate();
            }

            if (gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.intake.closeGate();
            }

            if (manualTurret) {
                bot.turret.runManual(gp2.getLeftX());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.resetEncoder();
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

            // TELEMETRY

//
//            colors = bot.intake.color.getNormalizedColors();
//
//            telemetry.addData("rgb: ", colors.red + " " + colors.blue + " " + colors.green);
//
//            double r = colors.red, g = colors.green, b = colors.blue;
//            double cmax = Math.max(r, Math.max(g, b)), cmin = Math.min(r, Math.min(g, b));
//            double d = cmax - cmin;
//
//            double h = d == 0 ? 0 :
//                    cmax == r ? 60 * (((g - b) / d) % 6) :
//                            cmax == g ? 60 * (((b - r) / d) + 2) :
//                                    60 * (((r - g) / d) + 4);
//
//            double s = cmax == 0 ? 0 : (d / cmax);
//            double v = cmax;
//
//            telemetry.addData("hsv: ", h + " " + s + " " + v);


            telemetry.addData("Odom Pose", Math.round(Bot.drive.localizer.getPose().position.x) + " " + Math.round(Bot.drive.localizer.getPose().position.y) + " " + Math.round(Math.toDegrees(Bot.drive.localizer.getPose().heading.log())));
            telemetry.addData("LL Pose", Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).x + Turret.llxRLOffset) + " " + Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).y + Turret.llyRLOffset) + " " + Math.round(Turret.llBotPose.getOrientation().getYaw()));
            telemetry.addData("\nalliance", Bot.getAlliance());
            telemetry.addData("starting pos", Bot.getStartingPos());
            telemetry.addData("\n", bot.intake.storageCount());
            telemetry.addData("\nHolding Bottom", bot.intake.holdingBottom());
            telemetry.addData("Status Bottom", bot.intake.bottomStatus());
            telemetry.addData("Color Bottom", bot.intake.rawBottomColor());
            telemetry.addData("Break Beam Bottom", bot.intake.rawBottomBreakBeam());
//            telemetry.addData("Bottom Purple State", bot.intake.blb0.getState());
//            telemetry.addData("Bottom Green State", bot.intake.blb1.getState());
            telemetry.addData("\nHolding Middle", bot.intake.holdingMiddle());
            telemetry.addData("Status Middle", bot.intake.middleStatus());
            telemetry.addData("Color Middle", bot.intake.rawMiddleColor());
            telemetry.addData("Break Beam Middle", bot.intake.rawMiddleBreakBeam());
//            telemetry.addData("Middle Purple State", bot.intake.blm0.getState());
//            telemetry.addData("CMiddle Green State", bot.intake.blm1.getState());
            telemetry.addData("\nHolding Top", bot.intake.holdingTop());
            telemetry.addData("Status Top", bot.intake.topStatus());
            telemetry.addData("Color Top", bot.intake.rawTopColor());
            telemetry.addData("Break Beam Top", bot.intake.rawTopBreakBeam());
//            telemetry.addData("Top Purple State", bot.intake.blt0.getState());
//            telemetry.addData("Top Green State", bot.intake.blt1.getState());
//
//            telemetry.addData("\nPose", Bot.drive.localizer.getPose());
//            telemetry.addData("Velocity", Bot.drive.localizer.update());
            telemetry.addData("\nGoal Distance", Turret.trackingDistance);
            telemetry.addData("Shoot Delay", Bot.shootDelay);

//
//            telemetry.addData("\ntx", Turret.tx);
//            telemetry.addData("ty", Turret.ty);
//
//            telemetry.addData("txAvg", bot.turret.txAvg);
//
//            telemetry.addData("correct distance", Turret.distance);
//            telemetry.addData( "tag angle", Turret.tAngle);
//            telemetry.addData("tOffset", Turret.tOffset);
            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());

            telemetry.addData("auto target rpm", Turret.shooterRpm);
            telemetry.addData("filtered rpm", bot.turret.shooter.getFilteredRPM());

//            telemetry.addData("\nLeft Climb Position", bot.lift.getLeftEncContinuousDeg());
//            telemetry.addData("Right Climb Position", bot.lift.getRightEncContinuousDeg());
//            telemetry.addData("Climb Loop?", bot.lift.isClosedLoopEnabled());
//            telemetry.addData("Left Power", bot.lift.leftPower);
//            telemetry.addData("Right Power", bot.lift.rightPower);
////            telemetry.addData("Left PID out", bot.lift.leftPidOut);
////            telemetry.addData("Right PID out", bot.lift.rightPidOut);
//            telemetry.addData("Left Climb Target", bot.lift.leftTargetDeg);
//            telemetry.addData("Right Climb Target", bot.lift.rightTargetDeg);
//            telemetry.addData("Offset", bot.lift.offset);
//            telemetry.addData("Roll", Turret.orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Velocity", Bot.drive.localizer.update());
            telemetry.update();


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