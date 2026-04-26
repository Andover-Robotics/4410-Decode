package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Main Teleop", group = "Competition")
public class MainTeleop extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1 ;
    private GamepadEx gp1, gp2;
    private Thread thread;
    private List<Action> runningActions = new ArrayList<>();
    private boolean useStoredPose = true;
    private boolean headingLockEnabled = false, intakeCurrentOverride = false;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean kickersInitialized = false;

    public static boolean stallIntake = true, manualTurret = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.enableFullAuto(true);
        bot.setTargetGoalPose();
        bot.turret.setShooterOverride(false);
        stallIntake = true;


        // Initialize bot
//        bot.stopMotors();

//        waitForStart();

        while (!isStarted()) {
            bot.clearBulkCache();
            bot.indexer.updateSensorCache();

            gp1.readButtons();
            gp2.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                Bot.motif = Bot.Motif.GPP;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                Bot.motif = Bot.Motif.PPG;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                Bot.motif = Bot.Motif.PGP;
            }

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
            telemetry.addData("HEADING LOCK (TOUCHPAD)", headingLockEnabled);
            telemetry.addData("Last Auto Artifacts", Limelight.artifactHistory.toString());
            telemetry.addData("Far Artifacts", Limelight.farArtifacts);
            telemetry.addData("HP Artifacts", Limelight.hpArtifacts);

            telemetry.addData("Motif:", bot.indexer.getMotifPattern());
            telemetry.addLine("DPAD Down: PPG");
            telemetry.addLine("DPAD Left: GPP");
            telemetry.addLine("DPAD Right: PGP");

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

        loopTimer.reset();
        bot.indexer.resetIndexer();
        runningActions.add(bot.indexer.shootRapidFire());

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();

            if (!bot.actionsRunning && !Bot.unjamming) {
                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                    intakeCurrentOverride = false;
                    if (bot.sensorIntaking) {
                        if (bot.indexer.countBalls()==3) {
                            bot.teleopReverseIntake();
                            gp1.gamepad.rumble(1, 1, -1);
                        } else {
                            if (!Indexer.shooting){
                                bot.teleopIntake();
                                gp1.gamepad.stopRumble();
                            }
                        }
                    } else {
                        if (!Indexer.shooting){
                            bot.teleopIntake();
                        }
                    }
                } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                    intakeCurrentOverride = true;
                    bot.teleopReverseIntake();
                    if (bot.indexer.countBalls() == 3) {
                        gp1.gamepad.rumble(1, 1, -1);
                    } else {
                    intakeCurrentOverride = true;
                        bot.teleopReverseIntake();
                        gp1.gamepad.stopRumble();
                    }
                } else if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                    intakeCurrentOverride = false;
                    bot.teleopIntake();
                } else {
                    intakeCurrentOverride = false;
                    bot.teleopStopIntake();
                    gp1.gamepad.stopRumble();
                }
            }

            if (bot.turret.shooterInRange()) {
                gp2.gamepad.rumble(1, 1, -1);
            } else {
                gp2.gamepad.stopRumble();
            }

            // TURRET

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { //everything!
                bot.enableFullAuto(true);
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

            if (gp2.wasJustPressed(GamepadKeys.Button.A) && !gp1.isDown(GamepadKeys.Button.START)) {
                runningActions.add(bot.indexer.shootMotif());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                runningActions.add(bot.indexer.shootRapidFire());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                runningActions.add(bot.indexer.shootNotRapidFire());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.B) && !bot.shooting) {
                runningActions.add(bot.indexer.shootRight());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.X) && !bot.shooting) {
                runningActions.add(bot.indexer.shootLeft());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.Y) && !bot.shooting) {
                runningActions.add(bot.indexer.shootBack());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && !bot.shooting) {
                runningActions.add(bot.indexer.shootPurple());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && !bot.shooting) {
                runningActions.add(bot.indexer.shootGreen());
            }

            // FAILSAFES

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.resetXY();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.resetPose();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                bot.resetGateXY();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.switchAlliance();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.sensorIntaking = !bot.sensorIntaking;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.limelight.periodic();
                bot.limelight.relocalizeBotPose();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.turret.shooter.switchEncoder();
            }


            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                runningActions.add(bot.clearKickerJam());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.indexer.resetDisabledSensors();
            }

            if (manualTurret) {
                bot.turret.runManual(gp2.getLeftX());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.turret.resetEncoder();
            }
            //|| gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)
            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON) ) {
                bot.toggleScreenPeriodic();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.limelight.relocalizeBotPose();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.resetEncoder();
            }


            bot.periodic();
            drive();


            if (Intake.intakeJammed && !intakeCurrentOverride){
                runningActions.add(bot.clearIntakeJam());
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
//
            telemetry.addLine("=== BALL COLORS ===");
//            telemetry.addData("Right", bot.indexer.getRightColor());
//            telemetry.addData("Left", bot.indexer.getLeftColor());
//            telemetry.addData("Back", bot.indexer.getBackColor());
            String right = bot.indexer.getRightColor();
            String left  = bot.indexer.getLeftColor();
            String back  = bot.indexer.getBackColor();

            String rightColor = right.equals("GREEN") ? "green"
                    : right.equals("PURPLE") ? "#f000c3"
                    : right.equals("UNKNOWN") ? "#ff0000"
                    : right.equals("EMPTY") ? "#ffffff"
                    : null;

            String leftColor = left.equals("GREEN") ? "green"
                    : left.equals("PURPLE") ? "#f000c3"
                    : left.equals("UNKNOWN") ? "#ff0000"
                    : left.equals("EMPTY") ? "#ffffff"
                    : null;

            String backColor = back.equals("GREEN") ? "green"
                    : back.equals("PURPLE") ? "#f000c3"
                    : back.equals("UNKNOWN") ? "#ff0000"
                    : back.equals("EMPTY") ? "#ffffff"
                    : null;

            telemetry.addData("<big>Right</big>",
                    rightColor != null
                            ? "<big><font color=\"" + rightColor + "\"><b>" + right + "</b></font></big>"
                            : right);

            telemetry.addData("<big>Left</big>",
                    leftColor != null
                            ? "<big><font color=\"" + leftColor + "\"><b>" + left + "</b></font></big>"
                            : left);

            telemetry.addData("<big>Back</big>",
                    backColor != null
                            ? "<big><font color=\"" + backColor + "\"><b>" + back + "</b></font></big>"
                            : back);
            telemetry.addLine();
            telemetry.addLine("=== SENSOR DISABLE STATUS ===");
            telemetry.addData("GP1 A", "Reset/re-enable all holder sensors");
            telemetry.addData("Right Sensors", "A:%s  B:%s",
                    bot.indexer.rightHolder.isSensorADisabled() ? "DISABLED" : "ON",
                    bot.indexer.rightHolder.isSensorBDisabled() ? "DISABLED" : "ON");
            telemetry.addData("Back Sensors", "A:%s  B:%s",
                    bot.indexer.backHolder.isSensorADisabled() ? "DISABLED" : "ON",
                    bot.indexer.backHolder.isSensorBDisabled() ? "DISABLED" : "ON");
            telemetry.addData("Left Sensors", "A:%s  B:%s",
                    bot.indexer.leftHolder.isSensorADisabled() ? "DISABLED" : "ON",
                    bot.indexer.leftHolder.isSensorBDisabled() ? "DISABLED" : "ON");
//
//            telemetry.addData("<big><b><u>Motif</big></b></u>", "<big><b> "+ Bot.motif + "</big></b></u>");
//
            telemetry.addData("Odom Pose", Math.round(Bot.storedPose.position.x) + " " + Math.round(Bot.storedPose.position.y) + " " + Math.round(Math.toDegrees(Bot.storedPose.heading.log())));
            telemetry.addData("LL Edited Pose", Math.round(Limelight.transformedBotPose.position.x) + " " + Math.round(Limelight.transformedBotPose.position.y) + " " + Math.round(Limelight.llBotPose.getOrientation().getYaw() - 180));
//            telemetry.addData("X", Bot.storedPose.position.x);
//            telemetry.addData("Y", Bot.storedPose.position.y);
//
//
            telemetry.addData("\nalliance", Bot.getAlliance());
            telemetry.addData("\nError (Degs)", "<big><b>" + bot.turret.getErrorDegs() + "</big></b>\n");
            telemetry.addData("\nSensor Intaking", "<big>" + bot.sensorIntaking + "</big>");
//
//            telemetry.addData("Tracking Target", bot.turret.trackingTarget);
//
//

            telemetry.addData("Tracking Target", bot.turret.trackingTarget);
            telemetry.addData("Last Auto Artifacts", Limelight.lastDetectedArtifacts);


            telemetry.addData("\nGoal Distance", Turret.trackingDistance);
////            telemetry.addData("Hood Angle Setpoint:", bot.turret.shooter.getHoodAngle());
////            telemetry.addData("Hood Angle Setpoint:", bot.turret.shooter.getServoPosition());
            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());
            telemetry.addData("Error (Degs)", bot.turret.getErrorDegs());
            telemetry.addData("Power", bot.turret.getPower());
            telemetry.addData("Shooter Encoder", Shooter.leftEncoder ? "<big>Left</big>" : "<big>Right</big>");
            telemetry.addData("Calculated RPM", Turret.shooterRpm);
            telemetry.addData("Current RPM", bot.turret.shooter.getFilteredRPM());
//
//            telemetry.addLine("Shooter PID Data:");
//            telemetry.addData("RPM Error", bot.turret.shooter.getController().getPositionError());
//            telemetry.addData("RPM Target", bot.turret.shooter.getController().getSetPoint());
//            telemetry.addData("Intake Current Threshold?", bot.intake.isAboveCurrentThreshold());
//            telemetry.addData("Intake Jammed?", "<big><b> "+ Intake.intakeJammed + "</big></b></u>");
//            telemetry.addData("Unjamming?", "<big><b> "+ Bot.unjamming + "</big></b></u>");
//            telemetry.addData("<big><b><u>Intake Current</big></b></u>", "<big><b> "+ bot.intake.getCurrent() + "</big></b></u>");
//            telemetry.addData("Current", bot.intake.getCurrent());


//            telemetry.addData("PID Power", bot.turret.shooter.getController().calculate());
//            telemetry.addData("FF Power", bot.turret.shooter.ff);
            telemetry.addData("Shooter Power", bot.turret.shooter.getPower());

            telemetry.addData("Screen periodic (GP2 R stick)", bot.isScreenPeriodicEnabled());
            telemetry.addLine("Loop timing (ms):");
            telemetry.addData("  bulk cache", "%.2f", bot.getBulkCacheMs());
            telemetry.addData("  sensor cache", "%.2f", bot.getSensorCacheMs());
            telemetry.addData("  limelight", "%.2f", bot.getLimelightMs());
            telemetry.addData("  turret", "%.2f", bot.getTurretMs());
            telemetry.addData("  intake", "%.2f", bot.getIntakeMs());
            telemetry.addData("  screen", "%.2f", bot.getScreenMs());
            telemetry.addData("  pose update", "%.2f", bot.getDrivePoseMs());
            telemetry.addData("  bot.periodic total", "%.2f", bot.getPeriodicTotalMs());
            telemetry.addData("Loop ms", "%.1f", loopTimer.milliseconds());
            loopTimer.reset();

            telemetry.update();


        }
    }

    // Driving
    private void drive() { // Robot centric, drive multiplier default 1
        driveSpeed = driveMultiplier - 0.75 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        if (headingLockEnabled) {
            bot.driveHeadingLock(gp1.getLeftY(), -gp1.getLeftX(), driveSpeed);
        } else {
            bot.driveRobotCentric(gp1.getLeftY(), -gp1.getLeftX(), -gp1.getRightX(), driveSpeed);
        }
    }
}
