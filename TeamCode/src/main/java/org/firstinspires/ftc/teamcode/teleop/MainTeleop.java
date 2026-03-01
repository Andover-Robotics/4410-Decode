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
import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Indexer;
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
    private boolean headingLockEnabled = false;
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

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();

//            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
//                sensing = !sensing;
//            } //

            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                if (!kickersInitialized) {
                    runningActions.add(bot.indexer.leftHolder.resetFastAction());
                    kickersInitialized = true;
                }
                if (bot.sensorIntaking) {
                    if (bot.indexer.countBalls()==3) {
                        bot.teleopReverseIntake();
                        gp1.gamepad.rumble(1, 1, -1);
                    } else {
                        bot.teleopIntake();
                        gp1.gamepad.stopRumble();
                    }
                } else {
                    bot.teleopIntake();
                }
            } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.teleopReverseIntake();
                if (bot.indexer.countBalls() == 3) {
                    gp1.gamepad.rumble(1, 1, -1);
                } else {
                    bot.teleopReverseIntake();
                    gp1.gamepad.stopRumble();
                }
            } else if (gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.teleopIntake();
            } else {
                bot.teleopStopIntake();
                gp1.gamepad.stopRumble();
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
                runningActions.add(bot.indexer.shootRapidFireSensor());
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

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON) && !bot.shooting) {
                bot.switchShooting();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.toggleScreenPeriodic();
            }


            // CLIMB
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.lift.enableClosedLoop(!bot.lift.isClosedLoopEnabled());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                bot.lift.liftUp();
            }

            // FAILSAFES

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.switchAlliance();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
//                bot.limelight.relocalizeBotPose();
//                headingLockEnabled = !headingLockEnabled;
                bot.sensorIntaking = !bot.sensorIntaking;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.resetPose();
            }

            if (manualTurret) {
                bot.turret.runManual(gp2.getLeftX());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.turret.resetEncoder();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.indexer.jiggleKickers();
            }

            bot.periodic();
            drive();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

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

            telemetry.addData("<big><b><u>Motif</big></b></u>", "<big><b> "+ Bot.motif + "</big></b></u>");

            telemetry.addData("Odom Pose", Math.round(Bot.storedPose.position.x) + " " + Math.round(Bot.storedPose.position.y) + " " + Math.round(Math.toDegrees(Bot.storedPose.heading.log())));
//            telemetry.addData("LL Pose", Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).x + Turret.llxRLOffset) + " " + Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).y + Turret.llyRLOffset) + " " + Math.round(Turret.llBotPose.getOrientation().getYaw()));
            telemetry.addData("\nalliance", Bot.getAlliance());
            telemetry.addData("starting pos", Bot.getStartingPos());


            telemetry.addData("\nGoal Distance", Turret.trackingDistance);
//            telemetry.addData("Hood Angle Setpoint:", bot.turret.shooter.getHoodAngle());
//            telemetry.addData("Hood Angle Setpoint:", bot.turret.shooter.getServoPosition());
//            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());
            telemetry.addData("Error (Degs)", bot.turret.getErrorDegs());
            telemetry.addData("Power", bot.turret.getPower());
            telemetry.addData("Calculated RPM", Turret.shooterRpm);
            telemetry.addData("Target RPM", bot.turret.shooter.getControllerTargetRPM());
            telemetry.addData("Current RPM", bot.turret.shooter.getFilteredRPM());
//            telemetry.addData("Shooter Power", bot.turret.shooter.getPower());

            telemetry.addData("Screen periodic (GP2 R stick)", bot.isScreenPeriodicEnabled());
            telemetry.addData("Sensor Intaking (GP1 Back Button)", bot.sensorIntaking);
            telemetry.addData("Loop ms", "%.1f", loopTimer.milliseconds());
            loopTimer.reset();


//            // Back A for hue
//            float backHue = bot.indexer.leftHolder.hueFromSensor(bot.indexer.colorBR()); // Back A sensor
//            telemetry.addLine("=== BACK SENSOR HUE ===");
//// Back B for distance
//            double backDist = bot.indexer.safeDistance(bot.indexer.colorBL()); // Back B sensor
//            telemetry.addLine("=== BACK SENSOR DISTANCE ===");
//            telemetry.addData("Back B Distance (mm)", "%.1f", backDist);

////
//
//            telemetry.addLine("=== Distance ===");
//            telemetry.addData("BR Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorBR()));
//            telemetry.addData("BL Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorBL()));
//            telemetry.addData("RL Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorRL()));
//            telemetry.addData("RR Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorRR()));
//            telemetry.addData("LR Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorLR()));
//            telemetry.addData("LL Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorLL()));
//
//            telemetry.addLine("=== HUE ===");
//            telemetry.addData("BR Hue", "%.1f", bot.indexer.leftHolder.hueFromSensor(bot.indexer.colorBR()));
//            telemetry.addData("BL Hue", "%.1f", bot.indexer.leftHolder.hueFromSensor(bot.indexer.colorBL()));
//            telemetry.addData("RL Hue", "%.1f", bot.indexer.leftHolder.hueFromSensor(bot.indexer.colorRL()));
//            telemetry.addData("RL HSV", bot.indexer.leftHolder.hsvFromSensor(bot.indexer.colorRL()));
//            telemetry.addData("RR Hue", "%.1f", bot.indexer.leftHolder.hueFromSensor(bot.indexer.colorRR()));
//            telemetry.addData("RR HSV", bot.indexer.leftHolder.hsvFromSensor(bot.indexer.colorRR()));
//            telemetry.addData("LR Hue", "%.1f", bot.indexer.leftHolder.hueFromSensor(bot.indexer.colorLR()));
//            telemetry.addData("LL Hue", "%.1f", bot.indexer.leftHolder.hueFromSensor(bot.indexer.colorLL()));








//            telemetry.addData("auto target rpm", Turret.shooterRpm);
//            telemetry.addData("filtered rpm", bot.turret.shooter.getFilteredRPM());
//
//            telemetry.addData("\nLeft Climb Position", bot.lift.getLeftEncContinuousDeg());
//            telemetry.addData("Right Climb Position", bot.lift.getRightEncContinuousDeg());
//            telemetry.addData("\nLeft Climb Abs Position", bot.lift.getLeftEncAbsDeg());
//            telemetry.addData("Right Climb Abs Position", bot.lift.getRightEncAbsDeg());
//
////            telemetry.addData("Climb Loop?", bot.lift.isClosedLoopEnabled());
//            telemetry.addData("Left Power", bot.lift.leftPower);
//            telemetry.addData("Right Power", bot.lift.rightPower);
//            telemetry.addData("\nActual Left Power", bot.lift.climbLeft.get());
//            telemetry.addData("Actual Right Power", bot.lift.climbRight.get());

////            telemetry.addData("Left PID out", bot.lift.leftPidOut);
//////            telemetry.addData("Right PID out", bot.lift.rightPidOut);
//            telemetry.addData("Left Climb Target", bot.lift.leftTargetDeg);
//            telemetry.addData("Right Climb Target", bot.lift.rightTargetDeg);
////            telemetry.addData("Offset", bot.lift.offset);
////            telemetry.addData("Roll", Turret.orientation.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Velocity", Bot.drive.localizer.update());

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
