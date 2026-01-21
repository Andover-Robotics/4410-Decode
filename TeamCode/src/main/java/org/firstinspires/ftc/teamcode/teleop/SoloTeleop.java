package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.OldPoses;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Indexer;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Solo Teleop", group = "Competition")
public class SoloTeleop extends LinearOpMode {


    private Bot bot;
    private double driveSpeed = 1, driveMultiplier = 1 ;
    private GamepadEx gp1, gp2;
    private Thread thread;
    private List<Action> runningActions = new ArrayList<>();
    private boolean useStoredPose = true;
    private final ElapsedTime loopTimer = new ElapsedTime();

    NormalizedRGBA colors;

    public static int rpm=2000;
    public static boolean stallIntake = true, manualTurret = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.enableFullAuto(true);
        bot.setTargetGoalPose();
        stallIntake = true;


        // Initialize bot
//        bot.stopMotors();

//        waitForStart();

        while (!isStarted()) {
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

            telemetry.addData("Motif:", Indexer.getMotifPattern());
            telemetry.addLine("DPAD Down: PPG");
            telemetry.addLine("DPAD Left: GPP");
            telemetry.addLine("DPAD Right: PGP");

            telemetry.update();
        }

        if (!useStoredPose) {
            if (Bot.isFar()) {
                if (Bot.isBlue()) {
                    Bot.drive.localizer.setPose(OldPoses.initialFarBluePose);
                } else {
                    Bot.drive.localizer.setPose(OldPoses.initialFarRedPose);
                }
            } else {
                if (Bot.isBlue()) {
                    Bot.drive.localizer.setPose(OldPoses.initialCloseBluePose);
                } else {
                    Bot.drive.localizer.setPose(OldPoses.initialCloseRedPose);
                }
            }
        } else {
            Bot.useStoredPose();
        }

        loopTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();
//            bot.shooting = false;

//            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
//                sensing = !sensing;
//            } //

            if (!bot.shooting) {
                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                    if (bot.indexer.countBalls()==3) {
                        bot.intake.reverse();
                    } else {
                        bot.intake.intake();
                    }
                } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                    bot.intake.reverse();
//                } else if (bot.indexer.countBalls()==3){
//                    bot.intake.reverse();
//                } else if (stallIntake){
//                    bot.intake.storage();
                } else {
                    bot.intake.stop();
                }
            }


            // TURRET

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) { //everything!
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

            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
//                bot.turret.shooter.setManualPower(rpm);
                bot.turret.enableShooter(true);
            } else {
                bot.turret.enableShooter(false);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                runningActions.add(bot.indexer.shootMotif());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                runningActions.add(bot.indexer.shootRapidFire());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.B) && !bot.shooting) {
                runningActions.add(bot.indexer.shootLeft());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X) && !bot.shooting) {
                runningActions.add(bot.indexer.shootRight());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.Y) && !bot.shooting) {
                runningActions.add(bot.indexer.shootBack());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && !bot.shooting) {
                runningActions.add(bot.indexer.shootPurple());
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && !bot.shooting) {
                runningActions.add(bot.indexer.shootGreen());
            }


            // CLIMB
            if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.lift.enableClosedLoop(!bot.lift.isClosedLoopEnabled());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                bot.lift.liftUp();
            }

            // FAILSAFES

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.switchAlliance();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                bot.limelight.relocalizeBotPose();
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

            bot.periodic();
            Bot.drive.localizer.update();
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
            telemetry.addData("Right Spot", bot.indexer.getRightColor());
            telemetry.addData("Left Spot", bot.indexer.getLeftColor());
            telemetry.addData("Back Spot", bot.indexer.getBackColor());

            telemetry.addData("Motif:", Indexer.getMotifPattern());


//            telemetry.addData("Odom Pose", Math.round(Bot.drive.localizer.getPose().position.x) + " " + Math.round(Bot.drive.localizer.getPose().position.y) + " " + Math.round(Math.toDegrees(Bot.drive.localizer.getPose().heading.log())));
//            telemetry.addData("LL Pose", Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).x + Turret.llxRLOffset) + " " + Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).y + Turret.llyRLOffset) + " " + Math.round(Turret.llBotPose.getOrientation().getYaw()));
            telemetry.addData("\nalliance", Bot.getAlliance());
            telemetry.addData("starting pos", Bot.getStartingPos());

            telemetry.addData("\nGoal Distance", Turret.trackingDistance);
            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());
            telemetry.addData("Error (Degs)", bot.turret.getErrorDegs());
            telemetry.addData("Power", bot.turret.getPower());
            telemetry.addData("Target RPM", Turret.shooterRpm);
            telemetry.addData("Current", bot.turret.shooter.getFilteredRPM());

            telemetry.addData("Loop ms", "%.1f", loopTimer.milliseconds());
            loopTimer.reset();


//            // Back A for hue
//            float backHue = bot.indexer.getHue(bot.indexer.colorBR()); // Back A sensor
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
//            telemetry.addData("BR Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorBR()));
//            telemetry.addData("BL Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorBL()));
//            telemetry.addData("RL Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorRL()));
//            telemetry.addData("RR Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorRR()));
//            telemetry.addData("LR Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorLR()));
//            telemetry.addData("LL Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorLL()));








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
//
//    private Bot bot;
//    private double driveSpeed = 1, driveMultiplier = 1 ;
//    private GamepadEx gp1;
//    private Thread thread;
//    private List<Action> runningActions = new ArrayList<>();
//    private boolean useStoredPose = true;
//
//    NormalizedRGBA colors;
//
//    public static boolean stallIntake = true, manualTurret = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
//
//        Bot.instance = null;
//        bot = Bot.getInstance(this);
//
//        gp1 = new GamepadEx(gamepad1);
//        bot.enableFullAuto(true);
//
//        // Initialize bot
////        bot.stopMotors();
//
////        waitForStart();
//
//        while (!isStarted()) {
//
//            gp1.readButtons();
//
//            TelemetryPacket packet = new TelemetryPacket();
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
//                bot.turret.resetEncoder();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
//                bot.switchAlliance();
//                useStoredPose = false;
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
//                bot.switchStartingPos();
//                useStoredPose = false;
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
//                useStoredPose = !useStoredPose;
//            }
//
//            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
//            telemetry.addData("STARTING POSITION (B)", Bot.getStartingPos());
//            telemetry.addData("STORED POSITION", useStoredPose);
//
//            telemetry.update();
//        }
//
//        if (!useStoredPose) {
//            if (Bot.isFar()) {
//                if (Bot.isBlue()) {
//                    Bot.drive.localizer.setPose(Pos.initialFarBluePose);
//                } else {
//                    Bot.drive.localizer.setPose(Pos.initialFarRedPose);
//                }
//            } else {
//                if (Bot.isBlue()) {
//                    Bot.drive.localizer.setPose(Pos.initialCloseBluePose);
//                } else {
//                    Bot.drive.localizer.setPose(Pos.initialCloseRedPose);
//                }
//            }
//        } else {
//            Bot.useStoredPose();
//        }
//
//
//        while (opModeIsActive() && !isStopRequested()) {
//            TelemetryPacket packet = new TelemetryPacket();
//
//            gp1.readButtons();
//
//            if (!bot.shooting) {
//                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
//                    bot.intake.intake();
//                } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
//                    bot.intake.reverse();
//                } else {
//                    if (stallIntake) {
//                        bot.intake.storage();
//                    } else {
//                        bot.intake.stop();
//                    }
//                }
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                stallIntake = !stallIntake;
//            }
//
//            // CLIMB
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
//                bot.lift.enableClosedLoop(!bot.lift.isClosedLoopEnabled());
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
//                bot.lift.liftUp();
//            }
//
//            // SHOOTING
//
//            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
//                bot.turret.enableShooter(true);
//            } else {
//                bot.turret.enableShooter(false);
//            }
//
//
//
//            // FAILSAFES
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
//                bot.switchAlliance();
//            }
//
////            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
////                bot.turret.relocalizeBotPose();
////            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
//                bot.resetPose();
//            }
//
//
//            bot.periodic();
//            // DRIVE
//            drive();
//
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//            runningActions = newActions;
//
//            // TELEMETRY
//
////
////            colors = bot.intake.color.getNormalizedColors();
////
////            telemetry.addData("rgb: ", colors.red + " " + colors.blue + " " + colors.green);
////
////            double r = colors.red, g = colors.green, b = colors.blue;
////            double cmax = Math.max(r, Math.max(g, b)), cmin = Math.min(r, Math.min(g, b));
////            double d = cmax - cmin;
////
////            double h = d == 0 ? 0 :
////                    cmax == r ? 60 * (((g - b) / d) % 6) :
////                            cmax == g ? 60 * (((b - r) / d) + 2) :
////                                    60 * (((r - g) / d) + 4);
////
////            double s = cmax == 0 ? 0 : (d / cmax);
////            double v = cmax;
////
////            telemetry.addData("hsv: ", h + " " + s + " " + v);
//
//
//            telemetry.addData("Odom Pose", Math.round(Bot.drive.localizer.getPose().position.x) + " " + Math.round(Bot.drive.localizer.getPose().position.y) + " " + Math.round(Math.toDegrees(Bot.drive.localizer.getPose().heading.log())));
////            telemetry.addData("LL Pose", Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).x + Turret.llxRLOffset) + " " + Math.round(Turret.llBotPose.getPosition().toUnit(DistanceUnit.INCH).y + Turret.llyRLOffset) + " " + Math.round(Turret.llBotPose.getOrientation().getYaw()));
//            telemetry.addData("\nalliance", Bot.getAlliance());
//            telemetry.addData("starting pos", Bot.getStartingPos());
////            telemetry.addData("\n", bot.intake.storageCount());
////            telemetry.addData("\nHolding Bottom", bot.intake.holdingBottom());
//////            telemetry.addData("Color Bottom", bot.intake.blbColor());
//////            telemetry.addData("Bottom Purple State", bot.intake.blb0.getState());
//////            telemetry.addData("Bottom Green State", bot.intake.blb1.getState());
////            telemetry.addData("\nHolding Middle", bot.intake.holdingMiddle());
//////            telemetry.addData("Color Middle", bot.intake.blmColor());
//////            telemetry.addData("Middle Purple State", bot.intake.blm0.getState());
//////            telemetry.addData("CMiddle Green State", bot.intake.blm1.getState());
////            telemetry.addData("\nHolding Top", bot.intake.holdingTop());
////            telemetry.addData("Color Top", bot.intake.bltColor());
////            telemetry.addData("Top Purple State", bot.intake.blt0.getState());
////            telemetry.addData("Top Green State", bot.intake.blt1.getState());
////
////            telemetry.addData("\nPose", Bot.drive.localizer.getPose());
////            telemetry.addData("Velocity", Bot.drive.localizer.update());
//            telemetry.addData("\nGoal Distance", Turret.trackingDistance);
//            telemetry.addData("Shoot Delay", Bot.shootDelay);
//
////
////            telemetry.addData("\ntx", Turret.tx);
////            telemetry.addData("ty", Turret.ty);
////
////            telemetry.addData("txAvg", bot.turret.txAvg);
////
////            telemetry.addData("correct distance", Turret.distance);
////            telemetry.addData( "tag angle", Turret.tAngle);
////            telemetry.addData("tOffset", Turret.tOffset);
//            telemetry.addData("Pos (Degs)", bot.turret.getPositionDegs());
//
//            telemetry.addData("auto target rpm", Turret.shooterRpm);
//            telemetry.addData("filtered rpm", bot.turret.shooter.getFilteredRPM());
//
////            telemetry.addData("\nLeft Climb Position", bot.lift.getLeftEncContinuousDeg());
////            telemetry.addData("Right Climb Position", bot.lift.getRightEncContinuousDeg());
////            telemetry.addData("Climb Loop?", bot.lift.isClosedLoopEnabled());
////            telemetry.addData("Left Power", bot.lift.leftPower);
////            telemetry.addData("Right Power", bot.lift.rightPower);
//////            telemetry.addData("Left PID out", bot.lift.leftPidOut);
//////            telemetry.addData("Right PID out", bot.lift.rightPidOut);
////            telemetry.addData("Left Climb Target", bot.lift.leftTargetDeg);
////            telemetry.addData("Right Climb Target", bot.lift.rightTargetDeg);
////            telemetry.addData("Offset", bot.lift.offset);
////            telemetry.addData("Roll", Turret.orientation.getRoll(AngleUnit.DEGREES));
//            telemetry.addData("Velocity", Bot.drive.localizer.update());
//            telemetry.update();
//
//
//        }
//    }
//
//    // Driving
//    private void drive() { // Robot centric, drive multiplier default 1
//        driveSpeed = driveMultiplier - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
//        driveSpeed = Math.max(0, driveSpeed);
////        bot.fixMotors();
//        com.arcrobotics.ftclib.geometry.Vector2d driveVector = new com.arcrobotics.ftclib.geometry.Vector2d(-gp1.getLeftX(), -gp1.getLeftY());
////                turnVector = new com.arcrobotics.ftclib.geometry.Vector2d(-gp1.getRightX(), 0);
////        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
////                driveVector.getY() * driveSpeed,
////                turnVector.getX() * driveSpeed
////        );
//
//        Bot.drive.setDrivePowers(new PoseVelocity2d(new com.acmerobotics.roadrunner.Vector2d(driveSpeed * gp1.getLeftY(),driveSpeed * -gp1.getLeftX()),driveSpeed * -gp1.getRightX()));
//    }
//}