package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.Pos;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "V2 Bot Tester!!!", group = "Competition")
public class NewBotTester extends LinearOpMode {

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
//        bot.enableFullAuto(true);
//        bot.setTargetGoalPose();
//        stallIntake = true;


        // Initialize bot
//        bot.stopMotors();

//        waitForStart();

        while (!isStarted()) {


            gp1.readButtons();
            gp2.readButtons();

            TelemetryPacket packet = new TelemetryPacket();

//            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
//                bot.turret.resetEncoder();
//            }

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                bot.switchStartingPos();
                useStoredPose = false;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                useStoredPose = !useStoredPose;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.indexer.motifPattern="GPP";
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.indexer.motifPattern="PPG";
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.indexer.motifPattern="PGP";
            }


            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
            telemetry.addData("STARTING POSITION (B)", Bot.getStartingPos());
            telemetry.addData("STORED POSITION", useStoredPose);
            telemetry.addData("Motif:", bot.indexer.motifPattern);
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

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            gp2.readButtons();
            bot.shooting = false;
            bot.turret.enableFullAuto(false);
            bot.turret.enablePositionTracking(false);



            if (!bot.shooting) {
                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                    bot.intake.intake();
                } else if (gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                    bot.intake.reverse();
                } else if (bot.indexer.countBalls()==3){
                    bot.intake.reverse();
                } else if (stallIntake){
                    bot.intake.storage();
                } else {
                    bot.intake.stop();
                }
            }



            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                stallIntake = !stallIntake;
            }

            // TURRET

//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { //everything!
//                bot.enableFullAuto(true);
//                manualTurret = false;
//            }
//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) { //position tracking
//                bot.enableFullAuto(false);
//                bot.turret.enablePositionTracking(true);
//                manualTurret = false;
//            }
//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { //no tracking
//                bot.enableFullAuto(false);
//                manualTurret = true;
//            }

            // SHOOTING

            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                bot.turret.shooter.setManualPower(2500);
                bot.turret.enableShooter(true);
            } else {
                bot.turret.enableShooter(false);
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.indexer.resetIndexer();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.indexer.shootMotifDirect();
            }

            if (gp1.getButton(GamepadKeys.Button.B) && !bot.shooting) {
                runningActions.add(bot.indexer.shootLeft());
            }
            if (gp1.getButton(GamepadKeys.Button.X) && !bot.shooting) {
                runningActions.add(bot.indexer.shootRight());
            }
            if (gp1.getButton(GamepadKeys.Button.Y) && !bot.shooting) {
                runningActions.add(bot.indexer.shootBack());
            }


//            if (gp1.getButton(GamepadKeys.Button.DPAD_LEFT)) {
//                bot.indexer.isGreen(bot.indexer.colorBL);
//                bot.indexer.isGreen(bot.indexer.colorBR);
//                bot.indexer.isPurple(bot.indexer.colorRR);
//                bot.indexer.isPurple(bot.indexer.colorRL);
//                bot.indexer.isNone(bot.indexer.colorLL);
//                bot.indexer.isNone((bot.indexer.colorLR));
//            }

//            if (gp1.getButton(GamepadKeys.Button.Y) && !bot.shooting) {
//                if (bot.indexer.isGreenRight(bot.indexer.colorRR)){
//                    runningActions.add(bot.shootRight());
//
//                } else if (bot.indexer.isGreenRight(bot.indexer.colorLL)){
//                    runningActions.add(bot.shootLeft());
//
//                } else if (bot.indexer.isGreenRight(bot.indexer.colorBR)){
//                    runningActions.add(bot.shootBack());
//                }
//
//          }



//            if (gp1.getButton(GamepadKeys.Button.A) && !bot.shooting && !gp1.isDown(GamepadKeys.Button.START)) {
//                runningActions.add(bot.shootLRB());
//            }

            // FAILSAFES

//            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
//                bot.switchAlliance();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
//                bot.turret.relocalizeBotPose();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
//                bot.resetPose();
//            }
//
//
//            if (manualTurret) {
//                bot.turret.runManual(gp2.getLeftX());
//            }

//            if (gp2.wasJustPressed(GamepadKeys.Button.BACK)) {
//                bot.turret.resetEncoder();
//            }



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

            telemetry.addLine("=== BALL COLORS ===");
            telemetry.addData("Right Spot", bot.indexer.getRightColor());
            telemetry.addData("Left Spot", bot.indexer.getLeftColor());
            telemetry.addData("Back Spot", bot.indexer.getBackColor());

            telemetry.addData("Motif:", bot.indexer.motifPattern);


            // Back A for hue
            float backHue = bot.indexer.getHue(bot.indexer.colorBR); // Back A sensor
            telemetry.addLine("=== BACK SENSOR HUE ===");
// Back B for distance
            double backDist = bot.indexer.safeDistance(bot.indexer.colorBL); // Back B sensor
            telemetry.addLine("=== BACK SENSOR DISTANCE ===");
            telemetry.addData("Back B Distance (mm)", "%.1f", backDist);



            telemetry.addLine("=== DIHstance pls speed i need dihs ===");
            telemetry.addData("BR Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorBR));
            telemetry.addData("BL Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorBL));
            telemetry.addData("RL Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorRL));
            telemetry.addData("RR Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorRR));
            telemetry.addData("LR Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorLR));
            telemetry.addData("LL Distance (mm)", "%.1f", bot.indexer.safeDistance(bot.indexer.colorLL));

            telemetry.addLine("=== HUEGE AHHH ===");
            telemetry.addData("BR Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorBR));
            telemetry.addData("BL Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorBL));
            telemetry.addData("RL Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorRL));
            telemetry.addData("RR Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorRR));
            telemetry.addData("LR Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorLR));
            telemetry.addData("LL Hue", "%.1f", bot.indexer.getHue(bot.indexer.colorLL));








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
