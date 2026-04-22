package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.tuning.ActionHelper;
import org.firstinspires.ftc.teamcode.auto.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "24 Ball Auto", group = "Competition")
public class Playoff24BallAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // ---------------- CONFIG STRUCT ----------------
    public static class AutoConfig {
        public boolean startFar = false;
        public boolean runPreload = true;
        public boolean runMid     = true;
        public int gateCycles = 5;
        public boolean runClose   = true;
        public boolean runOpenGate = false;
        public boolean runFar     = false;
        public boolean runPushPark = false;
        public boolean runHp      = false;

        public int delayPreload = 0;
        public int delayGate    = 0;
        public int delayClose   = 0;
        public int delayOpenGate = 0;
        public int delayMid     = 0;
        public int delayFar     = 0;
        public int delayHp      = 0;
    }

    private AutoConfig cfg = new AutoConfig();

    // 0 = starting position, 1 = preload, 2 = mid, 3 = gate, 4 = close, 5 = open gate, 6 = far, 7 = push park, 8 = hp
    private int selectedSegment = 0;

    private Action builtAuto = null;
    private TrajectoryActionBuilder builder;
    private boolean addedAction = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        Bot.instance = null;
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);

        MecanumDrive drive = Bot.drive;

        bot.enableFullAuto(true);
        bot.enableShooter(false);
        bot.setAllianceBlue();
        applyStartingPosition(drive);
        bot.trackObelisk();
        bot.indexer.resetIndexer();
        Actions.runBlocking(bot.indexer.leftHolder.resetFastAction());
        Bot.drive.localizer.recalibrateIMU();

        builtAuto = buildAuto(Bot.drive, Bot.isBlue(), cfg);
        bot.limelight.trackObelisk();

        // ------------- INIT LOOP: CONFIGURE AUTO -------------
        while (opModeInInit() && !isStopRequested() && !isStarted()) {
            handleConfigInput();
            applyStartingPosition(drive);

            telemetry.addData("ALLIANCE (A)", "<big><b>%s</b></big>", Bot.getAlliance());
            telemetry.addData("<big><b><u>Motif</big></b></u>", "<big><b> "+ Bot.motif + "</big></b></u>");
            addSegmentLine(0, "STARTING POSITION (X)", "%s", cfg.startFar ? "Far" : "Close");
            addSegmentLine(1, "Preload: run (X) / delay (L/R)", "%b / %ds",
                    cfg.runPreload, cfg.delayPreload);
            addSegmentLine(2, "Mid:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runMid, cfg.delayMid);
            addSegmentLine(3, "Gate:    cycles (X) / delay (L/R)", "%d / %ds",
                    cfg.gateCycles, cfg.delayGate);
            addSegmentLine(4, "Close:   run (X) / delay (L/R)", "%b / %ds",
                    cfg.runClose, cfg.delayClose);
            addSegmentLine(5, "Open Gate: run (X) / delay (L/R)", "%b / %ds",
                    cfg.runOpenGate, cfg.delayOpenGate);
            addSegmentLine(6, "Far:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runFar, cfg.delayFar);
            addSegmentLine(7, "Push Park: run (X)", "%b", cfg.runPushPark);
            addSegmentLine(8, "HP:      run (X) / delay (L/R)", "%b / %ds",
                    cfg.runHp, cfg.delayHp);
            if (builtAuto == null || addedAction) {
                telemetry.addData("", "<big><b><font color='red'>AUTO NOT BUILT (Y to build)</font></b></big>");
            } else {
                telemetry.addLine("<big><b><font color='green'> Built! (Y to build again)</font></b></big>");
            }
            if (builtAuto != null) {
                telemetry.addData("build", builtAuto);
            }
            telemetry.update();

            bot.periodic();
        }

        waitForStart();
        if (isStopRequested()) return;
        bot.setTargetGoalPose();
        bot.enableShooter(true);
        if (builtAuto == null || addedAction) {
            builtAuto = buildAuto(Bot.drive, Bot.isBlue(), cfg);
        }

        telemetry.addData("Auto", "Built for %s", Bot.getAlliance());
        telemetry.addData("Segments", "preload:%b mid:%b gate:%d close:%b openGate:%b far:%b pushPark:%b hp:%b",
                cfg.runPreload, cfg.runMid, cfg.gateCycles, cfg.runClose, cfg.runOpenGate, cfg.runFar, cfg.runPushPark, cfg.runHp);
        telemetry.update();

        applyStartingPosition(drive);

        if (builtAuto == null) {
            telemetry.addData("Auto", "Build failed, nothing to run");
            telemetry.update();
            return;
        } else {
            telemetry.addData("build", builtAuto.toString());
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(builtAuto)
                )
        );
    }

    private void applyStartingPosition(MecanumDrive drive) {
        if (cfg.startFar) {
            bot.setFar();
            if (Bot.isBlue()) {
                drive.localizer.setPose(Pos.initialFarBluePose);
            } else {
                drive.localizer.setPose(Pos.initialFarRedPose);
            }
        } else {
            bot.setClose();
            if (Bot.isBlue()) {
                drive.localizer.setPose(Pos.initialCloseBluePose);
            } else {
                drive.localizer.setPose(Pos.initialCloseRedPose);
            }
        }
    }

    // ---------------- CONFIG INPUT HANDLING ----------------

    private void handleConfigInput() {
        gp1.readButtons();

        if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
            bot.switchAlliance();
            bot.trackObelisk();
            addedAction = true;
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            selectedSegment = (selectedSegment + 9 - 1) % 9;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            selectedSegment = (selectedSegment + 1) % 9;
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
            addedAction = true;
            switch (selectedSegment) {
                case 0:
                    cfg.startFar = !cfg.startFar;
                    builtAuto = null;
                    break;
                case 1:
                    cfg.runPreload = true;
                    break;
                case 2:
                    cfg.runMid = !cfg.runMid;
                    break;
                case 3:
                    cfg.gateCycles = (cfg.gateCycles + 1) % 6;
                    break;
                case 4:
                    cfg.runClose = !cfg.runClose;
                    break;
                case 5:
                    cfg.runOpenGate = !cfg.runOpenGate;
                    break;
                case 6:
                    cfg.runFar = !cfg.runFar;
                    break;
                case 7:
                    cfg.runPushPark = !cfg.runPushPark;
                    break;
                case 8:
                    cfg.runHp = !cfg.runHp;
                    break;
            }
        }

        int delta = 0;
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            addedAction = true;
            delta = +1;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            addedAction = true;
            delta = -1;
        }

        if (delta != 0) {
            switch (selectedSegment) {
                case 0:
                    break;
                case 1:
                    cfg.delayPreload =
                            clampDelay(cfg.delayPreload + delta);
                    break;
                case 2:
                    cfg.delayMid =
                            clampDelay(cfg.delayMid + delta);
                    break;
                case 3:
                    cfg.delayGate =
                            clampDelay(cfg.delayGate + delta);
                    break;
                case 4:
                    cfg.delayClose =
                            clampDelay(cfg.delayClose + delta);
                    break;
                case 5:
                    cfg.delayOpenGate =
                            clampDelay(cfg.delayOpenGate + delta);
                    break;
                case 6:
                    cfg.delayFar =
                            clampDelay(cfg.delayFar + delta);
                    break;
                case 7:
                    break;
                case 8:
                    cfg.delayHp =
                            clampDelay(cfg.delayHp + delta);
                    break;
            }
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
            builtAuto = buildAuto(Bot.drive, Bot.isBlue(), cfg);
        }
    }

    private int clampDelay(int d) {
        if (d < 0) return 0;
        if (d > 28) return 28;
        return d;
    }

    private void addSegmentLine(int segmentIndex, String label, String format, Object... args) {
        if (selectedSegment == segmentIndex) {
            String value = String.format(format, args);
            telemetry.addData("", "<b>%s %s</b>", label, value);
        } else {
            telemetry.addData(label, format, args);
        }
    }

    // ---------------- BUILDER: BUILD BLUE/RED AUTO ----------------

    private Action buildAuto(MecanumDrive drive, boolean isBlue, AutoConfig cfg) {
        cfg.runPreload = true;
        Pose2d startPose = cfg.startFar ? Pos.initialFarBluePose : Pos.initialCloseBluePose;
        builder = isBlue
                ? drive.actionBuilderBlue(startPose)
                : drive.actionBuilderRed(startPose);

        int gateCycles = cfg.gateCycles;

        if (cfg.runPreload) {
            if (cfg.delayPreload > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayPreload));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd(() -> bot.stopIntake())
                    .stopAndAdd(bot.enableShooter())
                    .stopAndAdd(new SleepAction(0.3))
                    .afterTime(0.45, bot.indexer.shootRapidFire())
                    .strafeToSplineHeading(Pos.firstShoot, Math.toRadians(175));

            addedAction = true;
        }

        if (cfg.runMid) {
            if (cfg.delayMid > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayMid));
                addedAction = true;
                bot.sensorIntake(true);
            }
            builder = builder
                    .afterTime(0.01, (() -> bot.sensorIntake(true)))
                    .splineToSplineHeading(new Pose2d(Pos.blueMidIntake.position.x,
                            Pos.blueMidIntake.position.y + Pos.intakeDisp,
                            Math.toRadians(80)), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-75, 80))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.25, bot.indexer.jiggleKickers())
                    .afterTime(0.85, (() -> bot.reverseIntake()));
            if (cfg.runClose && cfg.gateCycles == 0) {
                builder = builder
                        .strafeToSplineHeading(new Vector2d(Pos.blueCloseIntake.position.x, Pos.closeShoot.y), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-67, 70))
                        .stopAndAdd(bot.indexer.shootRapidFire());
            } else {
                builder = builder
                        .afterTime(1.2, bot.indexer.shootRapidFire())
                        .strafeToSplineHeading(Pos.closeShoot, Math.toRadians((cfg.gateCycles == 0 ? 135 : 110)), drive.defaultVelConstraint, new ProfileAccelConstraint(-60, 80));
            }
            addedAction = true;
        }

        builder = builder.stopAndAdd(() -> bot.limelight.trackAlliance());

        if (cfg.delayGate > 0 && gateCycles > 0) {
            builder = builder.stopAndAdd(new SleepAction(cfg.delayGate));
            addedAction = true;
        }
        for (int gateIndex = 0; gateIndex < gateCycles; gateIndex++) {
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineToSplineHeading(Pos.gate, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 72))
                    .waitSeconds(0.6);
            builder = (gateIndex != gateCycles - 1) ?
                    builder
                            .afterTime(0.2, bot.indexer.jiggleKickers())
                            .afterTime(0.9, (() -> bot.reverseIntake()))
                            .setReversed(true)
                            .afterTime(1.3, bot.indexer.shootRapidFire())
                            .strafeToSplineHeading(Pos.closeGateCycleShoot, Math.toRadians(110), drive.defaultVelConstraint, new ProfileAccelConstraint(-67, 80))
                    :
                    builder
                            .afterTime(0.2, bot.indexer.jiggleKickers())
                            .afterTime(0.9, (() -> bot.reverseIntake()))
                            .setReversed(true)
                            .afterTime(1.45, bot.indexer.shootRapidFire())
                            .strafeToSplineHeading(new Vector2d(Pos.blueCloseIntake.position.x, Pos.closeGateCycleShoot.y), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-67, 80));
//                            .stopAndAdd(bot.indexer.shootRapidFire());
            addedAction = true;
        }

        if (cfg.runClose) {
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineTo(new Vector2d(Pos.blueCloseIntake.position.x,
                            Pos.blueCloseIntake.position.y + Pos.closeIntake), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-67, 80))
                    .afterTime(0.01, bot.enableShooter());

            builder = builder
                    .afterTime(0.4, bot.indexer.jiggleKickers())
                    .afterTime(0.8, bot.indexer.shootRapidFire())
                    .afterTime(0.9, (() -> bot.reverseIntake()))
                    .splineToSplineHeading(new Pose2d(Pos.closeShootPark, Math.toRadians(135)), Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-90, 90));

            builder = builder.waitSeconds(0.3);
            //.stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }


        if (!addedAction) {
            builder = builder.stopAndAdd((() -> telemetry.addData("Auto", "No segments enabled")));
        }

        addedAction = false;
        return builder.build();
    }
}
