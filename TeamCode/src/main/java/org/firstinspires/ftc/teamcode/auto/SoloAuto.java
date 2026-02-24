package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.tuning.ActionHelper;
import org.firstinspires.ftc.teamcode.auto.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "18 Ball Auto", group = "Competition")
public class SoloAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // ---------------- CONFIG STRUCT ----------------
    public static class AutoConfig {
        public boolean startFar = false;
        public boolean runPreload = true;
        public boolean runMid     = true;
        public int gateCycles = 3;
        public boolean runClose   = true;
        public boolean runOpenGate = false;
        public boolean runFar     = true;
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

    // 0 = starting position, 1 = preload, 2 = mid, 3 = gate, 4 = close, 5 = open gate, 6 = far, 7 = hp
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
        bot.setTargetGoalPose();
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
            addSegmentLine(7, "HP:      run (X) / delay (L/R)", "%b / %ds",
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
        if (builtAuto == null || addedAction) {
            builtAuto = buildAuto(Bot.drive, Bot.isBlue(), cfg);
        }

        telemetry.addData("Auto", "Built for %s", Bot.getAlliance());
        telemetry.addData("Segments", "preload:%b mid:%b gate:%d close:%b openGate:%b far:%b hp:%b",
                cfg.runPreload, cfg.runMid, cfg.gateCycles, cfg.runClose, cfg.runOpenGate, cfg.runFar, cfg.runHp);
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
            bot.limelight.trackObelisk();
            addedAction = true;
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            selectedSegment = (selectedSegment + 8 - 1) % 8;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            selectedSegment = (selectedSegment + 1) % 8;
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
            addedAction = true;
            switch (selectedSegment) {
                case 0:
                    cfg.startFar = false;
                    builtAuto = null;
                    break;
                case 1:
                    cfg.runPreload = true;
                    break;
                case 2:
                    cfg.runMid = true;
                    break;
                case 3:
                    cfg.gateCycles = 3; //2 before far, one after far
                    break;
                case 4:
                    cfg.runClose = true;
                    break;
                case 5:
                    cfg.runOpenGate = false;
                    break;
                case 6:
                    cfg.runFar = true;
                    break;
                case 7:
                    cfg.runHp = false;
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

        int gateCycles = Math.max(0, Math.min(3, cfg.gateCycles));

        builder = builder.stopAndAdd(() -> bot.limelight.trackObelisk());

        if (cfg.runPreload) {
            if (cfg.delayPreload > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayPreload));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd(bot.enableShooter())
                    .strafeToLinearHeading(Pos.closeShoot, Math.toRadians(-8)) //shoot once we've entered close zone
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));

            addedAction = true;
        }

        if (cfg.runMid) {
            if (cfg.delayMid > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayMid));
                addedAction = true;
                bot.sensorIntake(true);
            }
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .setTangent(Math.toRadians(180))
                    .splineToSplineHeading(Pos.blueMidIntake, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.position.x,
                            Pos.blueMidIntake.position.y + Pos.intakeDisp), drive.defaultVelConstraint, new ProfileAccelConstraint(-67, 70))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.1, bot.indexer.jiggleKickers())
                    .afterTime(0.85, (() -> bot.reverseIntake()));
            if (cfg.runClose && cfg.gateCycles == 0) {
                builder = builder
                        .strafeToSplineHeading(new Vector2d(Pos.blueCloseIntake.position.x, Pos.closeShoot.y), Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-67, 70))
                        .stopAndAdd(bot.indexer.shootRapidFire())
                        .stopAndAdd((() -> bot.disableShooter()));
            } else {
                builder = builder
                        .strafeToSplineHeading(Pos.closeShoot, Math.toRadians((110)))
                        .stopAndAdd(bot.indexer.shootRapidFire())
                        .stopAndAdd((() -> bot.disableShooter()));
            }
            addedAction = true;
        }

        builder = builder.stopAndAdd(() -> bot.limelight.trackAlliance());

        //GATE CYCLE 1
        builder = builder
                .stopAndAdd((() -> bot.sensorIntake(true)))
                .splineToSplineHeading(Pos.gate, Math.toRadians(85))
                .waitSeconds(0.825)
                .strafeToLinearHeading(Pos.gateIntaking.position, Math.toRadians(45))
                .waitSeconds(0.1)
                .stopAndAdd(bot.enableShooter());
        builder = builder
                .afterTime(0.2, bot.indexer.jiggleKickers())
                .afterTime(0.9, (() -> bot.reverseIntake()))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(Pos.blueCloseIntake.position.x, Pos.closeGateCycleShoot.y, Math.toRadians(90)), Math.toRadians(-52.5), drive.defaultVelConstraint, new ProfileAccelConstraint(-67, 70))
                .stopAndAdd(bot.indexer.shootRapidFire())
                .stopAndAdd((() -> bot.disableShooter()));
        addedAction = true;

        if (cfg.runClose) {
            if (cfg.delayClose > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayClose));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.position.x,
                            Pos.blueCloseIntake.position.y + Pos.closeIntake));


            builder = builder
                    .afterTime(0.01, bot.enableShooter())
                    .afterTime(0.4, bot.indexer.jiggleKickers())
                    .afterTime(1.1, (() -> bot.reverseIntake()));

            builder = builder
                    .setReversed(true)
                    .splineTo(Pos.closeShoot, Math.toRadians(-90));

            builder = builder
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        //GATE CYCLE 2
        builder = builder
                .stopAndAdd((() -> bot.sensorIntake(true)))
                .splineToSplineHeading(Pos.gate, Math.toRadians(85))
                .waitSeconds(0.825)
                .strafeToLinearHeading(Pos.gateIntaking.position, Math.toRadians(45))
                .waitSeconds(0.1)
                .stopAndAdd(bot.enableShooter());
        builder = builder
                .afterTime(0.2, bot.indexer.jiggleKickers())
                .afterTime(0.9, (() -> bot.reverseIntake()))
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(165))
                .stopAndAdd(bot.indexer.shootRapidFire())
                .stopAndAdd((() -> bot.disableShooter()));
        addedAction = true;

        if (cfg.runFar) {
            if (cfg.delayFar > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayFar));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.position.x,
                            Pos.blueFarIntake.position.y + Pos.intakeDisp))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.4, bot.indexer.jiggleKickers())
                    .afterTime(1.00, (() -> bot.reverseIntake()))
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians((110)))
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));

            addedAction = true;
        }

        //GATE CYCLE 3
        builder = builder
                .stopAndAdd((() -> bot.sensorIntake(true)))
                .splineToSplineHeading(Pos.gate, Math.toRadians(85))
                .waitSeconds(0.825)
                .strafeToLinearHeading(Pos.gateIntaking.position, Math.toRadians(45))
                .waitSeconds(0.1)
                .stopAndAdd(bot.enableShooter());
        builder = builder
                .afterTime(0.2, bot.indexer.jiggleKickers())
                .afterTime(0.9, (() -> bot.reverseIntake()))
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShootPark, Math.toRadians(100))
                .stopAndAdd(bot.indexer.shootRapidFire())
                .stopAndAdd((() -> bot.disableShooter()));
        addedAction = true;


        if (!addedAction) {
            builder = builder.stopAndAdd((() -> telemetry.addData("Auto", "No segments enabled")));
        }

        addedAction = false;
        return builder.build();
    }
}
