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
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.tuning.ActionHelper;
import org.firstinspires.ftc.teamcode.auto.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "Adaptive CS Auto", group = "Competition")
public class AdaptiveCSAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // ---------------- CONFIG STRUCT ----------------
    public static class AutoConfig {
        public boolean startFar = false;
        public boolean runPreload = true;
        public boolean runMid     = true;
        public int gateCycles = 1;
        public boolean runClose   = true;
        public boolean runFar     = true;
        public boolean runHp      = true;

        public int delayPreload = 0;
        public int delayGate    = 0;
        public int delayClose   = 0;
        public int delayMid     = 0;
        public int delayFar     = 0;
        public int delayHp      = 0;
    }

    private AutoConfig cfg = new AutoConfig();

    // 0 = starting position, 1 = preload, 2 = mid, 3 = gate, 4 = close, 5 = far, 6 = hp
    private int selectedSegment = 0;

    private Action builtAuto = null;
    private TrajectoryActionBuilder builder;

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
            addSegmentLine(5, "Far:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runFar, cfg.delayFar);
            addSegmentLine(6, "HP:      run (X) / delay (L/R)", "%b / %ds",
                    cfg.runHp, cfg.delayHp);
            if (builtAuto == null) {
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
        if (builtAuto == null) {
            builtAuto = buildAuto(Bot.drive, Bot.isBlue(), cfg);
        }

        telemetry.addData("Auto", "Built for %s", Bot.getAlliance());
        telemetry.addData("Segments", "preload:%b mid:%b gate:%d close:%b far:%b hp:%b",
                cfg.runPreload, cfg.runMid, cfg.gateCycles, cfg.runClose, cfg.runFar, cfg.runHp);
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
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            selectedSegment = (selectedSegment + 7 - 1) % 7;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            selectedSegment = (selectedSegment + 1) % 7;
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
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
                    cfg.gateCycles = (cfg.gateCycles + 1) % 4;
                    break;
                case 4:
                    cfg.runClose = !cfg.runClose;
                    break;
                case 5:
                    cfg.runFar = !cfg.runFar;
                    break;
                case 6:
                    cfg.runHp = !cfg.runHp;
                    break;
            }
        }

        int delta = 0;
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            delta = +1;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
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
                    cfg.delayFar =
                            clampDelay(cfg.delayFar + delta);
                    break;
                case 6:
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

        boolean addedAction = false;
        int gateCycles = Math.max(0, Math.min(3, cfg.gateCycles));

        builder = builder.stopAndAdd(() -> bot.limelight.trackObelisk());

        if (cfg.runPreload) {
            if (cfg.delayPreload > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayPreload));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd(bot.enableShooter())
                    .strafeToLinearHeading(Pos.closeShoot, Math.toRadians(0))
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        if (cfg.runMid) {
            if (cfg.delayMid > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayMid));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd((() -> bot.intake.intake()))
//                    .turnTo(Math.toRadians(135))
//                    .setTangent(Math.toRadians(135))
                    .splineTo(new Vector2d(10, 19), Math.toRadians(90))
                    .splineTo(Pos.blueMidIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.position.x,
                            Pos.blueMidIntake.position.y + 22))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.4, (() -> bot.intake.reverse()))
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        builder = builder.stopAndAdd(() -> bot.limelight.trackAlliance());

        if (cfg.delayGate > 0 && gateCycles > 0) {
            builder = builder.stopAndAdd(new SleepAction(cfg.delayGate));
            addedAction = true;
        }
        for (int gateIndex = 0; gateIndex < gateCycles; gateIndex++) {
            builder = builder
                    .strafeToLinearHeading(Pos.gate.position, Pos.gate.heading) //TODO try spline heading
                    .stopAndAdd((() -> bot.intake.intake()))
                    .waitSeconds(1)
                    .stopAndAdd(bot.enableShooter())
                    .waitSeconds(1)
                    .stopAndAdd((() -> bot.intake.reverse()))
                    .setReversed(true)
                    .splineTo(Pos.closeShoot, Math.toRadians(-135))
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        if (cfg.runClose) {
            if (cfg.delayClose > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayClose));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd((() -> bot.intake.intake()))
                    .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90),
                            drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 65))
                    .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.position.x,
                            Pos.blueCloseIntake.position.y + 22))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.4, (() -> bot.intake.reverse()))
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                    .stopAndAdd(bot.indexer.shootMotif())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        if (cfg.runFar) {
            if (cfg.delayFar > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayFar));
                addedAction = true;
            }
            builder = builder

                    .stopAndAdd((() -> bot.intake.intake()))
                    .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.position.x,
                            Pos.blueFarIntake.position.y + 22))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.4, (() -> bot.intake.reverse()))
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(155))
                    .stopAndAdd(bot.indexer.shootMotif())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        if (cfg.runHp) {
            if (cfg.delayHp > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayHp));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd((() -> bot.intake.intake()))
                    .splineTo(Pos.blueHpIntake.component1(), Pos.blueHpIntake.component2())
//                    .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 11.5, Pos.blueHpIntake.position.y))

                    .stopAndAdd((() -> bot.intake.reverse()))
                    .setReversed(true)
                    .afterTime(0.1, bot.enableShooter())
                    .splineTo(Pos.closeShoot, Math.toRadians(155))
                    .stopAndAdd(bot.indexer.shootMotif());
            addedAction = true;
        }
        builder = builder.strafeToConstantHeading(Pos.park);

        if (!addedAction) {
            builder = builder.stopAndAdd((() -> telemetry.addData("Auto", "No segments enabled")));
        }
        return builder.build();
    }
}