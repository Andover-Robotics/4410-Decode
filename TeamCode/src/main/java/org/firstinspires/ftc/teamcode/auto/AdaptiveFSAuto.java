package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "Adaptive Far Shot Auto (No Motif)", group = "Competition")
public class AdaptiveFSAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // ---------------- CONFIG STRUCT ----------------
    public static class AutoConfig {
        public boolean runPreload = true;
        public boolean runMid     = false;
        public int gateCycles = 0;
        public boolean runClose   = false;
        public boolean runHp      = true;
        public boolean runFar     = true;
        public int tunnelCycles   = 3;

        public int delayPreload = 0;
        public int delayGate    = 0;
        public int delayClose   = 0;
        public int delayMid     = 0;
        public int delayHp      = 0;
        public int delayFar     = 0;
        public int delayTunnel  = 0;
        public int intervalTunnel = 0;
    }

    private AutoConfig cfg = new AutoConfig();

    // 0 = preload, 1 = mid, 2 = gate, 3 = close, 4 = hp, 5 = far, 6 = tunnel
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
            addSegmentLine(0, "Preload: run (X) / delay (L/R)", "%b / %ds",
                    cfg.runPreload, cfg.delayPreload);
            addSegmentLine(1, "Mid:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runMid, cfg.delayMid);
            addSegmentLine(2, "Gate:    cycles (X) / delay (L/R)", "%d / %ds",
                    cfg.gateCycles, cfg.delayGate);
            addSegmentLine(3, "Close:   run (X) / delay (L/R)", "%b / %ds",
                    cfg.runClose, cfg.delayClose);
            addSegmentLine(4, "Far:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runFar, cfg.delayFar);
            addSegmentLine(5, "HP:      run (X) / delay (L/R)", "%b / %ds",
                    cfg.runHp, cfg.delayHp);
            addSegmentLine(6, "Tunnel:  cycles (X) / delay (L/R) / interval (LB/RB)", "%d / %ds / %ds",
                    cfg.tunnelCycles, cfg.delayTunnel, cfg.intervalTunnel);
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
        telemetry.addData("Segments", "preload:%b mid:%b gate:%d close:%b far:%b hp:%b tunnel:%d",
                cfg.runPreload, cfg.runMid, cfg.gateCycles, cfg.runClose, cfg.runFar, cfg.runHp, cfg.tunnelCycles);
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
        bot.setFar();
        if (Bot.isBlue()) {
            drive.localizer.setPose(Pos.initialFarBluePose);
        } else {
            drive.localizer.setPose(Pos.initialFarRedPose);
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
            selectedSegment = (selectedSegment + 7 - 1) % 7;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            selectedSegment = (selectedSegment + 1) % 7;
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
            addedAction = true;
            switch (selectedSegment) {
                case 0:
                    cfg.runPreload = !cfg.runPreload;
                    break;
                case 1:
                    cfg.runMid = !cfg.runMid;
                    break;
                case 2:
                    cfg.gateCycles = (cfg.gateCycles + 1) % 4;
                    break;
                case 3:
                    cfg.runClose = !cfg.runClose;
                    break;
                case 4:
                    cfg.runFar = !cfg.runFar;
                    break;
                case 5:
                    cfg.runHp = !cfg.runHp;
                    break;
                case 6:
                    cfg.tunnelCycles = (cfg.tunnelCycles + 1) % 6;
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
                    cfg.delayPreload =
                            clampDelay(cfg.delayPreload + delta);
                    break;
                case 1:
                    cfg.delayMid =
                            clampDelay(cfg.delayMid + delta);
                    break;
                case 2:
                    cfg.delayGate =
                            clampDelay(cfg.delayGate + delta);
                    break;
                case 3:
                    cfg.delayClose =
                            clampDelay(cfg.delayClose + delta);
                    break;
                case 4:
                    cfg.delayFar =
                            clampDelay(cfg.delayFar + delta);
                    break;
                case 5:
                    cfg.delayHp =
                            clampDelay(cfg.delayHp + delta);
                    break;
                case 6:
                    cfg.delayTunnel =
                            clampDelay(cfg.delayTunnel + delta);
                    break;
            }
        }

        int intervalDelta = 0;
        if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            intervalDelta = 1;
            addedAction = true;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            intervalDelta = -1;
            addedAction = true;
        }

        if (intervalDelta != 0 && selectedSegment == 6) {
            cfg.intervalTunnel = clampDelay(cfg.intervalTunnel + intervalDelta);
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
        Pose2d startPose = Pos.initialFarBluePose;
        builder = isBlue
                ? drive.actionBuilderBlue(startPose)
                : drive.actionBuilderRed(startPose);

        int gateCycles = Math.max(0, Math.min(3, cfg.gateCycles));
        int tunnelCycles = Math.max(0, Math.min(5, cfg.tunnelCycles));

        builder = builder.stopAndAdd(() -> bot.limelight.trackObelisk());

        if (cfg.runPreload) {
            if (cfg.delayPreload > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayPreload));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd(bot.enableShooter())
                    .waitSeconds(0.9)
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd(bot.disableShooter());
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
                    .splineTo(Pos.blueMidIntake.component1(), Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.position.x,
                            Pos.blueMidIntake.position.y + Pos.intakeDisp))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.1, bot.indexer.jiggleKickers())
                    .afterTime(0.85, (() -> bot.reverseIntake()))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(Pos.farShoot, Math.toRadians(5)), Math.toRadians(-180))
                    .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
                    .stopAndAdd(bot.indexer.shootRapidFire());
            addedAction = true;
        }

        builder = builder.stopAndAdd(() -> bot.limelight.trackAlliance());

        if (cfg.delayGate > 0 && gateCycles > 0) {
            builder = builder.stopAndAdd(new SleepAction(cfg.delayGate));
            addedAction = true;
        }
        for (int gateIndex = 0; gateIndex < gateCycles; gateIndex++) { //TODO - Make sure this doesn't hit far balls, if it does, then add another .splineTo above line 302 that goes to a position that is just 10 inches lower y than the gate position
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineTo(Pos.gate.component1(), Math.toRadians(90))
                    .waitSeconds(0.5)
                    .strafeToLinearHeading(Pos.gateIntaking.position, Math.toRadians(45))
                    .waitSeconds(0.35)
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.2, bot.indexer.jiggleKickers())
                    .afterTime(0.9, (() -> bot.reverseIntake()))
                    .setTangent(Math.toRadians(-85));
            builder = (gateIndex == gateCycles - 1) ?
                    builder
                    .splineToSplineHeading(new Pose2d(Pos.farShoot, Math.toRadians(45)), Math.toRadians(-180)) // TODO tune this move to not hit the far balls - we need to do far after gate because otherwise overflow will become an issue
                    .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter())) :
                    builder
                    .splineToSplineHeading(new Pose2d(Pos.farShoot, Math.toRadians(5)), Math.toRadians(-180)) // TODO tune this move to not hit the far balls - we need to do far after gate because otherwise overflow will become an issue
                    .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
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
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.position.x,
                            Pos.blueCloseIntake.position.y + Pos.closeIntake))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.4, bot.indexer.jiggleKickers())
                    .afterTime(1.1, (() -> bot.reverseIntake()))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(Pos.farShoot, Math.toRadians(5)), Math.toRadians(-180))
                    .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        if (cfg.runFar) {
            if (cfg.delayFar > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayFar));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.position.x,
                            Pos.blueFarIntake.position.y + Pos.farIntakeFarAuto))
                    .stopAndAdd(bot.enableShooter())
                    .afterTime(0.4, bot.indexer.jiggleKickers())
                    .afterTime(0.8, (() -> bot.reverseIntake()))
                    .setReversed(true)
                    .splineTo(Pos.farShoot, Math.toRadians(-135))
                    .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
                    .stopAndAdd(bot.indexer.shootRapidFire())
                    .stopAndAdd((() -> bot.disableShooter()));
            addedAction = true;
        }

        if (cfg.runHp) {
            if (cfg.delayHp > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayHp));
                addedAction = true;
            }
            builder = builder
                    .stopAndAdd((() -> bot.sensorIntake(true)))
                    .splineTo(Pos.blueHpSideInterIntake.position, Math.toRadians(90)) //gov cup hp intakes
                    .splineTo(Pos.blueHpSideIntake.position, Math.toRadians(-180))
                    .waitSeconds(0.15)
//                    .splineTo(Pos.blueHpFarInterIntake.position, Pos.blueHpFarInterIntake.heading)
//                    .splineTo(Pos.blueHpFarIntake.position, Pos.blueHpFarIntake.heading)
//                    .waitSeconds(0.15)

                    .afterTime(0.01, new SequentialAction(
                            bot.enableShooter(),
                            new SleepAction(0.5),
                            new InstantAction((() -> bot.reverseIntake()))
                    ))
                    .strafeToSplineHeading(Pos.farShoot, Math.toRadians(60))
                    .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
                    .stopAndAdd(bot.indexer.shootRapidFire());
            addedAction = true;
        }

        if (tunnelCycles > 0) {
            if (cfg.delayTunnel > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayTunnel));
                addedAction = true;
            }

            for (int tunnelIndex = 0; tunnelIndex < tunnelCycles; tunnelIndex++) {
                if (tunnelIndex > 0 && cfg.intervalTunnel > 0) {
                    builder = builder.stopAndAdd(new SleepAction(cfg.intervalTunnel));
                }
                if (tunnelIndex == 0 || tunnelIndex == 2) {
                    builder = builder
                            // old tunnel intaking (gov cup)
//                        .stopAndAdd((() -> bot.sensorIntake(true)))
//                        .splineTo(Pos.blueSecretTunnelStart.position, Math.toRadians(45))
//                        .splineTo(new Vector2d(Pos.blueSecretTunnelStart.position.x + 35, Pos.blueSecretTunnelStart.position.y), 0)
//                        .afterTime(0.50, (() -> bot.reverseIntake()))
//                        .setReversed(true)
//                        .splineTo(Pos.farShoot, Math.toRadians(-135))
//                        .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
//                        .stopAndAdd(bot.indexer.shootRapidFire())
//                        .stopAndAdd((() -> bot.disableShooter()));

                            .stopAndAdd((() -> bot.sensorIntake(true)))
                            .splineTo(new Vector2d(Pos.blueSecretTunnelStart.position.x + 17, Pos.blueSecretTunnelStart.position.y), Math.toRadians(150))
                            .strafeToLinearHeading(Pos.blueSecretTunnelStart.position, Math.toRadians(-180))
                            .stopAndAdd(bot.enableShooter())
                            .afterTime(1.2, (() -> bot.reverseIntake()))
                            .setReversed(true)
                            .splineTo(Pos.farShoot, Math.toRadians(-100))
                            .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
                            .stopAndAdd(bot.indexer.shootRapidFire())
                            .stopAndAdd((() -> bot.disableShooter()));
                } else {
                    builder = builder
                            .stopAndAdd((() -> bot.sensorIntake(true)))
                            .splineTo(Pos.blueHpCycle.position, Math.toRadians(90))
                            .stopAndAdd(bot.enableShooter())
                            .afterTime(1.5, (() -> bot.reverseIntake()))
                            .setReversed(true)
                            .splineTo(Pos.farShoot, Math.toRadians(-135))
                            .stopAndAdd(new InstantAction((() -> bot.stopIntake())))
                            .stopAndAdd(bot.indexer.shootRapidFire())
                            .stopAndAdd((() -> bot.disableShooter()));
                }

                addedAction = true;
            }
        }

        builder = builder.strafeToConstantHeading(Pos.farPark);

        if (!addedAction) {
            builder = builder.stopAndAdd((() -> telemetry.addData("Auto", "No segments enabled")));
        }

        addedAction = false;
        return builder.build();
    }
}
