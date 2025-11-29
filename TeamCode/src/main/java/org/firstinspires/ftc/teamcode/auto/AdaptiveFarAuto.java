package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
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

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
@Config
@Autonomous(name = "Adaptive Far Auto", group = "Competition")
public class AdaptiveFarAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // Coordinates ->
    // positive X is towards goal side, negative X is towards HP Stations
    // positive Y is towards blue goal / red driver side, negative Y is towards red goal / blue driver side
    // 0 degrees is toward goals, -45 is red goal, +45 is blue goal

    // SOME VARIABLES
    public static final double WALL_INTAKE_ANGLE = Math.toRadians(170);

    // INITIAL
    public static Pose2d initialFarBluePose  = new Pose2d(-63,  9,  Math.toRadians(90));
    public static Pose2d initialCloseBluePose = new Pose2d(63, 12, Math.toRadians(-135));
    public static Pose2d initialFarRedPose   = new Pose2d(-63, -9, Math.toRadians(-90));
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE
    public static Pose2d blueHpIntake    = new Pose2d(-49, 61, WALL_INTAKE_ANGLE);
    public static Pose2d blueFarIntake   = new Pose2d(-33, 29, Math.toRadians(90));
    public static Pose2d blueMidIntake   = new Pose2d(-11, 29, Math.toRadians(90));
    public static Pose2d blueCloseIntake = new Pose2d(13,  29, Math.toRadians(90));

    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d closeShoot      = new Vector2d(5,  15);
    public static Vector2d closeFirstShoot = new Vector2d(13, 15);
    public static Vector2d farShoot        = new Vector2d(-61, 9);

    public static Pose2d gate = new Pose2d(7, 57, Math.toRadians(0));

    // ---------------- CONFIG STRUCT ----------------
    public static class AutoConfig {
        // which segments to run
        public boolean runPreload = true;
        public boolean runHp      = true;
        public boolean runClose   = true;
        public boolean runMid     = true;
        public boolean runFar     = true;

        // delay after each segment (seconds, 0-20)
        public int delayAfterPreload = 0;
        public int delayAfterHp      = 0;
        public int delayAfterClose   = 0;
        public int delayAfterMid     = 0;
        public int delayAfterFar     = 0;
    }

    private AutoConfig cfg = new AutoConfig();

    // index of which segment driver is editing in init:
    // 0 = preload, 1 = hp, 2 = close, 3 = mid, 4 = far
    private int selectedSegment = 0;

    // built auto we will run after start
    private Action builtAuto = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        MecanumDrive drive = Bot.drive;

        bot.turret.trackObelisk();
        bot.enableFullAuto(true);
        bot.enableShooter(false);
        bot.setAllianceBlue();
        bot.setFar();
        bot.intake.closeGate();
        bot.intake.storage();

        // ------------- INIT LOOP: CONFIGURE AUTO -------------
        while (opModeInInit() && !isStopRequested()) {
            handleConfigInput();

            // keep pose synced to chosen alliance
            if (Bot.isBlue()) {
                drive.localizer.setPose(initialFarBluePose);
            } else {
                drive.localizer.setPose(initialFarRedPose);
            }

            // Telemetry for configuration
            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
            telemetry.addData("Selected segment", segmentName(selectedSegment));
            telemetry.addData("Preload: run / delay", "%b / %ds",
                    cfg.runPreload, cfg.delayAfterPreload);
            telemetry.addData("HP:      run / delay", "%b / %ds",
                    cfg.runHp, cfg.delayAfterHp);
            telemetry.addData("Close:   run / delay", "%b / %ds",
                    cfg.runClose, cfg.delayAfterClose);
            telemetry.addData("Mid:     run / delay", "%b / %ds",
                    cfg.runMid, cfg.delayAfterMid);
            telemetry.addData("Far:     run / delay", "%b / %ds",
                    cfg.runFar, cfg.delayAfterFar);
            telemetry.addData("Built? (Y to build)", builtAuto != null);
            telemetry.update();

            bot.periodic();
        }

        waitForStart();
        if (isStopRequested()) return;
        // Safety: if driver did not build in init, build now with current config
        if (builtAuto == null) {
            builtAuto = buildFarAuto(drive, Bot.isBlue(), cfg);
        }

        telemetry.addData("Auto", "Built for %s", Bot.getAlliance());
        telemetry.addData("Segments", "preload:%b hp:%b close:%b mid:%b far:%b",
                cfg.runPreload, cfg.runHp, cfg.runClose, cfg.runMid, cfg.runFar);
        telemetry.update();

        // Set starting pose again at start
        if (Bot.isBlue()) {
            drive.localizer.setPose(initialFarBluePose);
        } else {
            drive.localizer.setPose(initialFarRedPose);
        }

        // ------------- RUN AUTO -------------
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

    // ---------------- CONFIG INPUT HANDLING ----------------

    private void handleConfigInput() {
        gp1.readButtons();

        // Alliance toggle (you already had A for this)
        if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
            bot.switchAlliance();
        }

        // Move selected segment up/down
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            selectedSegment = (selectedSegment + 5 - 1) % 5;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            selectedSegment = (selectedSegment + 1) % 5;
        }

        // Toggle run/skip for selected segment
        if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
            switch (selectedSegment) {
                case 0:
                    cfg.runPreload = !cfg.runPreload;
                    break;
                case 1:
                    cfg.runHp = !cfg.runHp;
                    break;
                case 2:
                    cfg.runClose = !cfg.runClose;
                    break;
                case 3:
                    cfg.runMid = !cfg.runMid;
                    break;
                case 4:
                    cfg.runFar = !cfg.runFar;
                    break;
            }
        }

        // Adjust delay with left/right
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
                    cfg.delayAfterPreload =
                            clampDelay(cfg.delayAfterPreload + delta);
                    break;
                case 1:
                    cfg.delayAfterHp =
                            clampDelay(cfg.delayAfterHp + delta);
                    break;
                case 2:
                    cfg.delayAfterClose =
                            clampDelay(cfg.delayAfterClose + delta);
                    break;
                case 3:
                    cfg.delayAfterMid =
                            clampDelay(cfg.delayAfterMid + delta);
                    break;
                case 4:
                    cfg.delayAfterFar =
                            clampDelay(cfg.delayAfterFar + delta);
                    break;
            }
        }

        // Y: build the auto with current config
        if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
            /*builtAuto = */buildFarAuto(Bot.drive, Bot.isBlue(), cfg);
            }
    }

    private int clampDelay(int d) {
        if (d < 0) return 0;
        if (d > 20) return 20;
        return d;
    }

    private String segmentName(int idx) {
        switch (idx) {
            case 0: return "Preload";
            case 1: return "HP";
            case 2: return "Close";
            case 3: return "Mid";
            case 4: return "Far";
            default: return "?";
        }
    }

    // ---------------- BUILDER: BUILD BLUE/RED FAR AUTO ----------------

    private Action buildFarAuto(MecanumDrive drive, boolean isBlue, AutoConfig cfg) {
        Pose2d startPose = isBlue ? initialFarBluePose : initialFarRedPose;

        // Assuming your MecanumDrive has these methods:
        TrajectoryActionBuilder builder = isBlue
                ? drive.actionBuilderBlue(startPose)
                : drive.actionBuilderRed(startPose);
//        TrajectoryActionBuilder builder = drive.actionBuilderBlue(startPose);

        boolean addedAction = false;

        builder.stopAndAdd(bot.shootThree());

        // PRELOAD SEGMENT
        if (cfg.runPreload) {
            builder.stopAndAdd(
                    new SequentialAction(
                            bot.enableShooter(),
                            new SleepAction(0.4),
                            bot.shootThree(),
                            bot.disableShooter()
                    )
            );
            if (cfg.delayAfterPreload > 0) {
                builder.stopAndAdd(new SleepAction(cfg.delayAfterPreload));
            }
            addedAction = true;
        }

        // HP SEGMENT: go to HP intake, collect, go to closeFirstShoot and shoot 3
        if (cfg.runHp) {
            builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .strafeToSplineHeading(
                            new Vector2d(
                                    blueHpIntake.position.x,
                                    blueHpIntake.position.y
                            ),
                            Math.toRadians(150)
                    )
                    .strafeToConstantHeading(
                            new Vector2d(
                                    blueHpIntake.position.x - 11.5,
                                    blueHpIntake.position.y
                            )
                    )
                    .setReversed(true)
                    .setTangent(Math.toRadians(-90))
                    .afterTime(0.1, bot.enableShooter())
                    .splineToSplineHeading(
                            new Pose2d(closeFirstShoot, Math.toRadians(90)),
                            Math.toRadians(0)
                    )
                    .stopAndAdd(new InstantAction(() -> bot.intake.storage()))
                    .stopAndAdd(bot.shootThree());

            if (cfg.delayAfterHp > 0) {
                builder.stopAndAdd(new SleepAction(cfg.delayAfterHp));
            }
            addedAction = true;
        }

        // CLOSE SEGMENT: close intake -> gate -> shoot at closeShoot
        if (cfg.runClose) {
            builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .splineTo(
                            blueCloseIntake.position,
                            Math.toRadians(90),
                            drive.defaultVelConstraint,
                            new ProfileAccelConstraint(-45, 65)
                    )
                    .strafeToConstantHeading(
                            new Vector2d(
                                    blueCloseIntake.position.x,
                                    blueCloseIntake.position.y + 18
                            )
                    )
                    .stopAndAdd(new InstantAction(() -> bot.intake.storage()))
                    .strafeToLinearHeading(gate.position, gate.heading)
                    .waitSeconds(1)
                    .stopAndAdd(bot.enableShooter())
                    .setReversed(true)
                    .strafeToSplineHeading(closeShoot, Math.toRadians(135))
                    .stopAndAdd(bot.shootThree());

            if (cfg.delayAfterClose > 0) {
                builder.stopAndAdd(new SleepAction(cfg.delayAfterClose));
            }
            addedAction = true;
        }

        // MID SEGMENT
        if (cfg.runMid) {
            builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .setTangent(Math.toRadians(135))
                    .splineTo(blueMidIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(
                            new Vector2d(
                                    blueMidIntake.position.x,
                                    blueMidIntake.position.y + 18
                            )
                    )
                    .stopAndAdd(new InstantAction(() -> bot.intake.storage()))
                    .stopAndAdd(bot.enableShooter())
                    .setReversed(true)
                    .splineTo(closeShoot, Math.toRadians(-60))
                    .stopAndAdd(bot.shootThree());

            if (cfg.delayAfterMid > 0) {
                builder.stopAndAdd(new SleepAction(cfg.delayAfterMid));
            }
            addedAction = true;
        }

        // FAR SEGMENT
        if (cfg.runFar) {
            builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .splineTo(blueFarIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(
                            new Vector2d(
                                    blueFarIntake.position.x,
                                    blueFarIntake.position.y + 18
                            )
                    )
                    .stopAndAdd(new InstantAction(() -> bot.intake.storage()))
                    .setReversed(true)
                    .splineTo(
                            closeShoot,
                            Math.toRadians(-45),
                            drive.defaultVelConstraint,
                            new ProfileAccelConstraint(-50, 70)
                    )
                    .stopAndAdd(bot.shootThree());

            if (cfg.delayAfterFar > 0) {
                builder.stopAndAdd(new SleepAction(cfg.delayAfterFar));
            }
            addedAction = true;
        }

        if (!addedAction) {
            builder.stopAndAdd(new InstantAction(() -> telemetry.addData("Auto", "No segments enabled")));
        }

        builtAuto = builder.build();

        return builder.build();
    }

    // ---------------- HELPER: RED TRANSFORM ----------------

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(
                new Vector2d(-pose.position.x, pose.position.y),
                Math.PI - pose.heading.log()
        );
    }
}
