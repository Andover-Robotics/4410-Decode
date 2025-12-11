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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
@Config
@Autonomous(name = "Adaptive Far Auto", group = "Competition")
public class AdaptiveFarAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // ---------------- CONFIG STRUCT ----------------
    public static class AutoConfig {
        // which segments to run
        public boolean runPreload = true;
        public boolean runHp      = true;
        public boolean runClose   = true;
        public boolean runMid     = true;
        public boolean runFar     = true;
        public boolean runCycles  = false;

        // delay before starting preload actions (seconds, 0-20)
        public int startDelay = 0;

        // whether to drive to the field gate after the close intake
        public boolean pushFieldGate = true;
        // delay after pushing the field gate (seconds, 0-20)
        public int delayAfterGate = 0;

        // delay after each segment (seconds, 0-20)
        public int delayAfterPreload = 0;
        public int delayAfterHp      = 0;
        public int delayAfterClose   = 0;
        public int delayAfterMid     = 0;
        public int delayAfterFar     = 0;
        public int delayAfterCycles  = 0;
    }

    private AutoConfig cfg = new AutoConfig();

    // index of which segment driver is editing in init:
    // 0 = start delay, 1 = preload, 2 = hp, 3 = close, 4 = field gate push, 5 = mid, 6 = far
    private int selectedSegment = 0;

    // built auto we will run after start
    private Action builtAuto = null;
    private TrajectoryActionBuilder builder;
    private final ElapsedTime configEditTimer = new ElapsedTime();
    private boolean configDirty = false;
    private static final double AUTO_BUILD_DELAY_SECONDS = 8.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);
        configEditTimer.reset();

        MecanumDrive drive = Bot.drive;

        bot.enableFullAuto(true);
        bot.enableShooter(false);
        bot.setAllianceBlue();
        bot.setFar();
        bot.intake.closeGate();
        bot.intake.storage();
        bot.setTargetGoalPose();
        Bot.drive.localizer.recalibrateIMU();

        // ------------- INIT LOOP: CONFIGURE AUTO -------------
        while (!isStarted() && !isStopRequested()) {
            handleConfigInput();

//            if (configDirty && configEditTimer.seconds() >= AUTO_BUILD_DELAY_SECONDS) {
//                builtAuto = buildFarAuto(Bot.drive, Bot.isBlue(), cfg);
//                configDirty = false;
//            }

            // keep pose synced to chosen alliance
            if (Bot.isBlue()) {
                drive.localizer.setPose(Pos.initialFarBluePose);
            } else {
                drive.localizer.setPose(Pos.initialFarRedPose);
            }

            // Telemetry for configuration
            telemetry.addData("ALLIANCE (A)", Bot.getAlliance()); 
            telemetry.addData("STARTING POSITION", Bot.getStartingPos());
            telemetry.addData("Selected segment (UP/DOWN)", segmentName(selectedSegment));
            telemetry.addData("Start: delay (L/R)", "%ds", cfg.startDelay);
            telemetry.addData("Preload: run (X) / delay (L/R)", "%b / %ds",
                    cfg.runPreload, cfg.delayAfterPreload);
            telemetry.addData("HP:      run (X) / delay (L/R)", "%b / %ds",
                    cfg.runHp, cfg.delayAfterHp);
            telemetry.addData("Close:   run (X) / delay (L/R)", "%b / %ds",
                    cfg.runClose, cfg.delayAfterClose);
            telemetry.addData("Push gate: run (X) / delay (L/R)", "%b / %ds",
                    cfg.pushFieldGate, cfg.delayAfterGate);
            telemetry.addData("Mid:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runMid, cfg.delayAfterMid);
            telemetry.addData("Far:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runFar, cfg.delayAfterFar);
            telemetry.addData("Cycles:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runCycles, cfg.delayAfterCycles);
            telemetry.addData("Built? (Y to build)", builtAuto != null);
            if (configDirty) {
                telemetry.addData("BUILD NOT UPDATED", "Changes pending build");
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
            builtAuto = buildFarAuto(Bot.drive, Bot.isBlue(), cfg);
            configDirty = false;
        }

        telemetry.addData("Auto", "Built for %s", Bot.getAlliance());
        telemetry.addData("Segments", "preload:%b hp:%b close:%b mid:%b far:%b",
                cfg.runPreload, cfg.runHp, cfg.runClose, cfg.runMid, cfg.runFar, cfg.runCycles);
        telemetry.update();

        // Set starting pose again at start
        if (Bot.isBlue()) {
            drive.localizer.setPose(Pos.initialFarBluePose);
        } else {
            drive.localizer.setPose(Pos.initialFarRedPose);
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
            markConfigEdited();
        }

        // Move selected segment up/down
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            selectedSegment = (selectedSegment + 7 - 1) % 7;
        }
        if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            selectedSegment = (selectedSegment + 1) % 7;
        }

        // Toggle run/skip for selected segment
        if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
            switch (selectedSegment) {
                case 0:
                    // start delay has no enable/disable toggle
                    break;
                case 1:
                    cfg.runPreload = !cfg.runPreload;
                    markConfigEdited();
                    break;
                case 2:
                    cfg.runHp = !cfg.runHp;
                    markConfigEdited();
                    break;
                case 3:
                    cfg.runClose = !cfg.runClose;
                    markConfigEdited();
                    break;
                case 4:
                    cfg.pushFieldGate = !cfg.pushFieldGate;
                    markConfigEdited();
                    break;
                case 5:
                    cfg.runMid = !cfg.runMid;
                    markConfigEdited();
                    break;
                case 6:
                    cfg.runFar = !cfg.runFar;
                    markConfigEdited();
                    break;
                case 7:
                    cfg.runCycles = !cfg.runCycles;
                    markConfigEdited();
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
                    cfg.startDelay = clampDelay(cfg.startDelay + delta);
                    markConfigEdited();
                    break;
                case 1:
                    cfg.delayAfterPreload =
                            clampDelay(cfg.delayAfterPreload + delta);
                    markConfigEdited();
                    break;
                case 2:
                    cfg.delayAfterHp =
                            clampDelay(cfg.delayAfterHp + delta);
                    markConfigEdited();
                    break;
                case 3:
                    cfg.delayAfterClose =
                            clampDelay(cfg.delayAfterClose + delta);
                    markConfigEdited();
                    break;
                case 4:
                    cfg.delayAfterGate = clampDelay(cfg.delayAfterGate + delta);
                    markConfigEdited();
                    break;
                case 5:
                    cfg.delayAfterMid =
                            clampDelay(cfg.delayAfterMid + delta);
                    markConfigEdited();
                    break;
                case 6:
                    cfg.delayAfterFar =
                            clampDelay(cfg.delayAfterFar + delta);
                    markConfigEdited();
                    break;
                case 7:
                    cfg.delayAfterCycles =
                            clampDelay(cfg.delayAfterCycles + delta);
                    markConfigEdited();
                    break;
            }
        }
//
        if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            TrajectoryActionBuilder tester = Bot.drive.actionBuilderBlue(Pos.initialFarBluePose)
                    .stopAndAdd(new SleepAction(6.7));
            tester = tester.stopAndAdd(bot.shootThreeAutoClose());
            builtAuto = tester.build();
            configDirty = false;
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            builtAuto = builder.build();
            configDirty = false;
        }

        // Y: build the auto with current config
        if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
            builtAuto = buildFarAuto(Bot.drive, Bot.isBlue(), cfg);
            configDirty = false;
        }
    }

    private void markConfigEdited() {
        configDirty = true;
        configEditTimer.reset();
        builtAuto = null;
    }

    private int clampDelay(int d) {
        if (d < 0) return 0;
        if (d > 20) return 20;
        return d;
    }

    private String segmentName(int idx) {
        switch (idx) {
            case 0: return "Start Delay";
            case 1: return "Preload";
            case 2: return "HP";
            case 3: return "Close";
            case 4: return "Push Gate";
            case 5: return "Mid";
            case 6: return "Far";
            case 7: return "Cycles";
            default: return "?";
        }
    }

    // ---------------- BUILDER: BUILD BLUE/RED FAR AUTO ----------------

    private Action buildFarAuto(MecanumDrive drive, boolean isBlue, AutoConfig cfg) {
        builder = isBlue
                ? drive.actionBuilderBlue(Pos.initialFarBluePose)
                : drive.actionBuilderRed(Pos.initialFarBluePose);

        boolean addedAction = false;

        // START DELAY
        if (cfg.startDelay > 0) {
            builder = builder.stopAndAdd(new SleepAction(cfg.startDelay));
            addedAction = true;
        }

        // PRELOAD SEGMENT
        if (cfg.runPreload) {
            builder = builder
                    .afterTime(0.1, bot.enableShooter())
                    .strafeToConstantHeading(Pos.closeShoot)
                    .stopAndAdd(bot.shootThreeAutoClose())
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .stopAndAdd(new InstantAction(() -> bot.disableShooter()));
            if (cfg.delayAfterPreload > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterPreload));
            }
            addedAction = true;
        }

        if (cfg.runHp) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .setTangent(Math.toRadians(160))
                    .splineTo(Pos.blueHpIntakeInter, Math.toRadians(180))
                    .setTangent(Math.toRadians(93))
                    .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(80))
                    .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 11.5, Pos.blueHpIntake.position.y - 2))

                    .setTangent(Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 5, Pos.blueHpIntake.position.y - 7), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 11.5, Pos.blueHpIntake.position.y), Math.toRadians(170))

                    .setReversed(true)
                    .setTangent(Math.toRadians(-90));
            if (cfg.runClose) {
                builder = builder
                        .afterTime(0.1, bot.enableShooter())
                        .splineToSplineHeading(new Pose2d(Pos.closeFirstShoot, Math.toRadians(90)), Math.toRadians(0))
                        .stopAndAdd(bot.shootThreeAutoClose());
            } else {
                builder = builder
                        .afterTime(0.1, bot.enableShooter())
                        .splineToSplineHeading(new Pose2d(Pos.closeShoot, Math.toRadians(90)), Math.toRadians(0))
                        .stopAndAdd(bot.shootThreeAutoClose());
            }

            if (cfg.delayAfterHp > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterHp));
            }
            addedAction = true;
        }

        if (cfg.runClose) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 65))
                    .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.position.x,Pos.blueCloseIntake.position.y + 18));

            if (cfg.pushFieldGate) {
                builder = builder
                        .strafeToLinearHeading(Pos.gate.position, Pos.gate.heading)
                        .waitSeconds(1);

                if (cfg.delayAfterGate > 0) {
                    builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterGate));
                }

            }

            builder = builder.stopAndAdd(bot.enableShooter())
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                    .stopAndAdd(bot.shootThreeAutoClose())
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .stopAndAdd(new InstantAction(() -> bot.disableShooter()));

            if (cfg.delayAfterClose > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterClose));
            }
            addedAction = true;
        }

        if (cfg.runMid) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .setTangent(Math.toRadians(135))
                    .splineToSplineHeading(Pos.blueMidIntake, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.position.x,Pos.blueMidIntake.position.y + 18))
                    .stopAndAdd(bot.enableShooter())
                    .setReversed(true)
                    .splineTo(Pos.closeShoot, Math.toRadians(-60))
                    .stopAndAdd(bot.shootThreeAutoClose())
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .stopAndAdd(new InstantAction(() -> bot.disableShooter()));

            if (cfg.delayAfterMid > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterMid));
            }
            addedAction = true;
        }

        if (cfg.runFar) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.position.x,Pos.blueFarIntake.position.y + 18))
                    .setReversed(true)
                    .splineTo(Pos.closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 70))
                    .stopAndAdd(bot.shootThreeAutoClose())
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .stopAndAdd(new InstantAction(() -> bot.disableShooter()));

            if (cfg.delayAfterFar > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterFar));
            }
            addedAction = true;
        }

        if (cfg.runCycles) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .setTangent(Math.toRadians(125))
                    .splineToSplineHeading(Pos.blueSecretTunnel, Math.toRadians(179))
                    .setTangent(Math.toRadians(180))
                    .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(180))
                    .setTangent(Math.toRadians(179))
                    .splineToConstantHeading(Pos.closeShoot, Math.toRadians(-10))
                    .stopAndAdd(bot.shootThreeAutoClose())
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))

                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .setTangent(Math.toRadians(125))
                    .splineToSplineHeading(Pos.blueSecretTunnel, Math.toRadians(179))
                    .setTangent(Math.toRadians(179))
                    .splineToConstantHeading(Pos.closeShoot, Math.toRadians(-10))
                    .stopAndAdd(bot.shootThreeAutoClose())
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()));

            if (cfg.delayAfterCycles > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterCycles));
            }
            addedAction = true;
        }

        if (!addedAction) {
            builder = builder.stopAndAdd(new InstantAction(() -> telemetry.addData("Auto", "No segments enabled")));
        }
        return builder.build();
    }

}
