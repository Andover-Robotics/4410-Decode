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
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

@Config
@Autonomous(name = "Adaptive Close Auto", group = "Competition")
public class AdaptiveCloseAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // ---------------- CONFIG STRUCT ----------------
    public static class AutoConfig {
        public boolean runPreload = true;
        public boolean runGate    = true;
        public boolean runClose   = true;
        public boolean runMid     = true;
        public boolean runFar     = true;
        public boolean runHp      = true;

        public int startDelay = 0;

        public int delayAfterPreload = 0;
        public int delayAfterGate    = 0;
        public int delayAfterClose   = 0;
        public int delayAfterMid     = 0;
        public int delayAfterFar     = 0;
        public int delayAfterHp      = 0;
    }

    private AutoConfig cfg = new AutoConfig();

    // 0 = start delay, 1 = preload, 2 = gate, 3 = close, 4 = mid, 5 = far, 6 = hp
    private int selectedSegment = 0;

    private Action builtAuto = null;
    private TrajectoryActionBuilder builder;

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
        bot.setClose();
        bot.intake.closeGate();
        bot.intake.storage();

        // ------------- INIT LOOP: CONFIGURE AUTO -------------
        while (opModeInInit() && !isStopRequested() && !isStarted()) {
            handleConfigInput();

            if (Bot.isBlue()) {
                drive.localizer.setPose(Pos.initialCloseBluePose);
            } else {
                drive.localizer.setPose(Pos.initialCloseRedPose);
            }

            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
            telemetry.addData("STARTING POSITION", Bot.getStartingPos());
            telemetry.addData("DETECTED MOTIF", Turret.motif);
            telemetry.addData("Selected segment (UP/DOWN)", segmentName(selectedSegment));
            telemetry.addData("Start: delay (L/R)", "%ds", cfg.startDelay);
            telemetry.addData("Preload: run (X) / delay (L/R)", "%b / %ds",
                    cfg.runPreload, cfg.delayAfterPreload);
            telemetry.addData("Gate:    run (X) / delay (L/R)", "%b / %ds",
                    cfg.runGate, cfg.delayAfterGate);
            telemetry.addData("Close:   run (X) / delay (L/R)", "%b / %ds",
                    cfg.runClose, cfg.delayAfterClose);
            telemetry.addData("Mid:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runMid, cfg.delayAfterMid);
            telemetry.addData("Far:     run (X) / delay (L/R)", "%b / %ds",
                    cfg.runFar, cfg.delayAfterFar);
            telemetry.addData("HP:      run (X) / delay (L/R)", "%b / %ds",
                    cfg.runHp, cfg.delayAfterHp);
            telemetry.addData("Built? (Y to build)", builtAuto != null);
            if (builtAuto != null) {
                telemetry.addData("build", builtAuto);
            }
            telemetry.update();

            bot.periodic();
        }

        waitForStart();
        if (isStopRequested()) return;
        if (builtAuto == null) {
            builtAuto = buildCloseAuto(Bot.drive, Bot.isBlue(), cfg);
        }

        telemetry.addData("Auto", "Built for %s", Bot.getAlliance());
        telemetry.addData("Segments", "preload:%b gate:%b close:%b mid:%b far:%b hp:%b",
                cfg.runPreload, cfg.runGate, cfg.runClose, cfg.runMid, cfg.runFar, cfg.runHp);
        telemetry.update();

        if (Bot.isBlue()) {
            drive.localizer.setPose(Pos.initialCloseBluePose);
        } else {
            drive.localizer.setPose(Pos.initialCloseRedPose);
        }

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

        if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
            bot.switchAlliance();
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
                    break;
                case 1:
                    cfg.runPreload = !cfg.runPreload;
                    break;
                case 2:
                    cfg.runGate = !cfg.runGate;
                    break;
                case 3:
                    cfg.runClose = !cfg.runClose;
                    break;
                case 4:
                    cfg.runMid = !cfg.runMid;
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
                    cfg.startDelay = clampDelay(cfg.startDelay + delta);
                    break;
                case 1:
                    cfg.delayAfterPreload =
                            clampDelay(cfg.delayAfterPreload + delta);
                    break;
                case 2:
                    cfg.delayAfterGate =
                            clampDelay(cfg.delayAfterGate + delta);
                    break;
                case 3:
                    cfg.delayAfterClose =
                            clampDelay(cfg.delayAfterClose + delta);
                    break;
                case 4:
                    cfg.delayAfterMid =
                            clampDelay(cfg.delayAfterMid + delta);
                    break;
                case 5:
                    cfg.delayAfterFar =
                            clampDelay(cfg.delayAfterFar + delta);
                    break;
                case 6:
                    cfg.delayAfterHp =
                            clampDelay(cfg.delayAfterHp + delta);
                    break;
            }
        }

        if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
            builtAuto = buildCloseAuto(Bot.drive, Bot.isBlue(), cfg);
        }
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
            case 2: return "Gate";
            case 3: return "Close";
            case 4: return "Mid";
            case 5: return "Far";
            case 6: return "HP";
            default: return "?";
        }
    }

    // ---------------- BUILDER: BUILD BLUE/RED CLOSE AUTO ----------------

    private Action buildCloseAuto(MecanumDrive drive, boolean isBlue, AutoConfig cfg) {
        builder = isBlue
                ? drive.actionBuilderBlue(Pos.initialCloseBluePose)
                : drive.actionBuilderRed(Pos.initialCloseBluePose);

        boolean addedAction = false;

        if (cfg.startDelay > 0) {
            builder = builder.stopAndAdd(new SleepAction(cfg.startDelay));
            addedAction = true;
        }

        if (cfg.runPreload) {
            builder = builder
                    .stopAndAdd(bot.enableShooter())
                    .strafeToLinearHeading(Pos.closeFirstShoot, Math.toRadians(90))
                    .stopAndAdd(bot.shootThreeAutoClose());

            if (cfg.delayAfterPreload > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterPreload));
            }
            addedAction = true;
        }

        if (cfg.runClose) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90),
                            drive.defaultVelConstraint, new ProfileAccelConstraint(-45, 65))
                    .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.position.x,
                            Pos.blueCloseIntake.position.y + 18));
            addedAction = true;
        }

        if (cfg.runGate) {
            builder = builder
                    .strafeToLinearHeading(Pos.gate.position, Pos.gate.heading)
                    .waitSeconds(1.6);

            if (cfg.delayAfterGate > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterGate));
            }
            addedAction = true;
        }

        if (cfg.runClose || cfg.runGate) {
            builder = builder.stopAndAdd(bot.enableShooter())
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                    .stopAndAdd(bot.shootThreeAutoClose());
        }

        if (cfg.delayAfterClose > 0 && cfg.runClose) {
            builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterClose));
        }

        if (cfg.runMid) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .setTangent(Math.toRadians(135))
                    .splineTo(Pos.blueMidIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.position.x,
                            Pos.blueMidIntake.position.y + 18))
                    .stopAndAdd(bot.enableShooter())
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                    .stopAndAdd(bot.shootThreeAutoClose());

            if (cfg.delayAfterMid > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterMid));
            }
            addedAction = true;
        }

        if (cfg.runFar) {
            builder = builder

                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                    .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.position.x,
                            Pos.blueFarIntake.position.y + 18))
                    .setReversed(true)
                    .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(155))
                    .stopAndAdd(bot.shootThreeAutoClose());

            if (cfg.delayAfterFar > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterFar));
            }
            addedAction = true;
        }

        if (cfg.runHp) {
            builder = builder
                    .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                    .stopAndAdd(bot.disableShooter())
                    .setTangent(Math.toRadians(155))
                    .splineToSplineHeading(Pos.blueStraightHpIntake, Math.toRadians(135))
//                    .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(90))
                    .stopAndAdd(new SleepAction(0.05))

                    .strafeToConstantHeading(new Vector2d(Pos.blueStraightHpIntake.component1().x - 14, Pos.blueStraightHpIntake.component1().y))
                    .stopAndAdd(new SleepAction(0.05))
                    .setTangent(0)
                    .splineToSplineHeading(new Pose2d(Pos.blueStraightHpIntake.component1().x - 14, Pos.blueStraightHpIntake.component1().y - 5, Math.toRadians(-135)), Math.toRadians(-90))
                    .splineTo(new Vector2d(Pos.blueHpIntakeInter.x, Pos.blueHpIntakeInter.y + 3), Math.toRadians(-90))

//                    .setTangent(Math.toRadians(-90))
                    .afterTime(0.1, bot.enableShooter())
                    .splineToLinearHeading(new Pose2d(Pos.closeShoot, Math.toRadians(-90)), Math.toRadians(0))
                    .stopAndAdd(bot.shootThreeAutoClose());

            if (cfg.delayAfterHp > 0) {
                builder = builder.stopAndAdd(new SleepAction(cfg.delayAfterHp));
            }
            addedAction = true;
        }

        if (!addedAction) {
            builder = builder.stopAndAdd(new InstantAction(() -> telemetry.addData("Auto", "No segments enabled")));
        }
        return builder.build();
    }
}
