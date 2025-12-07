package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
@Config
@Autonomous(name = "Preload Far Auto", group = "Competition")
public class PreloadFarAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        MecanumDrive drive = Bot.drive;

        bot.setTargetGoalPose();

        Action blueFarAutoOnlyHpPreFar = drive.actionBuilderBlue(Pos.initialFarBluePose)

                .stopAndAdd(new SequentialAction(
                        new InstantAction(() -> bot.enableShooter(true)),
                        new SleepAction(0.7),
                        bot.shootThreeAutoFar()
                ))
                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                .stopAndAdd(new InstantAction(() -> bot.disableShooter()))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(80))
                .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 11.5, Pos.blueHpIntake.position.y - 2))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 5, Pos.blueHpIntake.position.y - 7), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 11.5, Pos.blueHpIntake.position.y), Math.toRadians(170))
                .afterTime(0.1, bot.enableShooter())
                .strafeToLinearHeading(Pos.initialFarBluePose.component1(), Pos.initialFarBluePose.component2())
                .stopAndAdd(bot.shootThreeAutoFar())
                .build();


        Action redFarAutoOnlyHpPreFar = drive.actionBuilderRed(Pos.initialFarBluePose)

                .afterTime(0.1, bot.enableShooter())
                .strafeToConstantHeading(Pos.closeShoot)
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                .stopAndAdd(new InstantAction(() -> bot.disableShooter()))
                .setTangent(Math.toRadians(160))
                .splineTo(Pos.blueHpIntakeInter, Math.toRadians(180))
                .setTangent(Math.toRadians(93))
                .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(80))
                .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 11.5, Pos.blueHpIntake.position.y - 2))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 5, Pos.blueHpIntake.position.y - 7), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(Pos.blueHpIntake.position.x - 11.5, Pos.blueHpIntake.position.y), Math.toRadians(170))
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .afterTime(0.1, bot.enableShooter())
                .splineToSplineHeading(new Pose2d(Pos.closeFirstShoot, Math.toRadians(90)), Math.toRadians(0))
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.position.x,Pos.blueFarIntake.position.y + 18))
                .setReversed(true)
                .splineTo(Pos.closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50, 70))
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
                .stopAndAdd(new InstantAction(() -> bot.disableShooter()))
                .build();


        bot.turret.trackObelisk();

        bot.enableFullAuto(true);
        bot.enableShooter(false);
        bot.setAllianceBlue();
        bot.setFar();
        bot.intake.closeGate();
        bot.intake.storage();
//        bot.setTargetFarAutoGoal();
        while (!isStarted()) {
            bot.periodic();
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.switchAlliance();
            }
            if (Bot.isBlue()) {
                drive.localizer.setPose(Pos.initialFarBluePose);
            } else {
                drive.localizer.setPose(Pos.initialFarRedPose);
            }

            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
            telemetry.addData("STARTING POSITION", Bot.getStartingPos());
//            telemetry.addData("DETECTED MOTIF", Turret.motif);
            telemetry.update();
        }
        if (Bot.isBlue()) {
            drive.localizer.setPose(Pos.initialFarBluePose);
            Actions.runBlocking(
                    new ActionHelper.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    blueFarAutoOnlyHpPreFar
                            )
                    )
            );
        } else {
            drive.localizer.setPose(Pos.initialFarRedPose);
            Actions.runBlocking(
                    new ActionHelper.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    redFarAutoOnlyHpPreFar
                            )
                    )
            );
        }



    }



}

