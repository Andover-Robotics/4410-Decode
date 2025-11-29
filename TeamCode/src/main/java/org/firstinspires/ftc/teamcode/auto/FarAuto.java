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
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;
import org.firstinspires.ftc.teamcode.auto.Pos;

@Config
@Autonomous(name = "Far Auto", group = "Competition")
public class FarAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    //Coordinates ->
    // positive X is towards goal side, negative x is towards HP Stations
    // positive Y is towards blue goal / red driver side, negative Y is towards red goal / blue driver side
    // 0 degrees is toward goals, -45 is red goal, +45 is blue goal

    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        MecanumDrive drive = Bot.drive;

        Action blueFarAuto = drive.actionBuilderBlue(Pos.initialFarBluePose)
//                .stopAndAdd(new SequentialAction(
//                        bot.enableShooter(),
//                        new InstantAction(() -> bot.setTargetFarAutoGoal()),
//                        new SleepAction(0.9),
//                        bot.shootThreeAutoFar(),
//                        new SleepAction(0.1),
//                        new InstantAction(() -> bot.setTargetGoalPose()),
//                        bot.disableShooter()
//                        )
//                )

//                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .afterTime(0.2, bot.enableShooter())
                .strafeToConstantHeading(Pos.edgeShoot)
                .stopAndAdd(bot.shootThree())
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .stopAndAdd(new InstantAction(()-> bot.disableShooter()))

                .setTangent(Math.toRadians(160))
                .splineTo((Pos.blueHpIntakeInter), Math.toRadians(180))
                .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(80))
                .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.component1().x - 11.5, Pos.blueHpIntake.component1().y))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                // .setTangent(Math.toRadians(90))
                //.strafeToSplineHeading(new Vector2d(blueHpIntake.component1().x, blueHpIntake.component1().y), Math.toRadians(150))
                //.strafeToConstantHeading(new Vector2d(blueHpIntake.component1().x - 11.5, blueHpIntake.component1().y))

                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .afterTime(0.1, bot.enableShooter())
                .splineToSplineHeading(new Pose2d(Pos.closeFirstShoot, Math.toRadians(90)), Math.toRadians(0)) //might be +150? idk will have to test
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .stopAndAdd(bot.shootThree())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45,65))
                .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.component1().x, Pos.blueCloseIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .strafeToLinearHeading(Pos.gate.position, Pos.gate.heading)
                .waitSeconds(1)

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
//                .setTangent(Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(closeShoot, Math.toRadians(90)), Math.toRadians(-90))
                .stopAndAdd(bot.shootThree())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .setTangent(Math.toRadians(135))
                .splineTo(Pos.blueMidIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.component1().x, Pos.blueMidIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .splineTo(Pos.closeShoot, Math.toRadians(-60))
                .stopAndAdd(bot.shootThree())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.component1().x, Pos.blueFarIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .setReversed(true)
                .splineTo(Pos.closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50,70))
                .stopAndAdd(bot.shootThree())
                .build();

        Action redFarAuto = drive.actionBuilderRed(Pos.initialFarBluePose)//switched on purpose - DO NOT CHANGE
                .stopAndAdd(new SequentialAction(
                                bot.enableShooter(),
                                new SleepAction(0.5),
                                bot.shootThree(),
                                bot.disableShooter()
                        )
                )
//
//                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
//
//                // .setTangent(Math.toRadians(90))
//                .strafeToSplineHeading(new Vector2d(blueHpIntake.component1().x, blueHpIntake.component1().y), Math.toRadians(150))
//                //.splineToSplineHeading(blueHpIntake, Math.toRadians(150))
//                .strafeToConstantHeading(new Vector2d(blueHpIntake.component1().x - 11.5, blueHpIntake.component1().y))
//
//                .setReversed(true)
//                .setTangent(Math.toRadians(-90))
//                .afterTime(0.1, bot.enableShooter())
//                .splineToSplineHeading(new Pose2d(closeFirstShoot, Math.toRadians(90)), Math.toRadians(0)) //might be +150? idk will have to test
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
//                .stopAndAdd(bot.shootThree())
//
//                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
//                .splineTo(blueCloseIntake.position, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45,65))
//                .strafeToConstantHeading(new Vector2d(blueCloseIntake.component1().x, blueCloseIntake.component1().y + 18))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
//                .strafeToLinearHeading(gate.position, gate.heading)
//                .waitSeconds(1)
//
//                .stopAndAdd(bot.enableShooter())
//                .setReversed(true)
//                .strafeToSplineHeading(closeShoot, Math.toRadians(135))
////                .setTangent(Math.toRadians(-90))
////                .splineToSplineHeading(new Pose2d(closeShoot, Math.toRadians(90)), Math.toRadians(-90))
//                .stopAndAdd(bot.shootThree())
//
//                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
//                .setTangent(Math.toRadians(135))
//                .splineTo(blueMidIntake.position, Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(blueMidIntake.component1().x, blueMidIntake.component1().y + 18))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
//
//                .stopAndAdd(bot.enableShooter())
//                .setReversed(true)
//                .splineTo(closeShoot, Math.toRadians(-60))
//                .stopAndAdd(bot.shootThree())
//
//                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
//                .splineTo(blueFarIntake.position, Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(blueFarIntake.component1().x, blueFarIntake.component1().y + 18))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
//                .setReversed(true)
//                .splineTo(closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50,70))
//                .stopAndAdd(bot.shootThree())
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
            telemetry.addData("DETECTED MOTIF", Turret.motif);
            telemetry.update();
        }
        if (Bot.isBlue()) {
            drive.localizer.setPose(Pos.initialFarBluePose);
            Actions.runBlocking(
                    new ActionHelper.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    blueFarAuto
                            )
                    )
            );
        } else {
            drive.localizer.setPose(Pos.initialFarRedPose);
            Actions.runBlocking(
                    new ActionHelper.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    redFarAuto
                            )
                    )
            );
        }



    }


    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(-pose.position.x, pose.position.y), Math.PI - pose.heading.log());
    }
}
