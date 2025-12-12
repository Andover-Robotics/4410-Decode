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
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

@Config
@Autonomous(name = "Old Close Auto", group = "Old")
public class OldCloseAuto extends LinearOpMode {
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

        //go to shooting position and shoot
        //Intake first three and then shoot
        //open gate and intake middle three
        //shoot middle three
        //intake and shoot far three
        //intake and shoot hp balls

        Action blueCloseAuto = drive.actionBuilderBlue(Pos.initialCloseBluePose)
                .stopAndAdd(bot.enableShooter())
                .strafeToLinearHeading(Pos.closeFirstShoot, Math.toRadians(90))
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))

                //copied from far
                .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45,65))
                .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.component1().x, Pos.blueCloseIntake.component1().y + 18))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .strafeToLinearHeading(Pos.gate.position, Pos.gate.heading)
                .waitSeconds(1)

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .stopAndAdd(bot.disableShooter())

                .splineTo(Pos.blueMidIntakeFar.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueMidIntakeFar.component1().x, Pos.blueMidIntakeFar.component1().y + 18))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .stopAndAdd(bot.disableShooter())

                .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.component1().x, Pos.blueFarIntake.component1().y + 18))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(155))
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .stopAndAdd(bot.disableShooter())

                .setTangent(Math.toRadians(155))
                .splineTo(Pos.blueHpIntake.component1(), Math.toRadians(135))
//                .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(90))

                .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.component1().x - 14, Pos.blueHpIntake.component1().y - 1))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(Pos.blueHpIntake.component1().x - 14, Pos.blueHpIntake.component1().y - 5), Math.toRadians(180))

                .setTangent(Math.toRadians(-90))
                .afterTime(0.1, bot.enableShooter())
                .splineToLinearHeading(new Pose2d(Pos.blueHpIntakeInter, Math.toRadians(180)), Math.toRadians(-89))
                .splineToLinearHeading(new Pose2d(Pos.closeShoot, Math.toRadians(-90)), Math.toRadians(0))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .stopAndAdd(bot.shootThreeAutoClose())

                .build();

        Action redCloseAuto = drive.actionBuilderRed(Pos.initialCloseBluePose)//switched on purpose - DO NOT CHANGE
                .stopAndAdd(new SequentialAction(
                                bot.enableShooter(),
                                new SleepAction(0.5),
                                bot.shootThreeAutoClose(),
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
//                .stopAndAdd(bot.shootThreeAutoClose())
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
//                .stopAndAdd(bot.shootThreeAutoClose())
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
//                .stopAndAdd(bot.shootThreeAutoClose())
//
//                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
//                .splineTo(blueFarIntake.position, Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(blueFarIntake.component1().x, blueFarIntake.component1().y + 18))
//                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
//                .setReversed(true)
//                .splineTo(closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50,70))
//                .stopAndAdd(bot.shootThreeAutoClose())

                .build();

        bot.turret.trackObelisk();

        bot.enableFullAuto(true);
        bot.enableShooter(false);
        bot.setAllianceBlue();
        bot.setClose();
        bot.intake.closeGate();
        bot.intake.storage();
        while (!isStarted()) {
            bot.periodic();
            gp1.readButtons();

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.switchAlliance();
            }
            if (Bot.isBlue()) {
                drive.localizer.setPose(Pos.initialCloseBluePose);
            } else {
                drive.localizer.setPose(Pos.initialCloseRedPose);
            }

            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
            telemetry.addData("STARTING POSITION", Bot.getStartingPos());
            telemetry.addData("DETECTED MOTIF", Turret.motif);
            telemetry.update();
        }
        if (Bot.isBlue()) {
            drive.localizer.setPose(Pos.initialCloseBluePose);
            Actions.runBlocking(
                    new ActionHelper.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    blueCloseAuto
                            )
                    )
            );
        } else {
            drive.localizer.setPose(Pos.initialCloseRedPose);
            Actions.runBlocking(
                    new ActionHelper.RaceParallelCommand(
                            bot.actionPeriodic(),
                            new SequentialAction(
                                    redCloseAuto
                            )
                    )
            );
        }



    }

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(-pose.position.x, pose.position.y), Math.PI - pose.heading.log());
    }
}
