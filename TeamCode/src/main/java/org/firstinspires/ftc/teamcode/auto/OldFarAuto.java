package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "Old Far Auto", group = "Old")
public class OldFarAuto extends LinearOpMode {
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

                .afterTime(0.2, bot.enableShooter())
                .strafeToConstantHeading(Pos.closeShoot)
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .stopAndAdd(new InstantAction(()-> bot.disableShooter()))

                .setTangent(Math.toRadians(160))
                .splineTo((Pos.blueHpIntakeInter), Math.toRadians(180))
                .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(80))
                .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.component1().x - 11.5, Pos.blueHpIntake.component1().y))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .afterTime(0.1, bot.enableShooter())
                .splineToSplineHeading(new Pose2d(Pos.closeFirstShoot, Math.toRadians(90)), Math.toRadians(0)) //might be +150? idk will have to test
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .stopAndAdd(bot.shootThreeAutoClose())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45,65))
                .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.component1().x, Pos.blueCloseIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .strafeToLinearHeading(Pos.gate.position, Pos.gate.heading)
                .waitSeconds(1)

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                .stopAndAdd(bot.shootThreeAutoClose())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .setTangent(Math.toRadians(135))
                .splineTo(Pos.blueMidIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.component1().x, Pos.blueMidIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .splineTo(Pos.closeShoot, Math.toRadians(-60))
                .stopAndAdd(bot.shootThreeAutoClose())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.component1().x, Pos.blueFarIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .setReversed(true)
                .splineTo(Pos.closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50,70))
                .stopAndAdd(bot.shootThreeAutoClose())
                .build();

        Action redFarAuto = drive.actionBuilderRed(Pos.initialFarBluePose)

                .afterTime(0.2, bot.enableShooter())
                .strafeToConstantHeading(Pos.closeShoot)
                .stopAndAdd(bot.shootThreeAutoClose())
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .stopAndAdd(new InstantAction(()-> bot.disableShooter()))

                .setTangent(Math.toRadians(160))
                .splineTo((Pos.blueHpIntakeInter), Math.toRadians(180))
                .splineToSplineHeading(Pos.blueHpIntake, Math.toRadians(80))
                .strafeToConstantHeading(new Vector2d(Pos.blueHpIntake.component1().x - 11.5, Pos.blueHpIntake.component1().y))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .afterTime(0.1, bot.enableShooter())
                .splineToSplineHeading(new Pose2d(Pos.closeFirstShoot, Math.toRadians(90)), Math.toRadians(0)) //might be +150? idk will have to test
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .stopAndAdd(bot.shootThreeAutoClose())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(Pos.blueCloseIntake.position, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45,65))
                .strafeToConstantHeading(new Vector2d(Pos.blueCloseIntake.component1().x, Pos.blueCloseIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .strafeToLinearHeading(Pos.gate.position, Pos.gate.heading)
                .waitSeconds(1)

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .strafeToSplineHeading(Pos.closeShoot, Math.toRadians(135))
                .stopAndAdd(bot.shootThreeAutoClose())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .setTangent(Math.toRadians(135))
                .splineTo(Pos.blueMidIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueMidIntake.component1().x, Pos.blueMidIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .splineTo(Pos.closeShoot, Math.toRadians(-60))
                .stopAndAdd(bot.shootThreeAutoClose())

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(Pos.blueFarIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(Pos.blueFarIntake.component1().x, Pos.blueFarIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .setReversed(true)
                .splineTo(Pos.closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50,70))
                .stopAndAdd(bot.shootThreeAutoClose())
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


    
}
