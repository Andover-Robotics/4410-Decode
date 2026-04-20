//package org.firstinspires.ftc.teamcode.auto;
//
//// RR-specific imports
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.auto.tuning.ActionHelper;
//import org.firstinspires.ftc.teamcode.auto.tuning.MecanumDrive;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
//@Config
//@Autonomous(name = "Far Shooting Auto", group = "Competition")
//public class OldFarShootingAuto extends LinearOpMode {
//    Bot bot;
//    private GamepadEx gp1;
//    public void runOpMode() throws InterruptedException {
//        Bot.instance = null;
//        bot = Bot.getInstance(this);
//
//        gp1 = new GamepadEx(gamepad1);
//
//        MecanumDrive drive = Bot.drive;
//
//        bot.setTargetGoalPose();
//        Bot.drive.localizer.recalibrateIMU();
//
//        Action blueFarAutoOnlyHpPreFar = drive.actionBuilderBlue(OldPoses.initialFarBluePose)
//
//                .stopAndAdd(new SequentialAction(
//                        new InstantAction(() -> bot.enableShooter(true)),
//                        new SleepAction(0.7)
////                        bot.shootThreeAutoFar()
//                ))
//                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
//                .stopAndAdd(new InstantAction(() -> bot.disableShooter()))
//
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(OldPoses.blueHpIntake, Math.toRadians(80))
//                .strafeToConstantHeading(new Vector2d(OldPoses.blueHpIntake.position.x - 11.5, OldPoses.blueHpIntake.position.y - 2))
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(OldPoses.blueHpIntake.position.x - 5, OldPoses.blueHpIntake.position.y - 7), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(OldPoses.blueHpIntake.position.x - 11.5, OldPoses.blueHpIntake.position.y), Math.toRadians(170))
//                .afterTime(0.1, bot.enableShooter())
//                .strafeToLinearHeading(OldPoses.farShoot, Math.toRadians(0))
////                .stopAndAdd(bot.shootThreeAutoFar())
//                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
//
//                .setTangent(Math.toRadians(0))
//                .splineToConstantHeading(OldPoses.blueFarIntake.position, Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(OldPoses.blueFarIntake.position.x, OldPoses.blueFarIntake.position.y + 18))
//                .setReversed(true)
//                .splineTo(OldPoses.farShoot, Math.toRadians(-160))
////                .stopAndAdd(bot.shootThreeAutoFar())
//
//                .splineTo(OldPoses.blueMidIntakeFar.position, Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(OldPoses.blueMidIntakeFar.position.x, OldPoses.blueMidIntakeFar.position.y + 18))
//                .strafeToLinearHeading(OldPoses.farShoot, Math.toRadians(0))
////                .stopAndAdd(bot.shootThreeAutoFar())
//
//                .build();
//
//        Action redFarAutoOnlyHpPreFar = drive.actionBuilderRed(OldPoses.initialFarBluePose)
//
//                .stopAndAdd(new SequentialAction(
//                        new InstantAction(() -> bot.enableShooter(true)),
//                        new SleepAction(0.7)
////                        bot.shootThreeAutoFar()
//                ))
//                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
//                .stopAndAdd(new InstantAction(() -> bot.disableShooter()))
//
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(OldPoses.blueHpIntake, Math.toRadians(80))
//                .strafeToConstantHeading(new Vector2d(OldPoses.blueHpIntake.position.x - 11.5, OldPoses.blueHpIntake.position.y - 2))
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(OldPoses.blueHpIntake.position.x - 5, OldPoses.blueHpIntake.position.y - 7), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(OldPoses.blueHpIntake.position.x - 11.5, OldPoses.blueHpIntake.position.y), Math.toRadians(170))
//                .afterTime(0.1, bot.enableShooter())
//                .strafeToLinearHeading(OldPoses.farShoot, Math.toRadians(0))
////                .stopAndAdd(bot.shootThreeAutoFar())
//                .stopAndAdd(new InstantAction(() -> bot.intake.intake()))
//
//                .setTangent(Math.toRadians(0))
//                .splineToConstantHeading(OldPoses.blueFarIntake.position, Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(OldPoses.blueFarIntake.position.x, OldPoses.blueFarIntake.position.y + 18))
//                .setReversed(true)
//                .splineTo(OldPoses.farShoot, Math.toRadians(-160))
////                .stopAndAdd(bot.shootThreeAutoFar())
//
//                .splineTo(OldPoses.blueMidIntakeFar.position, Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(OldPoses.blueMidIntakeFar.position.x, OldPoses.blueMidIntakeFar.position.y + 18))
//                .strafeToLinearHeading(OldPoses.farShoot, Math.toRadians(0))
////                .stopAndAdd(bot.shootThreeAutoFar())
//                .build();
//
////        bot.turret.trackObelisk();
//
//        bot.enableFullAuto(true);
//        bot.enableShooter(false);
//        bot.setAllianceBlue();
//        bot.setFar();
//        bot.intake.storage();
////        bot.setTargetFarAutoGoal(); unneeded
//
//        while (!isStarted()) {
//            bot.periodic();
//            gp1.readButtons();
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
//                bot.switchAlliance();
//            }
//            if (Bot.isBlue()) {
//                drive.localizer.setPose(OldPoses.initialFarBluePose);
//            } else {
//                drive.localizer.setPose(OldPoses.initialFarRedPose);
//            }
//
//            telemetry.addData("ALLIANCE (A)", Bot.getAlliance());
//            telemetry.addData("STARTING POSITION", Bot.getStartingPos());
////            telemetry.addData("DETECTED MOTIF", Turret.motif);
//            telemetry.update();
//        }
//        if (Bot.isBlue()) {
//            drive.localizer.setPose(OldPoses.initialFarBluePose);
//            Actions.runBlocking(
//                    new ActionHelper.RaceParallelCommand(
//                            bot.actionPeriodic(),
//                            new SequentialAction(
//                                    blueFarAutoOnlyHpPreFar
//                            )
//                    )
//            );
//        } else {
//            drive.localizer.setPose(OldPoses.initialFarRedPose);
//            Actions.runBlocking(
//                    new ActionHelper.RaceParallelCommand(
//                            bot.actionPeriodic(),
//                            new SequentialAction(
//                                    redFarAutoOnlyHpPreFar
//                            )
//                    )
//            );
//        }
//
//
//
//    }
//
//
//
//}
//
