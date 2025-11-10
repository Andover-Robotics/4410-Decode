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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

@Config
@TeleOp(name = "Blue Far Auto", group = "Teleop")
public class BlueFarAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    //Coordinates ->
    // positive X is towards goal side, negative x is towards HP Stations
    // positive Y is towards blue goal / red driver side, negative Y is towards red goal / blue driver side
    // 0 degrees is toward goals, -45 is red goal, +45 is blue goal

    // SOME VARIABLES
    public static final double WALL_INTAKE_ANGLE = Math.toRadians(170);

    // INITIAL
    public static Pose2d initialFarBluePose = new Pose2d(-62, 12, Math.toRadians(90));
    public static Pose2d initialCloseBluePose = new Pose2d(63, 12, Math.toRadians(-135));
    public static Pose2d initialFarRedPose = transformRed(initialFarBluePose);
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE
    public static Pose2d blueHpIntake = new Pose2d(-48, 64, WALL_INTAKE_ANGLE);
    public static Pose2d blueFarIntake = new Pose2d(-32, 32, Math.toRadians(90));
    public static Pose2d blueMidIntake = new Pose2d(-10, 32, Math.toRadians(90));
    public static Pose2d blueCloseIntake = new Pose2d(14, 32, Math.toRadians(90));

    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d closeShoot = new Vector2d(6, 18);
    public static Vector2d closeFirstShoot = new Vector2d(14, 18);
    public static Vector2d farShoot = new Vector2d(-60, 12);

    public static Pose2d gate = new Pose2d(8, 60, Math.toRadians(0));
//    public static Pose2d leave = new Pose2d(-24, 12, Math.toRadians(0));

    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialFarBluePose);

        Action shootPreload = drive.actionBuilderBlue(drive.localizer.getPose())
                .stopAndAdd(new SequentialAction(
                        bot.enableShooter(),
                        new SleepAction(0.5),
                        bot.shootThreeAuto(),
                        bot.disableShooter()
                        )
                )
                .build();

        Action intakeHPAndShoot = drive.actionBuilderBlue(initialFarBluePose)

                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))

                // .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(blueHpIntake.component1().x, blueHpIntake.component1().y), Math.toRadians(150))
                //.splineToSplineHeading(blueHpIntake, Math.toRadians(150))
                .strafeToConstantHeading(new Vector2d(blueHpIntake.component1().x - 10, blueHpIntake.component1().y))

                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .afterTime(0.1, bot.enableShooter())
                .splineToSplineHeading(new Pose2d(closeFirstShoot, Math.toRadians(90)), Math.toRadians(0)) //might be +150? idk will have to test
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .stopAndAdd(bot.shootThreeAuto())
                .build();

        Action intakeCloseOpenGateShootThree = drive.actionBuilderBlue(new Pose2d(closeFirstShoot, Math.toRadians(90)))
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(blueCloseIntake.position, Math.toRadians(90), drive.defaultVelConstraint, new ProfileAccelConstraint(-45,65))
                .strafeToConstantHeading(new Vector2d(blueCloseIntake.component1().x, blueCloseIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .strafeToLinearHeading(gate.position, gate.heading)
                .waitSeconds(1)

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .strafeToSplineHeading(closeShoot, Math.toRadians(135))
//                .setTangent(Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(closeShoot, Math.toRadians(90)), Math.toRadians(-90))
                .stopAndAdd(bot.shootThreeAuto())
                .build();

        Action intakeMidShootThree = drive.actionBuilderBlue(new Pose2d(closeShoot, Math.toRadians(135)))
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .setTangent(Math.toRadians(135))
                .splineTo(blueMidIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(blueMidIntake.component1().x, blueMidIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .splineTo(closeShoot, Math.toRadians(-60))
                .stopAndAdd(bot.shootThreeAuto())
                .build();

        Action intakeFarShootThree = drive.actionBuilderBlue(new Pose2d(closeShoot, Math.toRadians(120)))
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(blueFarIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(blueFarIntake.component1().x, blueFarIntake.component1().y + 18))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .setReversed(true)
                .splineTo(closeShoot, Math.toRadians(-45), drive.defaultVelConstraint, new ProfileAccelConstraint(-50,70))
                .stopAndAdd(bot.shootThreeAuto())
                .build();


        bot.turret.trackObelisk();

        bot.enableFullAuto(true);
        bot.enableShooter(false);
        Bot.alliance = Bot.allianceOptions.BLUE_ALLIANCE;
        Bot.startingPos = Bot.startingPosition.FAR;
        bot.intake.closeGate();
        bot.intake.storage();
        while (!isStarted()) {
            bot.periodic();
            gp1.readButtons();

            telemetry.addData("ALLIANCE", Bot.alliance);
            telemetry.addData("STARTING POSITION", Bot.startingPos);
            telemetry.addData("DETECTED MOTIF", Turret.motif);
            telemetry.update();
        }

        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                shootPreload,
                                intakeHPAndShoot,
                                intakeCloseOpenGateShootThree,
                                intakeMidShootThree,
                                intakeFarShootThree
                        )
                )
        );
    }

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(-pose.position.x, pose.position.y), Math.PI - pose.heading.log());
    }
}
