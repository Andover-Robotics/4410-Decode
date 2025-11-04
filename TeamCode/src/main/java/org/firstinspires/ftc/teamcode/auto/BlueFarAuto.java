package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;

@Config
@Autonomous(name = "Blue Far Auto", group = "Autonomous")
public class BlueFarAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    //Coordinates ->
    // positive X is towards goal side, negative x is towards HP Stations
    // positive Y is towards blue goal / red driver side, negative Y is towards red goal / blue driver side
    // 0 degrees is toward goals, -45 is red goal, +45 is blue goal

    // SOME VARIABLES
    public static final double WALL_INTAKE_ANGLE = Math.toRadians(150);

    // INITIAL
    public static Pose2d initialFarBluePose = new Pose2d(-58, 48, Math.toRadians(0));
    public static Pose2d initialCloseBluePose = new Pose2d(63, 12, Math.toRadians(-135));
    public static Pose2d initialFarRedPose = transformRed(initialFarBluePose);
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE
    public static Pose2d blueHpIntake = new Pose2d(-58, 64, WALL_INTAKE_ANGLE);
    public static Pose2d blueFarIntake = new Pose2d(-36, 32, Math.toRadians(90));
    public static Pose2d blueMidIntake = new Pose2d(-12, 32, Math.toRadians(90));
    public static Pose2d blueCloseIntake = new Pose2d(12, 32, Math.toRadians(90));

    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d closeShoot = new Vector2d(6, 18);
    public static Vector2d farShoot = new Vector2d(-60, 12);

    public static Pose2d gate = new Pose2d(4, 60, Math.toRadians(0));
    public static Pose2d leave = new Pose2d(24, 12, Math.toRadians(0));

    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialFarBluePose);

        Action shootPreload = drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(new SequentialAction(
                        bot.enableShooter(),
                        new SleepAction(0.5),
                        bot.shootThree(),
                        bot.disableShooter()
                        )
                )
                .build();

        Action intakeHPAndShoot = drive.actionBuilder(initialCloseBluePose)
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))

                .setTangent(Math.toRadians(30))
                .splineToSplineHeading(blueHpIntake, Math.toRadians(-45))
                .strafeToConstantHeading(new Vector2d(blueHpIntake.component1().x - 8, blueHpIntake.component1().y))

                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .afterTime(0.1, bot.enableShooter())
                .splineTo(closeShoot, Math.toRadians(-30)) //might be +150? idk will have to test
                .stopAndAdd(bot.shootThree())
                .build();

        Action intakeCloseOpenGateShootThree = drive.actionBuilder(new Pose2d(closeShoot, Math.toRadians(150)))
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(blueCloseIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(blueCloseIntake.component1().x, blueCloseIntake.component1().y + 17))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))
                .strafeToLinearHeading(gate.position, gate.heading)
                .waitSeconds(2)

                .stopAndAdd(bot.enableShooter())
                .strafeToSplineHeading(closeShoot, Math.toRadians(90))
                .stopAndAdd(bot.shootThree())
                .build();

        Action intakeMidShootThree = drive.actionBuilder(new Pose2d(closeShoot, Math.toRadians(90)))
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(blueMidIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(blueMidIntake.component1().x, blueMidIntake.component1().y + 17))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .stopAndAdd(bot.enableShooter())
                .setReversed(true)
                .splineTo(closeShoot, Math.toRadians(-60))
                .stopAndAdd(bot.shootThree())
                .build();

        Action intakeFarShootThree = drive.actionBuilder(new Pose2d(closeShoot, Math.toRadians(-60)))
                .stopAndAdd(new InstantAction(()-> bot.intake.intake()))
                .splineTo(blueFarIntake.position, Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(blueFarIntake.component1().x, blueFarIntake.component1().y + 15))
                .stopAndAdd(new InstantAction(()-> bot.intake.storage()))

                .splineTo(closeShoot, Math.toRadians(-45))
                .stopAndAdd(bot.shootThree())
                .build();

        Action leaveZone = drive.actionBuilder(new Pose2d(closeShoot, Math.toRadians(-45)))
                .strafeToConstantHeading(leave.position)
                .build();

        bot.turret.trackObelisk();
        Bot.alliance = Bot.allianceOptions.BLUE_ALLIANCE;
        Bot.startingPos = Bot.startingPosition.FAR;
        while (!isStarted()) {
            bot.periodic();
            gp1.readButtons();

            telemetry.addData("ALLIANCE", Bot.alliance);
            telemetry.addData("STARTING POSITION", Bot.startingPos);
            telemetry.addData("DETECTED MOTIF", Turret.motif);
            telemetry.update();
        }

        bot.enableFullAuto(true);

        Actions.runBlocking(
                new ActionHelper.RaceParallelCommand(
                        bot.actionPeriodic(),
                        new SequentialAction(
                                shootPreload,
                                intakeHPAndShoot,
                                intakeCloseOpenGateShootThree,
                                intakeMidShootThree,
                                intakeFarShootThree,
                                leaveZone
                        )
                )
        );
    }

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(-pose.position.x, pose.position.y), Math.PI - pose.heading.log());
    }
}
