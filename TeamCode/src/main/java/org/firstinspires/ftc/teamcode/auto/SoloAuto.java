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

import org.firstinspires.ftc.teamcode.auto.piplines.AprilTagLimelightTest;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@Autonomous(name = "Solo Auto", group = "Autonomous")
public class SoloAuto extends LinearOpMode {
    Bot bot;
    private GamepadEx gp1;

    // INITIAL
    public static Pose2d initialPose = new Pose2d(12, -60, Math.toRadians(0));

    // INTAKE
    public static Pose2d hpIntake = new Pose2d(60, -60, Math.toRadians(-30));
    public static Pose2d farIntake = new Pose2d(36, -36, Math.toRadians(0));
    public static Pose2d midIntake = new Pose2d(36, -12, Math.toRadians(0));
    public static Pose2d closeIntake = new Pose2d(36, 12, Math.toRadians(0));

    // SHOOTING
    public static Pose2d closeShoot = new Pose2d(12, 12, Math.toRadians(0));
    public static Pose2d farShoot = new Pose2d(12, -60, Math.toRadians(0));

    public static Pose2d gate = new Pose2d(60, 0, Math.toRadians(90));
    public static Pose2d leave = new Pose2d(24, -24, Math.toRadians(0));

    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        gp1 = new GamepadEx(gamepad1);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // TODO: replace all actions with real names
    }

}
