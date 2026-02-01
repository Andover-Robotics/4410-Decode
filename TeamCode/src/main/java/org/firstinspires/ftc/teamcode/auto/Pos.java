package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class Pos {
    // SOME VARIABLES
    public static final double GATE_INTAKE_ANGLE = Math.toRadians(65);

    // INITIAL
    public static Pose2d initialFarBluePose = new Pose2d(-63, 9, Math.toRadians(0));
    public static Pose2d initialCloseBluePose = new Pose2d(60, 41, Math.toRadians(0));
    public static Pose2d initialFarRedPose = transformRed(initialFarBluePose);
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE
    public static Pose2d blueMidIntake = new Pose2d(-17, 32, Math.toRadians(90));
    public static int midIntake = 32;

    public static Pose2d gate = new Pose2d(-12, 63.5, GATE_INTAKE_ANGLE);
    public static Pose2d gateIntaking = new Pose2d(-14, 65.5, Math.toRadians(35));

    public static Pose2d gateSideOpen = new Pose2d(3, 61, Math.toRadians(-10));

    public static Pose2d blueCloseIntake = new Pose2d(11, 32, Math.toRadians(90));
    public static int closeIntake = 22;

    public static Pose2d blueFarIntake = new Pose2d(-35, 35, Math.toRadians(90));
    public static int farIntake = 29;

    public static Pose2d blueHpIntake = new Pose2d(-53, 67, Math.toRadians(179));


    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d closeShoot = new Vector2d(7, 16);
    public static Vector2d closeGateCycleShoot = new Vector2d(3, 20);
    public static Vector2d closePark = new Vector2d(-1, 21);
    public static Vector2d farShoot = new Vector2d(-55, 15);
    public static Vector2d farPark = new Vector2d(-44, 24);

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(pose.position.x, -pose.position.y), -pose.heading.log());
    }
}