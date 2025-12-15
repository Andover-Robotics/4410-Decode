package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class Pos {
    // SOME VARIABLES
    public static final double WALL_INTAKE_ANGLE = Math.toRadians(140);

    // INITIAL
    public static Pose2d initialFarBluePose = new Pose2d(-63, 9, Math.toRadians(90));
    public static Pose2d initialCloseBluePose = new Pose2d(61, 38, Math.toRadians(-180));
    public static Pose2d initialFarRedPose = transformRed(initialFarBluePose);
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE
    public static Pose2d blueHpIntake = new Pose2d(-49, 61.5, WALL_INTAKE_ANGLE);
    public static Pose2d blueHpIntakeCycles = new Pose2d(-53, 61.5, Math.toRadians(130));
    public static Pose2d cycleIntake = new Pose2d(-56, 62, WALL_INTAKE_ANGLE);
    public static Pose2d blueStraightHpIntake = new Pose2d(-50, 61.5, Math.toRadians(128));
    public static Pose2d blueFarIntake = new Pose2d(-33.5, 28, Math.toRadians(90));
    public static Pose2d blueFarIntakeFar = new Pose2d(-33.5, 30, Math.toRadians(90));
    public static Pose2d blueMidIntakeFar = new Pose2d(-15, 29, Math.toRadians(90));
    public static Pose2d blueMidIntakeClose = new Pose2d(-13, 30, Math.toRadians(90));
    public static Pose2d redMidIntakeFar = new Pose2d(-13, 30, Math.toRadians(90));
    public static Pose2d redFarIntakeFar = new Pose2d(-33.5, 30, Math.toRadians(90));
    public static Pose2d blueCloseIntake = new Pose2d(13, 29, Math.toRadians(90));
    public static Vector2d blueHpIntakeInter = new Vector2d(-58, 24);

    public static Pose2d blueSecretTunnel = new Pose2d(-40, 59, Math.toRadians(135));

    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d closeShoot = new Vector2d(6, 17);
    public static Vector2d closeFirstShoot = new Vector2d(13, 15);
    public static Vector2d closeAutoLastShoot = new Vector2d(11, 13);
    public static Vector2d park = new Vector2d(-1, 21);
    public static Vector2d farShoot = new Vector2d(-57, 11);
    public static Vector2d midShoot = new Vector2d(-38, 17);



    public static Pose2d gate = new Pose2d(5, 57, Math.toRadians(0));

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(pose.position.x, -pose.position.y), -pose.heading.log());
    }
}
