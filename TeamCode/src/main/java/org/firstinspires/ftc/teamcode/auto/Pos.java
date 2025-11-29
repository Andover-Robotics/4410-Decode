package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
public class Pos {
    // SOME VARIABLES
    public static final double WALL_INTAKE_ANGLE = Math.toRadians(170);

    // INITIAL
    public static Pose2d initialFarBluePose = new Pose2d(-63, 9, Math.toRadians(90));
    public static Pose2d initialCloseBluePose = new Pose2d(61, 38, Math.toRadians(-180));
    public static Pose2d initialFarRedPose = transformRed(initialFarBluePose);
    public static Pose2d initialCloseRedPose = transformRed(initialCloseBluePose);

    // INTAKE
    public static Pose2d blueHpIntake = new Pose2d(-47, 61.5, WALL_INTAKE_ANGLE);
    public static Pose2d blueStraightHpIntake = new Pose2d(-47, 61.5, Math.toRadians(128));
    public static Pose2d blueFarIntake = new Pose2d(-35, 29, Math.toRadians(90));
    public static Pose2d blueMidIntake = new Pose2d(-11, 29, Math.toRadians(90));
    public static Pose2d blueCloseIntake = new Pose2d(13, 29, Math.toRadians(90));
    public static Vector2d blueHpIntakeInter = new Vector2d(-54, 24);

    // SHOOTING (Vectors, as we do not care about robot orientation here)
    public static Vector2d closeShoot = new Vector2d(5, 17);
    public static Vector2d closeFirstShoot = new Vector2d(13, 15);
    public static Vector2d farShoot = new Vector2d(-61, 9);
    public static Vector2d edgeShoot = new Vector2d(3,10);

    public static Pose2d gate = new Pose2d(7, 57, Math.toRadians(0));

    public static Pose2d transformRed(Pose2d pose) {
        return new Pose2d(new Vector2d(-pose.position.x, pose.position.y), Math.PI - pose.heading.log());
    }
}
