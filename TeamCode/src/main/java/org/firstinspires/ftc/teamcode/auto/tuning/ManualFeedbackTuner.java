package org.firstinspires.ftc.teamcode.auto.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;
    public static boolean runShooter = false;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            MotorEx motor1 = new MotorEx(hardwareMap, "shooterL");
            MotorEx motor2 = new MotorEx(hardwareMap, "shooterR");
//            motor1.set(1);
//            motor2.set(-1);
            waitForStart();

            int i = 0;

            while (opModeIsActive() && i < 100) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
//                            .strafeToConstantHeading(new Pose2d(0, DISTANCE, 0).position)
//                            .strafeToConstantHeading(new Pose2d(0, 0, 0).position)
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
                i++;
            }
        } else {
            throw new RuntimeException();
        }
    }
}
