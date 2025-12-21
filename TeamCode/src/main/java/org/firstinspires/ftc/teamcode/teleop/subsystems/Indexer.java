package org.firstinspires.ftc.teamcode.teleop.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Indexer {

    public static double kickerLeftDown = 0.5;
    public static double kickerLeftUp = 0.7;
    public static double kickerRightDown = 0.5;
    public static double kickerRightUp = 0.7;
    public static double kickerBackDown = 0.5;
    public static double kickerBackUp = 0.7;

    public Servo leftKicker;
    public Servo rightKicker;
    public Servo backKicker;

    public RevColorSensorV3 colorRR;
    public RevColorSensorV3 colorRL;
    public RevColorSensorV3 colorLL;
    public RevColorSensorV3 colorLR;
    public RevColorSensorV3 colorBL;
    public RevColorSensorV3 colorBR;



    public Indexer (OpMode opmode) {
        leftKicker= opMode.hardwareMap.servo.get("leftKicker");
        rightKicker= opMode.hardwareMap.servo.get("rightKicker");
        backKicker= opMode.hardwareMap.servo.get("backKicker");

        colorRR= opMode.hardwareMap.get(RevColorSensorV3.class, "colorRR");
        colorRL= opMode.hardwareMap.get(RevColorSensorV3.class, "colorRL");
        colorLL= opMode.hardwareMap.get(RevColorSensorV3.class, "colorLL");
        colorLR= opMode.hardwareMap.get(RevColorSensorV3.class, "colorLR");
        colorBR= opMode.hardwareMap.get(RevColorSensorV3.class, "colorBR");
        colorBL= opMode.hardwareMap.get(RevColorSensorV3.class, "colorBL");

    }

}


