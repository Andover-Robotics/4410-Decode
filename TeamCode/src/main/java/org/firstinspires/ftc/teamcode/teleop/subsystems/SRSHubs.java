package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.SRSHub;

@Config
public class SRSHubs {
    public static int turretEncoderPort = 1;
    public static int shooterLeftEncoderPort = 2;
    public static int shooterRightEncoderPort = 3;

    public static double turretEncoderScale = 1.0;
    public static double shooterLeftEncoderScale = 1.0;
    public static double shooterRightEncoderScale = -1.0;

    public final SRSHub leftHub;
    public final SRSHub rightHub;

    public final SRSHub.APDS9151 rightFront = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 rightBack = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 backBottom = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 leftFront = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 leftBack = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 backRight = new SRSHub.APDS9151();

    public SRSHubs(OpMode opMode) {
        leftHub = opMode.hardwareMap.get(SRSHub.class, "srshubLeft");
        rightHub = opMode.hardwareMap.get(SRSHub.class, "srshubRight");

        SRSHub.Config leftConfig = new SRSHub.Config();
        leftConfig.addI2CDevice(1, rightFront);
        leftConfig.addI2CDevice(2, rightBack);
        leftConfig.addI2CDevice(3, backBottom);
        leftConfig.setEncoder(turretEncoderPort, SRSHub.Encoder.QUADRATURE);
        leftConfig.setEncoder(shooterLeftEncoderPort, SRSHub.Encoder.QUADRATURE);
        leftConfig.setEncoder(shooterRightEncoderPort, SRSHub.Encoder.QUADRATURE);

        SRSHub.Config rightConfig = new SRSHub.Config();
        rightConfig.addI2CDevice(1, leftFront);
        rightConfig.addI2CDevice(2, leftBack);
        rightConfig.addI2CDevice(3, backRight);

        leftHub.init(leftConfig);
        rightHub.init(rightConfig);
    }

    public void update() {
        leftHub.update();
        rightHub.update();
    }

    public int getTurretPositionTicks() {
        return (int) Math.round(leftHub.readEncoder(turretEncoderPort).position * turretEncoderScale);
    }

    public double getShooterLeftVelocityTicksPerSecond() {
        return leftHub.readEncoder(shooterLeftEncoderPort).velocity * shooterLeftEncoderScale;
    }

    public double getShooterRightVelocityTicksPerSecond() {
        return leftHub.readEncoder(shooterRightEncoderPort).velocity * shooterRightEncoderScale;
    }
}
