package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SRSHub;

@Config
public class SRSHubs {
    public static int turretEncoderPort = 1;
    public static int shooterLeftEncoderPort = 2;
    public static int shooterRightEncoderPort = 3;

    public static double turretEncoderScale = 1.0;
    public static double shooterLeftEncoderScale = 1.0;
    public static double shooterRightEncoderScale = -1.0;
    public static float xOffset = (float) (-3.03 * 25.4), yOffset = (float) (-5.9 * 25.4);

    public final SRSHub leftHub;
    public final SRSHub rightHub;

    public final SRSHub.APDS9151 rightFront = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 rightBack = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 backBottom = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 leftFront = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 leftBack = new SRSHub.APDS9151();
    public final SRSHub.APDS9151 backRight = new SRSHub.APDS9151();
    public final SRSHub.GoBildaPinpoint pinpoint = new SRSHub.GoBildaPinpoint(xOffset, yOffset, 19.89436789f, SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD, SRSHub.GoBildaPinpoint.EncoderDirection.REVERSED);

    public SRSHubs(OpMode opMode) {
        this(opMode.hardwareMap);
    }

    public SRSHubs(HardwareMap hardwareMap) {
        leftHub = hardwareMap.get(SRSHub.class, "srshubLeft");
        rightHub = hardwareMap.get(SRSHub.class, "srshubRight");

        SRSHub.Config leftConfig = new SRSHub.Config();
        leftConfig.addI2CDevice(1, rightFront);
        leftConfig.addI2CDevice(2, rightBack);
        leftConfig.addI2CDevice(3, backBottom);
        leftConfig.addI2CDevice(1, pinpoint);

        SRSHub.Config rightConfig = new SRSHub.Config();
        rightConfig.addI2CDevice(1, leftFront);
        rightConfig.addI2CDevice(2, leftBack);
        rightConfig.addI2CDevice(3, backRight);
        rightConfig.setEncoder(turretEncoderPort, SRSHub.Encoder.QUADRATURE);
        rightConfig.setEncoder(shooterLeftEncoderPort, SRSHub.Encoder.QUADRATURE);
        rightConfig.setEncoder(shooterRightEncoderPort, SRSHub.Encoder.QUADRATURE);

        leftHub.init(leftConfig);
        rightHub.init(rightConfig);
    }

    public void update() {
        leftHub.update();
        rightHub.update();
    }

    public SRSHub.GoBildaPinpoint getPinpoint() {
        return getPinpointHub().getI2CDevice(
                1,
                SRSHub.GoBildaPinpoint.class
        );
    }

    public SRSHub getPinpointHub() {
        return leftHub;
    }

    public int getTurretPositionTicks() {
        return (int) Math.round(rightHub.readEncoder(turretEncoderPort).position * turretEncoderScale);
    }

    public double getShooterLeftVelocityTicksPerSecond() {
        return rightHub.readEncoder(shooterLeftEncoderPort).velocity * shooterLeftEncoderScale;
    }

    public double getShooterRightVelocityTicksPerSecond() {
        return rightHub.readEncoder(shooterRightEncoderPort).velocity * shooterRightEncoderScale;
    }
}
