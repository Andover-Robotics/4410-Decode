package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.SRSHub;

@Config
public class SRSHubs {

    public static SRSHub leftHub;
    public static SRSHub rightHub;

    public static final SRSHub.APDS9151 rightFront = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 rightBack = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 backBottom = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 leftFront = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 leftBack = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 backRight = new SRSHub.APDS9151();
    private static boolean initialized;
    public static boolean leftUpdate = true;

    public SRSHubs(OpMode opMode) {
        this(opMode.hardwareMap);
    }

    public SRSHubs(HardwareMap hardwareMap) {
        synchronized (SRSHubs.class) {
            if (initialized) {
                return;
            }

            leftHub = hardwareMap.get(SRSHub.class, "srshubLeft");
            rightHub = hardwareMap.get(SRSHub.class, "srshubRight");

            SRSHub.Config leftConfig = new SRSHub.Config();
            leftConfig.addI2CDevice(1, rightFront);
            leftConfig.addI2CDevice(2, rightBack);
            leftConfig.addI2CDevice(3, backBottom);

            SRSHub.Config rightConfig = new SRSHub.Config();
            rightConfig.addI2CDevice(1, leftFront);
            rightConfig.addI2CDevice(2, leftBack);
            rightConfig.addI2CDevice(3, backRight);

            leftHub.init(leftConfig);
            rightHub.init(rightConfig);
            initialized = true;
        }
    }

    public void update() {
        if (!initialized) {
            throw new IllegalStateException("SRSHubs must be initialized before update()");
        }
        if (leftUpdate) {
            leftHub.update();
            leftUpdate = false;
        } else {
            rightHub.update();
            leftUpdate = true;
        }
//        leftHub.update();
//        rightHub.update();
    }
}
