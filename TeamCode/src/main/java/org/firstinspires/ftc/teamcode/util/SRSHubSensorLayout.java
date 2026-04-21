package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.tuning.PinpointLocalizer;

public final class SRSHubSensorLayout {
    public static final SRSHub.APDS9151 rightFront = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 rightBack = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 backBottom = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 leftFront = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 leftBack = new SRSHub.APDS9151();
    public static final SRSHub.APDS9151 backRight = new SRSHub.APDS9151();

    public static final float GO_BILDA_4_BAR_TICKS_PER_MM = 19.89436789f;

    public static SRSHub.GoBildaPinpoint pinpoint;

    public static final int PINPOINT_BUS = 1;

    private static boolean initialized = false;
    private static SRSHub srsHubLeft;
    private static SRSHub srsHubRight;

    private SRSHubSensorLayout() {
    }

    public static synchronized void ensureInitialized(HardwareMap hardwareMap) {
        ensureInitialized(hardwareMap, PinpointLocalizer.xOffset, PinpointLocalizer.yOffset);
    }

    public static synchronized void ensureInitialized(HardwareMap hardwareMap, double xOffsetInches, double yOffsetInches) {
        if (initialized) {
            return;
        }

        srsHubLeft = hardwareMap.get(SRSHub.class, "srshubLeft");
        srsHubRight = hardwareMap.get(SRSHub.class, "srshubRight");

        pinpoint = new SRSHub.GoBildaPinpoint(
                (float) (xOffsetInches * 25.4),
                (float) (yOffsetInches * 25.4),
                GO_BILDA_4_BAR_TICKS_PER_MM,
                SRSHub.GoBildaPinpoint.EncoderDirection.FORWARD,
                SRSHub.GoBildaPinpoint.EncoderDirection.REVERSED
        );

        SRSHub.Config leftConfig = new SRSHub.Config();
        leftConfig.addI2CDevice(1, rightFront);
        leftConfig.addI2CDevice(1, pinpoint);
        leftConfig.addI2CDevice(2, rightBack);
        leftConfig.addI2CDevice(3, backBottom);

        SRSHub.Config rightConfig = new SRSHub.Config();
        rightConfig.addI2CDevice(1, leftFront);
        rightConfig.addI2CDevice(2, leftBack);
        rightConfig.addI2CDevice(3, backRight);

        srsHubLeft.init(leftConfig);
        srsHubRight.init(rightConfig);

        initialized = true;
    }

    public static SRSHub getLeftHub() {
        return srsHubLeft;
    }

    public static SRSHub getRightHub() {
        return srsHubRight;
    }

    public static SRSHub getPinpointHub() {
        return srsHubLeft;
    }
}
