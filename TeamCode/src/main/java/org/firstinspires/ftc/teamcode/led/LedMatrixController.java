package org.firstinspires.ftc.teamcode.led;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * High-level controller for an 8x24 NeoPixel matrix driven by the Adafruit
 * NeoDriver over I2C. The controller owns a {@link LedMatrixBuffer} and pushes
 * the buffer to the driver via {@link #update()}.
 */
public class LedMatrixController {
    private final LedMatrixBuffer buffer = new LedMatrixBuffer();
    private final AdafruitNeoDriver driver;

    private static final String[] DEFAULT_DEVICE_NAMES = new String[]{"i2cMatrix", "screen"};

    public LedMatrixController(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null, AdafruitNeoDriver.DEFAULT_ADDRESS);
    }

    public LedMatrixController(@NonNull HardwareMap hardwareMap,
                               @Nullable String i2cDeviceName) {
        this(hardwareMap, i2cDeviceName, AdafruitNeoDriver.DEFAULT_ADDRESS);
    }

    public LedMatrixController(@NonNull HardwareMap hardwareMap,
                               @Nullable String i2cDeviceName,
                               @NonNull I2cAddr address) {
        int pixelCount = LedMatrixBuffer.ROWS * LedMatrixBuffer.COLS;
        String[] candidates = i2cDeviceName == null
                ? DEFAULT_DEVICE_NAMES
                : new String[]{i2cDeviceName};
        driver = new AdafruitNeoDriver(
                AdafruitNeoDriver.findDevice(hardwareMap, candidates),
                pixelCount,
                address);
        driver.initialize();
    }

    public LedMatrixBuffer getBuffer() {
        return buffer;
    }

    /**
     * Push the current buffer contents to the I2C NeoDriver.
     */
    public void update() {
        byte[] frame = buffer.toNeoPixelBuffer();
        driver.writeFrame(frame);
    }
}
