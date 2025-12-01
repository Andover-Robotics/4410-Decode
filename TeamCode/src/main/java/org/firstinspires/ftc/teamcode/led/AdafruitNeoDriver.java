package org.firstinspires.ftc.teamcode.led;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareDevice;
//import com.qualcomm.robotcore.hardware.HardwareDeviceManufacturer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;

/**
 * Minimal I2C driver for the Adafruit NeoDriver / Seesaw NeoPixel bridge.
 * The device accepts NeoPixel style buffers over I2C and handles the timing
 * for the attached LED strip or matrix. The class exposes only the subset of
 * commands required to stream RGB data to an 8x24 matrix.
 *
 * The register map is based on Adafruit's public Seesaw documentation:
 * - Module 0x0E (NEOPIXEL)
 *   - NEO_PIN      (0x01): set the NeoPixel output pin
 *   - NEO_SPEED    (0x02): set the speed (800kHz vs 400kHz)
 *   - NEO_BUF_LENGTH(0x03): configure the pixel buffer length
 *   - NEO_BUF      (0x04): write the raw pixel buffer
 *   - NEO_SHOW     (0x05): latch/display the latest buffer
 */
public class AdafruitNeoDriver extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    // Default I2C address for Seesaw NeoDriver boards
    public static final I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x49);

    private static final byte MODULE_NEOPIXEL = 0x0E;
    private static final byte REG_PIN = 0x01;
    private static final byte REG_SPEED = 0x02;
    private static final byte REG_BUF_LENGTH = 0x03;
    private static final byte REG_BUF = 0x04;
    private static final byte REG_SHOW = 0x05;

    private static final byte SPEED_800_KHZ = 0x00;

    private final int pixelCount;

    public AdafruitNeoDriver(@NonNull HardwareMap hardwareMap,
                             @NonNull String deviceName,
                             int pixelCount) {
        this(hardwareMap, deviceName, pixelCount, DEFAULT_ADDRESS);
    }

    public AdafruitNeoDriver(@NonNull HardwareMap hardwareMap,
                             @NonNull String deviceName,
                             int pixelCount,
                             @NonNull I2cAddr address) {
        super(hardwareMap.get(I2cDeviceSynch.class, deviceName), true);
        this.pixelCount = pixelCount;
        this.deviceClient.setI2cAddress(address);
        registerArmingStateCallback(true);
        deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        // Configure NeoPixel pin (default D5 on the breakout) and speed.
        write8(MODULE_NEOPIXEL, REG_PIN, (byte) 5);
        write8(MODULE_NEOPIXEL, REG_SPEED, SPEED_800_KHZ);
        // Configure buffer length (3 bytes per pixel: GRB order on Seesaw).
        int byteLength = pixelCount * 3;
        write16(MODULE_NEOPIXEL, REG_BUF_LENGTH, byteLength);
        return true;
    }

    /**
     * Write the entire NeoPixel buffer and trigger a latch.
     * @param buffer byte array containing GRB data for all pixels.
     */
    public void writeFrame(@NonNull byte[] buffer) {
        if (buffer.length != pixelCount * 3) {
            throw new IllegalArgumentException("Buffer length does not match pixel count");
        }
        writeBuffer(MODULE_NEOPIXEL, REG_BUF, buffer);
        // Latch the new frame
        write8(MODULE_NEOPIXEL, REG_SHOW, (byte) 0x00);
    }

    private void write8(byte module, byte register, byte value) {
        byte[] payload = new byte[]{module, register, value};
        deviceClient.write(payload, I2cWaitControl.WRITTEN);
    }

    private void write16(byte module, byte register, int value) {
        byte[] payload = new byte[]{module, register, (byte) (value >> 8), (byte) (value & 0xFF)};
        deviceClient.write(payload, I2cWaitControl.WRITTEN);
    }

    private void writeBuffer(byte module, byte register, byte[] data) {
        byte[] payload = new byte[data.length + 2];
        payload[0] = module;
        payload[1] = register;
        System.arraycopy(data, 0, payload, 2, data.length);
        deviceClient.write(payload, I2cWaitControl.WRITTEN);
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit NeoDriver (Seesaw)";
    }

    @Override
    public void close() {
        // Nothing beyond disengaging the client
        deviceClient.disengage();
    }
}
