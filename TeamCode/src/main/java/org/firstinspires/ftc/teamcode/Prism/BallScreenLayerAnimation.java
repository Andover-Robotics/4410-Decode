package org.firstinspires.ftc.teamcode.Prism;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

/**
 * A simple {@link PrismAnimations.AnimationBase} that renders a fully custom frame to the Prism.
 *
 * <p>This leverages the Prism animation pipeline (layer heights, artboards, etc.) while still
 * allowing us to stream arbitrary LED data for the BallScreen artboards. The frame is written via
 * the control register so it is stored on the Prism rather than composited on the phone.</p>
 */
public class BallScreenLayerAnimation extends PrismAnimations.AnimationBase {

    private Color[] frame;

    public BallScreenLayerAnimation() {
        super(PrismAnimations.AnimationType.NONE, 100, 0,
                GoBildaPrismDriver.MATRIX_LED_COUNT - 1, GoBildaPrismDriver.LayerHeight.LAYER_0);
    }

    public void setFrame(Color[] frame) {
        this.frame = frame;
    }

    @Override
    protected void updateAnimationSpecificValuesOverI2C(I2cDeviceSynchSimple deviceClient) {
        if (frame == null) return;

        int boundedLength = Math.min(frame.length, GoBildaPrismDriver.MATRIX_LED_COUNT);
        for (int i = 0; i < boundedLength; i++) {
            Color color = frame[i] == null ? Color.TRANSPARENT : frame[i];
            byte[] packet = new byte[]{
                    (byte) i,
                    (byte) color.red,
                    (byte) color.green,
                    (byte) color.blue
            };
            deviceClient.write(GoBildaPrismDriver.Register.CONTROL.address, packet);
        }
    }
}
