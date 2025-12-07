/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Prism;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.TimeUnit;

public class PrismAnimations {

    public static abstract class AnimationBase {
        public GoBildaPrismDriver.LayerHeight layerHeight = GoBildaPrismDriver.LayerHeight.DISABLED;
        public abstract void updateAnimationOverI2C(I2cDeviceSynchSimple device, boolean isNew);
    }

    public static class BackgroundLayer extends AnimationBase {
        private int color;

        public BackgroundLayer(int rgb) {
            this.color = rgb;
        }

        public void setColor(int rgb) {
            this.color = rgb;
        }

        @Override
        public void updateAnimationOverI2C(I2cDeviceSynchSimple device, boolean isNew) {
            int cmd =
                    (0x01 << 28) |
                            (color & 0xFFFFFF);

            byte[] data = TypeConversion.intToByteArray(cmd, ByteOrder.LITTLE_ENDIAN);
            device.write(layerHeight.register.address, data);
        }
    }

    public static class CircleLayer extends AnimationBase {
        private final int cx, cy;
        private final int innerRadius;
        private final int outerRadius;
        private final int innerColor;
        private final int outlineColor;

        public CircleLayer(int cx, int cy,
                           int innerRadius,
                           int outerRadius,
                           int innerColor,
                           int outlineColor) {
            this.cx = cx;
            this.cy = cy;
            this.innerRadius = innerRadius;
            this.outerRadius = outerRadius;
            this.innerColor = innerColor;
            this.outlineColor = outlineColor;
        }

        @Override
        public void updateAnimationOverI2C(I2cDeviceSynchSimple device, boolean isNew) {
            int cmd =
                    (0x02 << 28) |
                            ((cx & 0x1F) << 20) |
                            ((cy & 0x1F) << 15) |
                            ((innerRadius & 0x07) << 12) |
                            (outlineColor & 0x0FFF);

            byte[] data = TypeConversion.intToByteArray(cmd, ByteOrder.LITTLE_ENDIAN);
            device.write(layerHeight.register.address, data);
        }
    }
}