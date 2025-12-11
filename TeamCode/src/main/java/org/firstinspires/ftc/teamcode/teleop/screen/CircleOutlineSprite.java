package org.firstinspires.ftc.teamcode.teleop.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.RectangleSprite;
import team.techtigers.core.display.sprites.Sprite;
import team.techtigers.core.display.sprites.XSprite;

/**
 * A "donut" sprite: a ring that is larger than the inner 6-LED circle
 * on an 8x8 matrix, without touching the inner circle.
 *
 * Intended to pair with the Manhattan radius-3 circle:
 *
 * Inner circle pattern (radius 3, what you showed):
 *
 *  0 0 0 0 0 0 0 0
 *  0 0 0 1 1 0 0 0
 *  0 0 1 1 1 1 0 0
 *  0 1 1 1 1 1 1 0
 *  0 1 1 1 1 1 1 0
 *  0 0 1 1 1 1 0 0
 *  0 0 0 1 1 0 0 0
 *  0 0 0 0 0 0 0 0
 *
 * This sprite draws the outer ring (radius 4) around that.
 */
public class CircleOutlineSprite extends Sprite {

    /**
     * @param x    bottom-left x of the sprite region within the LED matrix
     * @param y    bottom-left y of the sprite region within the LED matrix
     * @param size size of the sprite region (use 8 for an 8x8 matrix)
     */
    public CircleOutlineSprite(int x, int y, int size) {
        super(x, y, size, size);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        int size = getWidth();          // == getHeight()
        int baseX = getX();
        int baseY = getY();
        Color color = getColor();

        // For an 8x8 region with a 6-LED diameter, we leave
        // a 1-pixel border around the shape:
        // diameter = size - 2; radius = diameter / 2;
        int diameter = size - 2;        // 6 when size == 8=
        int innerRadius = diameter / 2; // 3 when size == 8
        int outerRadius = innerRadius + 1; // 4

        // Center at half-integer for even size: 3.5 on 0..7
        double center = (size / 2.0) - 0.5;

        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {

                double dx = col - center;
                double dy = row - center;

                double manhattan = Math.abs(dx) + Math.abs(dy);

                if (manhattan > innerRadius + 1e-9 &&
                        manhattan <= outerRadius + 1e-9) {
                    leds[baseX + col][baseY + row] = color;
                }
            }
        }
    }
}
