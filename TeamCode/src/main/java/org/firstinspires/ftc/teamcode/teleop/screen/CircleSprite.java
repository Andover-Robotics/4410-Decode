package org.firstinspires.ftc.teamcode.teleop.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.RectangleSprite;
import team.techtigers.core.display.sprites.Sprite;
import team.techtigers.core.display.sprites.XSprite;


/**
 * A class which displays a "circle" with even diameter (6 LEDs)
 * inside an 8x8 region, using a rounded rhombus / diamond shape.
 *
 * On an 8x8 region this produces:
 *
 *  0 0 0 0 0 0 0 0
 *  0 0 0 1 1 0 0 0
 *  0 0 1 1 1 1 0 0
 *  0 1 1 1 1 1 1 0
 *  0 1 1 1 1 1 1 0
 *  0 0 1 1 1 1 0 0
 *  0 0 0 1 1 0 0 0
 *  0 0 0 0 0 0 0 0
 */
public class CircleSprite extends Sprite {

    /**
     * Creates a new circle sprite.
     *
     * @param x      the x coordinate of the bottom left corner of the sprite within the region
     * @param y      the y coordinate of the bottom left corner of the sprite within the region
     * @param size   the size of the sprite region (use 8 for an 8x8 matrix)
     */
    public CircleSprite(int x, int y, int size) {
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
        int diameter = size - 2;        // 6 when size == 8
        int radius = diameter / 2;      // 3 when size == 8

        // Center at half-integer for even size: 3.5 on 0..7
        double center = (size / 2.0) - 0.5;

        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {

                double dx = col - center;
                double dy = row - center;

                double manhattan = Math.abs(dx) + Math.abs(dy);

                if (manhattan <= radius + 1e-9) {
                    leds[baseX + col][baseY + row] = color;
                }
            }
        }

    }
}