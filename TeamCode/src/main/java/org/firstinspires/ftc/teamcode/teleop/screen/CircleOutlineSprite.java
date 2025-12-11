package org.firstinspires.ftc.teamcode.teleop.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.RectangleSprite;
import team.techtigers.core.display.sprites.Sprite;
import team.techtigers.core.display.sprites.XSprite;


/**
 * A class which displays a ring ("halo") that is larger
 * than the inner 6-LED circle on an 8x8 matrix.
 */
public class CircleOutlineSprite extends Sprite {

    /**
     * @param x    bottom-left x of the 8x8 region
     * @param y    bottom-left y of the 8x8 region
     * @param size size of the region (use 8)
     */
    public CircleOutlineSprite(int x, int y, int size) {
        super(x, y, size, size);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        int size = getWidth();          // 8
        int baseX = getX();
        int baseY = getY();
        Color color = getColor();

        int diameter = size - 2;        // 6
        int innerRadius = diameter / 2; // 3
        int outerRadius = innerRadius + 1; // 4 (bigger than circle)

        double center = (size / 2.0) - 0.5; // 3.5

        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                double dx = col - center;
                double dy = row - center;
                double manhattan = Math.abs(dx) + Math.abs(dy);

                // Ring at radius 4, 1-pixel thick
                if (Math.abs(manhattan - outerRadius) < 1e-9) {
                    leds[baseX + col][baseY + row] = color;
                }
            }
        }
    }
}