package org.firstinspires.ftc.teamcode.teleop.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.sprites.Sprite;

public class RectangleOutlineSprite extends Sprite {

    /**
     * Creates a new rectangle sprite
     *
     * @param x      the x coordinate of the bottom left corner of the sprite within the region
     * @param y      the y coordinate of the bottom left corner of the sprite within the region
     * @param width  the width of the line
     * @param height the height of the line
     */
    public RectangleOutlineSprite(int x, int y, int width, int height) {
        super(x, y, width, height);
    }


    @Override
    protected void showSprite(Color[][] leds) {
        for (int row = 0; row < getHeight(); row++) {
            leds[getX()][row] = getColor();
            leds[getX() + getWidth() - 1][row] = getColor();
        }
        for (int col = 0; col < getWidth(); col++) {
            leds[col][getY()] = getColor();
            leds[col][getY() + getHeight() - 1] = getColor();
        }
    }
}