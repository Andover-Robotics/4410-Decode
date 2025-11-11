package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.sprites.Sprite;

public class RectangleSprite extends Sprite {
    public RectangleSprite(int x, int y, int width, int height, Color color) {
        super(x, y, width, height);
        this.setColor(color);
    }

    @Override
    protected void showSprite(Color[][] leds) {
        for(int x = this.getX(); x < this.getX() + this.getWidth(); x++){
            for(int y = this.getY(); y < this.getY() + this.getHeight(); y++)
                leds[x][y] = this.getColor();
        }
    }
}
