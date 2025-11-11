package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.Sprite;

public class BarTestRegion extends DisplayRegion {
    private final Sprite[] sprites;

    public BarTestRegion(int x, int y, int width, int height) {
        super(x,y,width,height);
        this.sprites = new Sprite[]{
                new RectangleSprite(1,1,2, 6, Color.GREEN),
                new RectangleSprite(4,1,2, 6, Color.GREEN),
                new RectangleSprite(7,1,2, 6, Color.GREEN)
        };
    }

    @Override
    public void update() {

    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
