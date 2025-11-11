package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.Sprite;

public class SecondRegion extends DisplayRegion {
    private final Sprite[] sprites;
    private boolean movingRight = true;

    private int currentX = 0;

    private int frameCounter = 0;

    public SecondRegion(int x, int y, int width, int height) {
        super(x,y,width,height);
        this.sprites = new Sprite[]{
                new RectangleSprite(1,1,2, 2, Color.PURPLE)
        };
    }

    @Override
    public void update() {
        frameCounter = frameCounter + 1;
        if(frameCounter < 15) return;
        frameCounter = 0;

        if(movingRight){
            currentX = currentX + 1;
            if(currentX > 5){
                movingRight = false;
            }
        } else {
            currentX = currentX - 1;
            if(currentX <= 1) {
                movingRight = true;
            }
        }
        this.sprites[0].setPosition(currentX, 1);

    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}