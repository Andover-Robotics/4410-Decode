package org.firstinspires.ftc.teamcode.teleop.subsystems.screen;

import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.RectangleSprite;
import team.techtigers.core.display.sprites.Sprite;

public class ThirdRegion extends DisplayRegion {
    private final Sprite[] sprites;

    private boolean movingUp = true;

    private int currentY = 0;

    private int frameCounter = 0;


    public ThirdRegion(int x, int y, int width, int height) {
        super(x,y,width,height);
        this.sprites = new Sprite[]{
                new RectangleSprite(1,1,2,2)
        };
    }

    @Override
    public void update() {
        frameCounter = frameCounter + 1;
        if(frameCounter < 15) return;
        frameCounter = 0;

        if(movingUp){
            currentY = currentY + 1;
            if(currentY > 5){
                movingUp = false;
            }
        } else {
            currentY = currentY - 1;
            if(currentY <= 1) {
                movingUp = true;
            }
        }
        this.sprites[0].setPosition(1,currentY);

    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }
}
