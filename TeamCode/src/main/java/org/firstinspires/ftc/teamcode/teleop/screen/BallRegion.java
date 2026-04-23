////package org.firstinspires.ftc.teamcode.teleop.screen;
////
////
////import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
////import org.firstinspires.ftc.teamcode.teleop.subsystems.Intake;
////import org.firstinspires.ftc.teamcode.teleop.subsystems.Lift;
////import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;
////
////
////import team.techtigers.core.display.Color;
////import team.techtigers.core.display.DisplayRegion;
////import team.techtigers.core.display.sprites.RectangleSprite;
////import team.techtigers.core.display.sprites.Sprite;
////import team.techtigers.core.display.sprites.XSprite;
////
////public class BallRegion extends DisplayRegion {
////    private final CircleSprite circle1, circle2, circle3;
////    //    private final CircleOutlineSprite outline1, outline2, outline3;
////    private final RectangleSprite background;
////    private final RectangleOutlineSprite outline;
////    private final Sprite[] sprites;
////    public boolean teleop = false;
////    public Bot bot;
////
////
////    public BallRegion(int x, int y) {
////        super(x, y, 24, 8);
////        background = new RectangleSprite(0, 0, 24, 8);
////        background.setColor(Color.BLUE);
////        background.disable();
////
////        circle1 = new CircleSprite(0, 0, 8);
////        circle1.setColor(Color.GREEN);
////        circle1.enable();
////        circle2 = new CircleSprite(8, 0, 8);
////        circle2.setColor(Color.GREEN);
////        circle2.enable();
////        circle3 = new CircleSprite(16, 0, 8);
////        circle3.setColor(Color.GREEN);
////        circle3.enable();
////        outline = new RectangleOutlineSprite(0, 0, 24, 8);
////        outline.setColor(Color.BLACK);
////        outline.disable();
//////        outline1 = new CircleOutlineSprite(0, 0, 8);
//////        outline1.setColor(Color.WHITE);
//////        outline1.disable();
//////        outline2 = new CircleOutlineSprite(8, 0, 8);
//////        outline2.setColor(Color.WHITE);
//////        outline2.disable();
//////        outline3 = new CircleOutlineSprite(16, 0, 8);
//////        outline3.setColor(Color.WHITE);
//////        outline3.disable();
////
////        sprites = new Sprite[]{background, circle1, circle2, circle3, outline};
////    }
////
////    @Override
////    public void update() {
////        if (teleop) {
////            for (int i = 0; i < 3; i++) {
////                Sprite circle = sprites[i];
////                if (Lift.closedLoopEnabled) {
////                    circle.setColor(Color.RED);
////                } else if (bot.indexer.holders[i].getColor().equals("GREEN")) {
////                    circle.setColor(Color.GREEN);
////                } else if (bot.indexer.holders[i].getColor().equals("PURPLE")) {
////                    circle.setColor(Color.PURPLE);
////                } else if (bot.indexer.holders[i].getColor().equals("UNKNOWN")) {
////                    circle.setColor(Color.WHITE);
////                } else {
////                    circle.setColor(Color.BLACK);
////                }
////            }
////        } else if (ScreenTester.changed) {
////            for (int i = 0; i < 3; i++) {
////                Sprite circle = sprites[i];
////                if (circle.getColor().equals(Color.GREEN)) {
////                    circle.setColor(Color.BLACK);
////                } else {
////                    circle.setColor(Color.GREEN);
////                }
////            }
////            ScreenTester.changed = false;//useless btw
////        } else {
////            for (Sprite sprite : sprites) {
////                sprite.setColor(sprite.getColor());
////            }
////        }
////        if (Turret.deadzone) {
////            background.enable();
////            background.setColor(Color.RED);
////        } else {
////            background.disable();
////        }
////    }
////    //-55 -61 -90
////    //-55 -58 -91
////    // 20/20
////    // 20/20
////
////    public void setTeleop(boolean t, Bot bot) {
////        teleop = t;
////        this.bot = bot;
////    }
////
////    @Override
////    protected Sprite[] getSprites() {
////        return this.sprites;
////    }
////
////}
//package org.firstinspires.ftc.teamcode.teleop.screen;
//
//
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;
//
//
//import team.techtigers.core.display.Color;
//import team.techtigers.core.display.DisplayRegion;
//import team.techtigers.core.display.sprites.RectangleSprite;
//import team.techtigers.core.display.sprites.Sprite;
//import team.techtigers.core.display.sprites.XSprite;
//
//public class BallRegion extends DisplayRegion {
//    private final CircleSprite circle1, circle2, circle3;
//    //    private final CircleOutlineSprite outline1, outline2, outline3;
//    private final RectangleOutlineSprite outline;
////    private final RectangleSprite background;
//    private final Sprite[] sprites;
//    public boolean teleop = false;
//    public Bot bot;
//
//
//    public BallRegion(int x, int y) {
//        super(x, y, 8, 24);
////        background = new RectangleSprite(0, 0, 3, 3);
////        background.setColor(Color.RED);
////        background.disable();
//        circle1 = new CircleSprite(0, 0, 8);
//        circle1.setColor(Color.GREEN);
//        circle1.enable();
//        circle2 = new CircleSprite(0, 8, 8);
//        circle2.setColor(Color.GREEN);
//        circle2.enable();
//        circle3 = new CircleSprite(0, 16, 8);
//        circle3.setColor(Color.GREEN);
//        circle3.enable();
//        outline = new RectangleOutlineSprite(0, 0, 8, 24);
//        outline.setColor(Color.BLACK);
//        outline.disable();
////        outline1 = new CircleOutlineSprite(0, 0, 8);
////        outline1.setColor(Color.WHITE);
////        outline1.disable();
////        outline2 = new CircleOutlineSprite(8, 0, 8);
////        outline2.setColor(Color.WHITE);
////        outline2.disable();
////        outline3 = new CircleOutlineSprite(16, 0, 8);
////        outline3.setColor(Color.WHITE);
////        outline3.disable();
//
//        sprites = new Sprite[]{circle1, circle2, circle3, outline};
//    }
//
//    @Override
//    public void update() {
////        if (Turret.deadzone) {
////            background.enable();
////            background.setColor(Color.RED);
////        } else {
////            background.disable();
////        }
//        if (teleop) {
//            for (int i = 0; i < 3; i++) {
//                Sprite circle = sprites[i];
//                if (Lift.closedLoopEnabled) {
//                    circle.setColor(Color.RED);
//                } else if (bot.indexer.holders[i].getColor().equals("GREEN")) {
//                    circle.setColor(Color.GREEN);
//                } else if (bot.indexer.holders[i].getColor().equals("PURPLE")) {
//                    circle.setColor(Color.PURPLE);
//                } else if (bot.indexer.holders[i].getColor().equals("UNKNOWN")) {
//                    circle.setColor(Color.WHITE);
//                } else {
//                    circle.setColor(Color.BLACK);
//                }
//            }
//
//        } else if (ScreenTester.changed) {
//            for (int i = 0; i < 3; i++) {
//                Sprite circle = sprites[i];
//                if (circle.getColor().equals(Color.GREEN)) {
//                    circle.setColor(Color.BLACK);
//                } else {
//                    circle.setColor(Color.GREEN);
//                }
//            }
//            ScreenTester.changed = false;
//        } else {
//            for (Sprite sprite : sprites) {
//                sprite.setColor(sprite.getColor());
//            }
//        }
//    }
//
//    public void setTeleop(boolean t, Bot bot) {
//        teleop = t;
//        this.bot = bot;
//    }
//
//    @Override
//    protected Sprite[] getSprites() {
//        return this.sprites;
//    }
//
//}
//
//package org.firstinspires.ftc.teamcode.teleop.screen;
//
//
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;
//
//
//import team.techtigers.core.display.Color;
//import team.techtigers.core.display.DisplayRegion;
//import team.techtigers.core.display.sprites.RectangleSprite;
//import team.techtigers.core.display.sprites.Sprite;
//import team.techtigers.core.display.sprites.XSprite;
//
//public class BallRegion extends DisplayRegion {
//    private final CircleSprite circle1, circle2, circle3;
//    //    private final CircleOutlineSprite outline1, outline2, outline3;
//    private final RectangleSprite background;
//    private final RectangleOutlineSprite outline;
//    private final Sprite[] sprites;
//    public boolean teleop = false;
//    public Bot bot;
//
//
//    public BallRegion(int x, int y) {
//        super(x, y, 24, 8);
//        background = new RectangleSprite(0, 0, 24, 8);
//        background.setColor(Color.BLUE);
//        background.disable();
//
//        circle1 = new CircleSprite(0, 0, 8);
//        circle1.setColor(Color.GREEN);
//        circle1.enable();
//        circle2 = new CircleSprite(8, 0, 8);
//        circle2.setColor(Color.GREEN);
//        circle2.enable();
//        circle3 = new CircleSprite(16, 0, 8);
//        circle3.setColor(Color.GREEN);
//        circle3.enable();
//        outline = new RectangleOutlineSprite(0, 0, 24, 8);
//        outline.setColor(Color.BLACK);
//        outline.disable();
////        outline1 = new CircleOutlineSprite(0, 0, 8);
////        outline1.setColor(Color.WHITE);
////        outline1.disable();
////        outline2 = new CircleOutlineSprite(8, 0, 8);
////        outline2.setColor(Color.WHITE);
////        outline2.disable();
////        outline3 = new CircleOutlineSprite(16, 0, 8);
////        outline3.setColor(Color.WHITE);
////        outline3.disable();
//
//        sprites = new Sprite[]{background, circle1, circle2, circle3, outline};
//    }
//
//    @Override
//    public void update() {
//        if (teleop) {
//            for (int i = 0; i < 3; i++) {
//                Sprite circle = sprites[i];
//                if (Lift.closedLoopEnabled) {
//                    circle.setColor(Color.RED);
//                } else if (bot.indexer.holders[i].getColor().equals("GREEN")) {
//                    circle.setColor(Color.GREEN);
//                } else if (bot.indexer.holders[i].getColor().equals("PURPLE")) {
//                    circle.setColor(Color.PURPLE);
//                } else if (bot.indexer.holders[i].getColor().equals("UNKNOWN")) {
//                    circle.setColor(Color.WHITE);
//                } else {
//                    circle.setColor(Color.BLACK);
//                }
//            }
//        } else if (ScreenTester.changed) {
//            for (int i = 0; i < 3; i++) {
//                Sprite circle = sprites[i];
//                if (circle.getColor().equals(Color.GREEN)) {
//                    circle.setColor(Color.BLACK);
//                } else {
//                    circle.setColor(Color.GREEN);
//                }
//            }
//            ScreenTester.changed = false;//useless btw
//        } else {
//            for (Sprite sprite : sprites) {
//                sprite.setColor(sprite.getColor());
//            }
//        }
//        if (Turret.deadzone) {
//            background.enable();
//            background.setColor(Color.RED);
//        } else {
//            background.disable();
//        }
//    }
//    //-55 -61 -90
//    //-55 -58 -91
//    // 20/20
//    // 20/20
//
//    public void setTeleop(boolean t, Bot bot) {
//        teleop = t;
//        this.bot = bot;
//    }
//
//    @Override
//    protected Sprite[] getSprites() {
//        return this.sprites;
//    }
//
//}
package org.firstinspires.ftc.teamcode.teleop.screen;


import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Intake;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;


import team.techtigers.core.display.Color;
import team.techtigers.core.display.DisplayRegion;
import team.techtigers.core.display.sprites.RectangleSprite;
import team.techtigers.core.display.sprites.Sprite;
import team.techtigers.core.display.sprites.XSprite;

public class BallRegion extends DisplayRegion {
    private final CircleSprite circle1, circle2, circle3;
    private final CircleSprite fault1, fault2, fault3;
    private final RectangleOutlineSprite outline1;
    private final Sprite[] sprites;
    public boolean teleop = false;
    public Bot bot;


    public BallRegion(int x, int y) {
        super(x, y, 24, 8);
        circle1 = new CircleSprite(0, 0, 8);
        circle1.setColor(Color.GREEN);
        circle1.enable();
        circle2 = new CircleSprite(8, 0, 8);
        circle2.setColor(Color.GREEN);
        circle2.enable();
        circle3 = new CircleSprite(16, 0, 8);
        circle3.setColor(Color.GREEN);
        circle3.enable();
        fault1 = new CircleSprite(1, 1, 6);
        fault1.setColor(Color.RED);
        fault1.disable();
        fault2 = new CircleSprite(9, 1, 6);
        fault2.setColor(Color.RED);
        fault2.disable();
        fault3 = new CircleSprite(17, 1, 6);
        fault3.setColor(Color.RED);
        fault3.disable();
        outline1 = new RectangleOutlineSprite(0, 0, 24, 8);
        outline1.setColor(Color.RED);
        outline1.disable();

        sprites = new Sprite[]{circle1, circle2, circle3, fault1, fault2, fault3, outline1};
    }

    @Override
    public void update() {
        if (Turret.deadzone) {
            outline1.enable();
            outline1.setColor(Color.RED);
        } else {
            outline1.disable();
        }

        if (teleop) {
            for (int i = 0; i < 3; i++) {
                Sprite circle = sprites[i];
                CircleSprite fault = i == 0 ? fault1 : (i == 1 ? fault2 : fault3);

                int disabledSensorCount = bot.indexer.holders[i].disabledSensorCount();
                if (disabledSensorCount >= 2) {
                    circle.setColor(Color.RED);
                    fault.disable();
                    continue;
                } else if (disabledSensorCount == 1) {
                    fault.enable();
                } else {
                    fault.disable();
                }

                if (bot.indexer.holders[i].getColor().equals("GREEN")) {
                    circle.setColor(Color.GREEN);
                } else if (bot.indexer.holders[i].getColor().equals("PURPLE")) {
                    circle.setColor(Color.PURPLE);
                } else if (bot.indexer.holders[i].getColor().equals("UNKNOWN")) {
                    circle.setColor(Color.WHITE);
                } else {
                    circle.setColor(Color.BLACK);
                }
            }
        } else {
            fault1.disable();
            fault2.disable();
            fault3.disable();
            for (Sprite sprite : sprites) {
                sprite.setColor(sprite.getColor());
            }
        }
    }

    public void setTeleop(boolean t, Bot bot) {
        teleop = t;
        this.bot = bot;
    }

    @Override
    protected Sprite[] getSprites() {
        return this.sprites;
    }

}
