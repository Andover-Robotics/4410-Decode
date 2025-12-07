package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

public class LightsAnimation {
    private final GoBildaPrismDriver prism;
    private final PrismAnimations.BackgroundLayer background;
    private final PrismAnimations.CircleLayer[] circles = new PrismAnimations.CircleLayer[4];

    private final ElapsedTime teleopTimer = new ElapsedTime();
    private boolean teleopStarted = false;

    public LightsAnimation(GoBildaPrismDriver prism) {
        this.prism = prism;

        prism.clearAllAnimations();
        prism.setStripLength(192);
        prism.setTargetFPS(30);

        // Background
        background = new PrismAnimations.BackgroundLayer(0x000000);

        // Circles
        circles[0] = new PrismAnimations.CircleLayer(4, 4, 3, 4, 0x303030, 0xFFFFFF);
        circles[1] = new PrismAnimations.CircleLayer(10, 4, 3, 4, 0x303030, 0xFFFFFF);
        circles[2] = new PrismAnimations.CircleLayer(16, 4, 3, 4, 0x303030, 0xFFFFFF);
        circles[3] = new PrismAnimations.CircleLayer(22, 4, 3, 4, 0x303030, 0xFFFFFF);

        // Load animations into layers
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, background);

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, circles[0]);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, circles[1]);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, circles[2]);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, circles[3]);
    }

    public void startTeleopTimer() {
        teleopTimer.reset();
        teleopStarted = true;
    }

    public void periodic(Bot bot) {
        int bg = 0x000000;

        boolean llValid = false;
        boolean climb = false;

        try {
            if (bot != null && bot.turret != null && bot.turret.limelight != null) {
                llValid = bot.turret.limelight.getLatestResult() != null &&
                        bot.turret.limelight.getLatestResult().isValid();
            }
        } catch (Exception ignored) {}

        if (bot != null && bot.lift != null) {
            climb = bot.lift.isClosedLoopEnabled();
        }

        if (llValid) {
            bg = 0x202000;
        }

        if (climb) {
            if (bot != null && bot.getAlliance() == Bot.allianceOptions.RED_ALLIANCE) {
                bg = 0x201000;
            } else {
                bg = 0x001020;
            }
        }

        if (teleopStarted) {
            double t = teleopTimer.seconds();
            if (t >= 95) {
                if (((int)(t * 4)) % 2 == 0) {
                    bg = 0x000000;
                }
            }
        }

        background.setColor(bg);
        prism.updateAllAnimations();
    }
}