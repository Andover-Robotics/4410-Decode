package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class Lights {
    Prism prism;

    public void init() {
        // Initialize Prism driver on your hardware map
        prism = hardwareMap.get(Prism.class, "prism");
    }

    public void setupAnimation() {
        // Create an artboard (animation container)
        PrismArtboard artboard = new PrismArtboard();

        // Base layer - black initially
        PrismLayer baseLayer = artboard.createLayer();
        baseLayer.fillColor(0, 0, 0); // RGB black

        // Green circle layer
        PrismLayer greenCircle = artboard.createLayer();
        greenCircle.drawCircle(3, 3, 3, 0, 255, 0); // center x=3, y=3, radius=3

        // Purple circle layer
        PrismLayer purpleCircle = artboard.createLayer();
        purpleCircle.drawCircle(3, 3, 3, 128, 0, 128); // RGB purple

        // Optional top layer
        PrismLayer overlay = artboard.createLayer();
        overlay.fillColor(0, 0, 0); // nothing for now

        // Create animation with artboard
        PrismAnimation animation = new PrismAnimation(artboard, 1); // 1 frame for static effect

        // Set animation on Prism
        prism.setAnimation(animation);
    }
}
