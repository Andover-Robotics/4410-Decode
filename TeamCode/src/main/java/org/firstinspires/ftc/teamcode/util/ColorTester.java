package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorTester", group = "Utility")
public class ColorTester extends LinearOpMode {

    // Logic sensors
    RevColorSensorV3 colorRR; // RIGHT A
    RevColorSensorV3 colorLR; // LEFT A
    RevColorSensorV3 colorBR; // BACK A (hue only)

    // Distance sensors
    RevColorSensorV3 distRR;  // RIGHT A (same as colorRR)
    RevColorSensorV3 distLR;  // LEFT A (same as colorLR)
    RevColorSensorV3 distBR;  // BACK B (use for distance)

    float[] hsv = new float[3];

    // ===== TUNABLES =====
    static final int gain = 20;
    static final double DISTANCE_THRESHOLD_MM = 25.0;

    @Override
    public void runOpMode() {

        // Color sensors
        colorRR = hardwareMap.get(RevColorSensorV3.class, "colorRR");
        colorLR = hardwareMap.get(RevColorSensorV3.class, "colorLR");
        colorBR = hardwareMap.get(RevColorSensorV3.class, "colorBR");

        // Distance sensors
        distRR = colorRR;             // RIGHT A
        distLR = colorLR;             // LEFT A
        distBR = hardwareMap.get(RevColorSensorV3.class, "colorBL"); // BACK B

        // Apply gain to all sensors
        colorRR.setGain(gain);
        colorLR.setGain(gain);
        colorBR.setGain(gain);
        distBR.setGain(gain); // B sensor gain
        // distRR/distLR already applied via color sensors

        telemetry.addLine("=== COLOR TESTER ===");
        telemetry.addData("Gain", gain);
        telemetry.addData("Distance threshold (mm)", DISTANCE_THRESHOLD_MM);
        telemetry.addLine("Hue-only color + distance presence (Back B for distance)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            debugSpot("RIGHT", colorRR, distRR);
            debugSpot("LEFT",  colorLR, distLR);
            debugSpot("BACK",  colorBR, distBR);

            telemetry.addLine("--------------------------------");
            telemetry.update();
        }
    }

    /** Hue + distance (distance can be different sensor) */
    private void debugSpot(String name, RevColorSensorV3 colorSensor, RevColorSensorV3 distanceSensor) {

        double distance = safeDistance(distanceSensor);
        boolean ballPresent = distance > 0 && distance < DISTANCE_THRESHOLD_MM;

        telemetry.addLine("[" + name + "]");
        telemetry.addData(name + " distance (mm)", "%.1f", distance);
        telemetry.addData(name + " ball present?", ballPresent);

        if (!ballPresent) {
            telemetry.addData(name + " FINAL", "EMPTY");
            return;
        }

        // HSV (colorSensor only)
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
        float h = hsv[0];

        telemetry.addData(name + " Hue", "%.1f", h);

        boolean green = isGreen(h, 0, 0);
        boolean purple = isPurple(h, 0, 0);

        telemetry.addData(name + " green?", green);
        telemetry.addData(name + " purple?", purple);

        String finalColor = "UNKNOWN";
        if (green) finalColor = "GREEN";
        else if (purple) finalColor = "PURPLE";

        telemetry.addData(name + " FINAL", finalColor);
    }

    /** Safe distance */
    private double safeDistance(RevColorSensorV3 sensor) {
        double d = sensor.getDistance(DistanceUnit.MM);
        if (Double.isNaN(d) || Double.isInfinite(d)) {
            return -1;
        }
        return d;
    }

    /** Hue-only logic */
    private boolean isGreen(float h, float s, float v) {
        return h > 160 && h < 180;
    }

    private boolean isPurple(float h, float s, float v) {
        return h > 180 && h < 225;
    }
}
