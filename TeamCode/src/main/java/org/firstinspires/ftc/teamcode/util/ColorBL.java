
package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Reads HSV values from a Brushland Labs Color Rangefinder
 * configured as a REV Color Sensor V3.
 */
@TeleOp(name = "Brushland CRF HSV Test", group = "Sensor Test")
public class ColorBL extends LinearOpMode {

    private final float[] hsvValues = new float[3];

    @Override
    public void runOpMode() {
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");

        telemetry.addLine("Ready.");
        telemetry.addLine("Showing RGB, HSV, and distance.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();

            // Convert RGB -> HSV
            Color.RGBToHSV(red, green, blue, hsvValues);

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);

            telemetry.addData("Hue", "%.2f", hsvValues[0]);        // 0 to 360
            telemetry.addData("Saturation", "%.4f", hsvValues[1]); // 0 to 1
            telemetry.addData("Value", "%.4f", hsvValues[2]);      // 0 to 1

            telemetry.addData(
                    "Distance (cm)",
                    "%.2f",
                    colorSensor.getDistance(
                            org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM
                    )
            );

            telemetry.update();
        }
    }
}