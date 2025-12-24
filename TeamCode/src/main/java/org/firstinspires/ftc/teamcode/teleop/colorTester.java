package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "Ball Detection DEBUG MAX")
public class colorTester extends LinearOpMode {

    RevColorSensorV3 colorRR, colorRL;
    RevColorSensorV3 colorLR, colorLL;
    RevColorSensorV3 colorBR, colorBL;

    float[] hsvA = new float[3];
    float[] hsvB = new float[3];

    static final double DISTANCE_THRESHOLD_MM = 50.0; // tune this

    @Override
    public void runOpMode() {

        colorRR = hardwareMap.get(RevColorSensorV3.class, "colorRR");
        colorRL = hardwareMap.get(RevColorSensorV3.class, "colorRL");

        colorLR = hardwareMap.get(RevColorSensorV3.class, "colorLR");
        colorLL = hardwareMap.get(RevColorSensorV3.class, "colorLL");

        colorBR = hardwareMap.get(RevColorSensorV3.class, "colorBR");
        colorBL = hardwareMap.get(RevColorSensorV3.class, "colorBL");

        telemetry.addLine("DEBUG MODE: DIABOLICAL TELEMETRY ENABLED");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            debugSpot("RIGHT", colorRR, colorRL);
            debugSpot("LEFT",  colorLR, colorLL);
            debugSpot("BACK",  colorBR, colorBL);

            telemetry.addLine("--------------------------------");
            telemetry.update();
        }
    }

    private void debugSpot(String name, RevColorSensorV3 a, RevColorSensorV3 b) {

        double distA = a.getDistance(DistanceUnit.MM);
        double distB = b.getDistance(DistanceUnit.MM);
        double avgDist = (distA + distB) / 2.0;

        boolean ballPresent = avgDist < DISTANCE_THRESHOLD_MM;

        telemetry.addLine("[" + name + "]");
        telemetry.addData(name + " dist A (mm)", "%.1f", distA);
        telemetry.addData(name + " dist B (mm)", "%.1f", distB);
        telemetry.addData(name + " dist AVG (mm)", "%.1f", avgDist);
        telemetry.addData(name + " ball present?", ballPresent);

        if (!ballPresent) {
            telemetry.addData(name + " FINAL", "EMPTY");
            return;
        }

        // RGB
        telemetry.addData(name + " A RGB",
                "%d %d %d", a.red(), a.green(), a.blue());
        telemetry.addData(name + " B RGB",
                "%d %d %d", b.red(), b.green(), b.blue());

        // HSV
        Color.RGBToHSV(a.red(), a.green(), a.blue(), hsvA);
        Color.RGBToHSV(b.red(), b.green(), b.blue(), hsvB);

        float avgHue = (hsvA[0] + hsvB[0]) / 2;
        float avgSat = (hsvA[1] + hsvB[1]) / 2;
        float avgVal = (hsvA[2] + hsvB[2]) / 2;

        telemetry.addData(name + " AVG HSV",
                "H %.1f S %.2f V %.2f", avgHue, avgSat, avgVal);

        boolean green = isGreen(avgHue, avgSat, avgVal);
        boolean purple = isPurple(avgHue, avgSat, avgVal);

        telemetry.addData(name + " green check", green);
        telemetry.addData(name + " purple check", purple);

        String finalColor = "UNKNOWN";
        if (green) finalColor = "GREEN";
        else if (purple) finalColor = "PURPLE";

        telemetry.addData(name + " FINAL", finalColor);
    }

    private boolean isGreen(float h, float s, float v) {
        return h > 90 && h < 150 && s > 0.4 && v > 0.2;
    }

    private boolean isPurple(float h, float s, float v) {
        return h > 250 && h < 300 && s > 0.4 && v > 0.2;
    }
}

