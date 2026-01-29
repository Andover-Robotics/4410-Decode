package org.firstinspires.ftc.teamcode.auto.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class WaitUntilAction implements Action {
    public interface ConditionalFunction {
        boolean condition();
    }

    private ConditionalFunction f;
    private double minTime;
    private double maxTime;
    private double beginTs = -1.0;

    public WaitUntilAction(ConditionalFunction f, double minTime, double maxTime) {
        this.f = f;
        this.minTime = minTime;
        this.maxTime = maxTime;
    }

    public WaitUntilAction(ConditionalFunction f) {
        this(f, 0.0, Double.POSITIVE_INFINITY);
    }

    private static double getTimeSeconds() {
        return System.currentTimeMillis() / 1000.0;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        double deltaTs = 0;
        if (beginTs < 0) {
            beginTs = getTimeSeconds();
        } else {
            deltaTs = getTimeSeconds() - beginTs;
        }

        if (deltaTs < minTime) return true;
        if (deltaTs > maxTime) return false;

        return !f.condition();
    }
}