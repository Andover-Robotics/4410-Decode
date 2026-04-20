package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Method;

/**
 * Photon bootstrap helper based on Photon usage docs: enable once at startup,
 * then use enabled state to tune hub bulk-caching behavior.
 */
public final class PhotonBootstrap {
    private static final String TAG = "PhotonBootstrap";
    private static boolean initialized = false;
    private static boolean enabled = false;

    private PhotonBootstrap() {
    }

    public static synchronized boolean initialize() {
        if (initialized) {
            return enabled;
        }

        initialized = true;

        enabled = tryEnable("com.seattlesolvers.solverslib.photoncore.PhotonCore")
                || tryEnable("com.outoftheboxrobotics.photoncore.PhotonCore");

        if (!enabled) {
            RobotLog.ww(TAG, "PhotonCore not found on classpath; using standard FTC SDK path.");
        }

        return enabled;
    }

    public static boolean isEnabled() {
        return enabled;
    }

    private static boolean tryEnable(String className) {
        try {
            Class<?> photonCoreClass = Class.forName(className);
            Method enableMethod = photonCoreClass.getMethod("enable");
            enableMethod.invoke(null);
            RobotLog.ii(TAG, "Photon enabled with %s", className);
            return true;
        } catch (ReflectiveOperationException ignored) {
            return false;
        }
    }
}
