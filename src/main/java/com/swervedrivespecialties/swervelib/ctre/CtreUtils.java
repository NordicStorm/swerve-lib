package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.StatusCode;

public final class CtreUtils {
    private CtreUtils() {
    }

    public static void checkCtreError(StatusCode errorCode, String message) {
        if (errorCode.isError()) {
            throw new RuntimeException(String.format("%s: %s", message, errorCode.toString()));

            ///DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
        }
    }
}
