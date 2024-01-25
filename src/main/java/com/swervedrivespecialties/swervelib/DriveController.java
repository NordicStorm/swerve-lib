package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    void setReferenceVelocity(double velocity);
    
    double getStateVelocity();

    double getSensorPosition();

    TalonFX getTalonFX();

    CANSparkMax getSparkMax();

}
