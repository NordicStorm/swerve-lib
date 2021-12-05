package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;



public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void setWithVelocity(double driveVelocity, double steerAngle);

    TalonFX getTalonDriveMotor();

    CANSparkMax getSparkDriveMotor();
}
