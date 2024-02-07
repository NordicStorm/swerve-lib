package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public final class Falcon500DriveControllerFactoryBuilder {
    private double currentLimit = Double.NaN;

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction();
            double sensorVelocityCoefficient = sensorPositionCoefficient;

            
            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }

            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            motorConfiguration.MotorOutput.Inverted = moduleConfiguration.isDriveInverted() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;

            TalonFX motor = new TalonFX(driveConfiguration);
            CtreUtils.checkCtreError(motor.getVelocity().setUpdateFrequency(50), "Failed to configure Falcon 500 velocity frame. id="+driveConfiguration);
            CtreUtils.checkCtreError(motor.getPosition().setUpdateFrequency(50), "Failed to configure Falcon 500 position frame. id="+driveConfiguration);
           // CtreUtils.checkCtreError(motor.optimizeBusUtilization(), "Failed to optimize Falcon 500 status frame. id="+driveConfiguration);
            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to configure Falcon 500. id="+driveConfiguration);

            return new ControllerImplementation(motor, sensorVelocityCoefficient, sensorPositionCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double sensorPositionCoefficient;
        private final VoltageOut voltageRequest = new VoltageOut(0);
        private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient, double sensorPositionCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.sensorPositionCoefficient = sensorPositionCoefficient;
            velocityRequest.Slot = 0;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setControl(voltageRequest.withOutput(voltage));
        }

        @Override
        public void setReferenceVelocity(double velocity) {
            motor.setControl(velocityRequest.withVelocity(velocity / sensorVelocityCoefficient));
        }

        @Override
        public double getStateVelocity() {
            return motor.getVelocity().getValue() * sensorVelocityCoefficient;
        }
        @Override
        public double getSensorPosition() {
            return motor.getPosition().getValue() * sensorPositionCoefficient;
        }
        @Override
        public TalonFX getTalonFX() {
            return motor;
        }
        @Override
        public CANSparkMax getSparkMax() {
            throw new UnsupportedOperationException("not a spark drive controller");
        }
    }
}
