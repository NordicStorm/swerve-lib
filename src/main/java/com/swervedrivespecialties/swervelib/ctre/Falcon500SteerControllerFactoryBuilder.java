package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.swervedrivespecialties.swervelib.*;


import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static com.swervedrivespecialties.swervelib.ctre.CtreUtils.checkCtreError;

public final class Falcon500SteerControllerFactoryBuilder {

    // PID configuration
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public Falcon500SteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.proportionalConstant = proportional;
        this.integralConstant = integral;
        this.derivativeConstant = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public Falcon500SteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public Falcon500SteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public <T> SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> build(AbsoluteEncoderFactory<T> absoluteEncoderFactory) {
        return new FactoryImplementation<>(absoluteEncoderFactory);
    }

    private class FactoryImplementation<T> implements SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;

        private FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }

        @Override
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            SteerControllerFactory.super.addDashboardEntries(container, controller);
            if(container!=null){
                container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle()));
            }
        }

        @Override
        public ControllerImplementation create(Falcon500SteerConfiguration<T> steerConfiguration, ModuleConfiguration moduleConfiguration) {
            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());

            // multiply by this to convert motor rotations to radians
            final double sensorPositionCoefficient = 2.0 * Math.PI  * moduleConfiguration.getSteerReduction();

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            if (hasPidConstants()) {
                motorConfiguration.Slot0.kP = proportionalConstant;
                motorConfiguration.Slot0.kI = integralConstant;
                motorConfiguration.Slot0.kD = derivativeConstant;
            }
            
            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }
            int port = steerConfiguration.getMotorPort();
            TalonFX motor = new TalonFX(port);
            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            motorConfiguration.MotorOutput.Inverted = moduleConfiguration.isSteerInverted() ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;

            checkCtreError(motor.getVelocity().setUpdateFrequency(50), "Failed to configure Falcon 500 velocity frame. id="+port);
            checkCtreError(motor.getPosition().setUpdateFrequency(50), "Failed to configure Falcon 500 position frame. id="+port);
            checkCtreError(motor.optimizeBusUtilization(), "Failed to optimize Falcon 500 status frame. id="+port);
            checkCtreError(motor.getConfigurator().apply(motorConfiguration, 1), "Failed to configure Falcon 500 settings. id="+port);
            checkCtreError(motor.setPosition(absoluteEncoder.getAbsoluteAngle() / sensorPositionCoefficient), "Failed to set Falcon 500 encoder position. id="+port);

            return new ControllerImplementation(motor,
                    sensorPositionCoefficient,
                    absoluteEncoder);
        }
    }

    private static class ControllerImplementation implements SteerController {
        private final TalonFX motor;
        private final double motorEncoderPositionCoefficient;
        private final AbsoluteEncoder absoluteEncoder;
        private final PositionVoltage positionRequest = new PositionVoltage(0);
        private double referenceAngleRadians = 0.0;

        private ControllerImplementation(TalonFX motor,
                                         double motorEncoderPositionCoefficient,
                                         AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.motorEncoderPositionCoefficient = motorEncoderPositionCoefficient;
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motor.getPosition().getValue() * motorEncoderPositionCoefficient;
            
            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }
            motor.setControl(positionRequest.withPosition(adjustedReferenceAngleRadians / motorEncoderPositionCoefficient));
            this.referenceAngleRadians = referenceAngleRadians;
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motor.getPosition().getValue() * motorEncoderPositionCoefficient;
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }
    }
}
