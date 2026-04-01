package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;


import java.util.function.BooleanSupplier;

public class CommandClimber implements Subsystem{
    private TalonFX climberMotor;
    private TalonFXConfiguration climbConfig;

    // Define a constant for zero angular velocity to compare against
    public static final AngularVelocity kZero = RotationsPerSecond.of(0);

    public CommandClimber(){
        // Initialize the TalonFX motor controller for the climber
        climberMotor = new TalonFX(8);

        climbConfig.withSlot0(
            new Slot0Configs().withKP(0).withKD(0)
            .withKG(0).withGravityType(GravityTypeValue.Elevator_Static))
            .withSlot1(
                new Slot1Configs().withKP(0).withKD(0).withKG(0)
                .withGravityType(GravityTypeValue.Elevator_Static))
                .withCurrentLimits(
                    new CurrentLimitsConfigs().withStatorCurrentLimit(60))
                    .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(48)); //TODO: Tune these

        climberMotor.getConfigurator().apply(climbConfig);
    }

    // Method to check if the motor is stopped by comparing its velocity to zero
    private BooleanSupplier isMotorStopped(boolean not) {
        return () -> {
            if (not == true) {
                return climberMotor.getVelocity().getValue() == kZero;
            } else {
                return climberMotor.getVelocity().getValue() != kZero;
            }
        };
    }

    // Method to check if the motor is stalled by comparing its current to the stall threshold
    private BooleanSupplier isMotorStalled(boolean not) {
        return () -> {
            if (not == true) {
                return climberMotor.getTorqueCurrent().getValueAsDouble() >= climbConfig.CurrentLimits.StatorCurrentLimit;
            } else {
                return climberMotor.getTorqueCurrent().getValueAsDouble() < climbConfig.CurrentLimits.StatorCurrentLimit;
            }
        };
    }

    // Command to move the climber up, which runs the motor at 50% power unless the motor is stopped
    public Command climberUp() {
        return run(() -> climberMotor.set(0.5)).onlyWhile(isMotorStalled(false)).finallyDo(() -> climberMotor.setControl(new StaticBrake()));
    }

    // Command to move the climber down, which runs the motor at 50% power unless the motor is stopped
    public Command climberDown() {
        return run(() -> climberMotor.set(-0.5)).onlyWhile(isMotorStalled(false)).finallyDo(() -> climberMotor.setControl(new StaticBrake()));
    }

    public Command homeClimber() {
        // Command to home the climber, which runs the motor at -50% power until the motor is stalled
        return run(() -> climberMotor.set(-0.5)).until(isMotorStalled(false)).finallyDo(() -> climberMotor.setPosition(0));
    }
}
