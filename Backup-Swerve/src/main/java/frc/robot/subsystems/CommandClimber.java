package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

public class CommandClimber implements Subsystem{
    private TalonFX climberMotor;

    // Define a constant for zero angular velocity to compare against
    public static final AngularVelocity kZero = RotationsPerSecond.of(0);

    public CommandClimber(){
        // Initialize the TalonFX motor controller for the climber
        climberMotor = new TalonFX(8);
    }

    // Method to check if the motor is stopped by comparing its velocity to zero
    private BooleanSupplier isMotorStopped() {
        return () -> {return climberMotor.getVelocity().getValue() == kZero;};
    }

    // Command to move the climber up, which runs the motor at 50% power unless the motor is stopped
    public Command climberUp() {
        return run(() -> climberMotor.set(0.5)).unless(isMotorStopped());
    }

    // Command to move the climber down, which runs the motor at 50% power unless the motor is stopped
    public Command climberDown() {
        return run(() -> climberMotor.set(-0.5)).unless(isMotorStopped());
    }
}
