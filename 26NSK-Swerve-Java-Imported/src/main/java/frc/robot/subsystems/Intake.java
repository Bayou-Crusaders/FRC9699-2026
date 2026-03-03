package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Intake implements Subsystem{
    // Initialize the objects and variables
    TalonFX topIntake;
    TalonFX bottomIntake;
    TalonFX pitchMotor;
    CANBus kCANBus;
    Slot0Configs pitchGains;

    public Intake(int topIntakeCAN, int bottomIntakeCAN, int pitchCAN) {
        kCANBus = new CANBus("", "./logs/example.hoot");
        topIntake = new TalonFX(topIntakeCAN, kCANBus);
        bottomIntake = new TalonFX(bottomIntakeCAN, kCANBus);
        pitchMotor = new TalonFX(pitchCAN, kCANBus);

        // Make the right flywheel follow the left flywheel
        bottomIntake.setControl(
            new Follower(
                topIntakeCAN, MotorAlignmentValue.Opposed
            )
        );

        pitchGains = new Slot0Configs() // For more information on what this is see: 
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html
        //TODO: Tune pitch gains
        .withKP(0) // The proportional gain
        .withKI(0) // The integral gain
        .withKD(0) // The derivative gain
        .withKS(0) // The static gain
        .withKV(0) // The velocity gain
        .withKA(0) // The acceleration gain
        .withKG(0); // The gravity gain

        pitchMotor.getConfigurator().apply(pitchGains);
    }

    public Command setIntakeSpeed(double dutyCycle) { // Set the intake speed in percentage of max voltage
        return Commands.runOnce(
            () -> topIntake.setControl(
                new DutyCycleOut(
                    dutyCycle
                )
            )
        );
    }

     public Command setPitchAngle(double angle) { // Set the pitch angle in degrees
        return Commands.runOnce(
            () -> pitchMotor.setControl(
                new PositionVoltage(
                    Units.degreesToRotations( // Convert the angle from degrees to rotations
                        angle
                    )
                )
            )
        );
    }
    
}
