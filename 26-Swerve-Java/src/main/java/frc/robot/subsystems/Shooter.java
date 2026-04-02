package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class Shooter implements Subsystem {
    // Initialize the objects and variables
    TalonFX rightFlywheel;
    TalonFX leftFlywheel;
    TalonFX pitchMotor;
    CANBus kCANBus;
    Slot0Configs shooterGains;
    Slot0Configs pitchGains;


    public Shooter(int rightFlywheelCAN, int leftFlywheelCAN, int pitchCAN) {
        kCANBus = new CANBus("", "./logs/example.hoot");
        rightFlywheel = new TalonFX(rightFlywheelCAN, kCANBus);
        leftFlywheel = new TalonFX(leftFlywheelCAN, kCANBus);
        pitchMotor = new TalonFX(pitchCAN, kCANBus);

        // Make the right flywheel follow the left flywheel
        rightFlywheel.setControl(
            new Follower(
                leftFlywheelCAN, MotorAlignmentValue.Opposed
            )
        );

        shooterGains = new Slot0Configs() // For more information on what this is see: 
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html
        //TODO: Tune flywheel gains
        .withKP(0) // The proportional gain
        .withKI(0) // The integral gain
        .withKD(0) // The derivative gain
        .withKS(0) // The static gain
        .withKV(0) // The velocity gain
        .withKA(0); // The acceleration gain

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

        leftFlywheel.getConfigurator().apply(shooterGains);
        pitchMotor.getConfigurator().apply(pitchGains);
    }

    public Command setFlywheelRPM(double rpm) { // Set the flywheel speed in RPM
        return Commands.runOnce(
            () -> {
                leftFlywheel.setControl(
                    new VelocityVoltage(
                        rpm / 60.0 // Convert RPM to RPS (rotations per second)
                    )
                );
            },
            this // Requires {@link this}
        );
    }

    public Command setPitchAngle(double angleDegrees) { // Set the pitch angle in degrees
        return Commands.runOnce(
            () -> {
                pitchMotor.setControl(
                    new PositionVoltage(
                        Units.degreesToRotations(angleDegrees) // Convert degrees to rotations
                    )
                );
            },
            this // Requires {@link this}
        );
    }

}