package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class Hopper implements Subsystem {
    // TODO: Get real CAN IDs for the motors
    private final CANBus kCANBus = Constants.getRIOCANBus();
    private final TalonFX m_hopperMotor = new TalonFX(0, kCANBus);
    private final TalonFXS kickerTop = new TalonFXS(0, kCANBus);
    private final TalonFXS kickerBottom = new TalonFXS(0, kCANBus);

    // TODO: Get real PID values for the hopper and kicker motors, and find the correct positive direction for the motors
    private final TalonFXConfiguration hopperMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new com.ctre.phoenix6.configs.MotorOutputConfigs()
                    .withInverted(com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive))
            .withSlot0(new com.ctre.phoenix6.configs.Slot0Configs()
                    .withKP(5))
            .withCurrentLimits(new com.ctre.phoenix6.configs.CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(55));
    
    private final TalonFXSConfiguration kickerTopConfig = new TalonFXSConfiguration()
            .withMotorOutput(new com.ctre.phoenix6.configs.MotorOutputConfigs()
                    .withInverted(com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive))
            .withSlot0(new com.ctre.phoenix6.configs.Slot0Configs()
                    .withKP(5))
            .withCurrentLimits(new com.ctre.phoenix6.configs.CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(55));

    private final TalonFXSConfiguration kickerBottomConfig = new TalonFXSConfiguration()
            .withMotorOutput(new com.ctre.phoenix6.configs.MotorOutputConfigs()
                    .withInverted(com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive))
            .withSlot0(new com.ctre.phoenix6.configs.Slot0Configs()
                    .withKP(5))
            .withCurrentLimits(new com.ctre.phoenix6.configs.CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(55));
    
    public Hopper() {
        m_hopperMotor.getConfigurator().apply(hopperMotorConfig);
        kickerTop.getConfigurator().apply(kickerTopConfig);
        kickerBottom.getConfigurator().apply(kickerBottomConfig);
    }

    /**
     * Set the speed of the hopper motor
     * @param speed of the main hopper motor, from -1.0 to 1.0
     * @return a command that sets the speed of the hopper motor
     */
    public Command setHopperSpeed(double speed) {
        return Commands.run(() -> m_hopperMotor.set(speed), this);
    }

    /**
     * Set the speed of the kicker motors
     * @param speed of the kicker motors, from -1.0 to 1.0
     * @return a command that sets the speed of the kicker motors
     */
    public Command setKickerSpeed(double speed) {
        return Commands.run(() -> {
            kickerTop.set(speed);
            kickerBottom.set(speed);
        }, this);
    }

    public Command setHopperAndKickerSpeed(double hopperSpeed, double kickerSpeed) {
        return Commands.run(() -> {
            m_hopperMotor.set(hopperSpeed);
            kickerTop.set(kickerSpeed);
            kickerBottom.set(kickerSpeed);
        }, this);
    }

    /**
     * Stop the hopper and kicker motors
     * @return a command that stops the hopper and kicker motors
     */
    public Command stopHopper() {
        return Commands.run(() -> {
            m_hopperMotor.set(0);
            kickerTop.set(0);
            kickerBottom.set(0);
        }, this);
    }
}