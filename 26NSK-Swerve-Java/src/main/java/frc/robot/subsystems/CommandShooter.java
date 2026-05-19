package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandShooter implements Subsystem {
    private CANBus canBus = new CANBus("rio");
    private TalonFX m_rightFly = new TalonFX(0, canBus);
    private TalonFX m_leftFly = new TalonFX(0, canBus);
    private TalonFX m_pitch = new TalonFX(0, canBus);

    private TalonFXConfiguration rightFlyConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive))
        .withSlot0(new Slot0Configs()
            .withKP(100))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60));

    private TalonFXConfiguration leftFlyConfig = rightFlyConfig
        .clone()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive));

    private TalonFXConfiguration pitchConfig = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(5));

    public CommandShooter(){
        m_rightFly.getConfigurator().apply(rightFlyConfig);
        m_leftFly.getConfigurator().apply(leftFlyConfig);
        m_pitch.getConfigurator().apply(pitchConfig);
    }

    public Command setSpeed(double speed){
        return Commands.run(
            ()->{
                m_rightFly.setControl(
                    new VelocityVoltage(speed).withSlot(0)
                ); 
                m_leftFly.setControl(
                    new VelocityVoltage(speed).withSlot(0)
                );
            }, this
        );
    }

    public Command setPitch(double pitch){
        return Commands.run(
            ()->{
                m_pitch.setControl(new PositionVoltage(pitch).withSlot(0));
            }, this
        );
    }

    public Command setSpeedAndPitch(double speed, double pitch){
        return Commands.sequence(setPitch(pitch), setSpeed(speed));
    }
}