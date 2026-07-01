package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

// TODO: Get real CAN IDs and PID values for the shooter motors, and tune the PID values for the pitch motor

public class CommandShooter implements Subsystem {
    private CANBus kCANBus = Constants.getRIOCANBus();
    private TalonFX m_rightFly = new TalonFX(0, kCANBus);
    private TalonFX m_leftFly = new TalonFX(0, kCANBus);
    private TalonFX m_pitch = new TalonFX(0, kCANBus);

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

    // TODO: Find RotorOffset, SensorToMechanismRatio, and SoftwareLimitSwitch configs for the pitch motor
    private TalonFXConfiguration pitchConfig = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKP(5))
        .withFeedback(new FeedbackConfigs()
            .withFeedbackRotorOffset(0)
            .withSensorToMechanismRatio(0))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0));

    public CommandShooter(){
        m_rightFly.getConfigurator().apply(rightFlyConfig);
        m_leftFly.getConfigurator().apply(leftFlyConfig);
        m_pitch.getConfigurator().apply(pitchConfig);
    }

    /** Set the speed of the flywheels in RPM
     * @param speed the desired speed of the flywheels in Rotations per Minute
     */
    public Command setSpeed(double speed){
        return Commands.run(
            ()->{
                m_rightFly.setControl(new VelocityVoltage(speed).withSlot(0)); 
                m_leftFly.setControl(new VelocityVoltage(speed).withSlot(0));
            }, this
        );
    }

    /**
     * Set the pitch of the shooter
     * @param pitch the desired pitch of the shooter in Rotations
     */
    public Command setPitch(double pitch){
        return Commands.run(
            ()->m_pitch.setControl(new PositionVoltage(pitch).withSlot(0)), 
            this
        );
    }

    /**
     * Set the speed of the flywheels in RPM and the pitch of the shooter at the same time
     * @param speed the desired speed of the flywheels in Rotations per Minute
     * @param pitch the desired pitch of the shooter in Rotations
     */
    public Command setSpeedAndPitch(double speed, double pitch){
        return Commands.run(
            ()->{
                m_rightFly.setControl(new VelocityVoltage(speed).withSlot(0)); 
                m_leftFly.setControl(new VelocityVoltage(speed).withSlot(0));
                m_pitch.setControl(new PositionVoltage(pitch).withSlot(0));
            }, this
        );
    }
    
    /**
     * Stop the shooter by setting the flywheel speeds to 0 and keeping the pitch at its current position or changing it to a known safe position
     */
    public Command stop() {
        return stop(m_pitch.getPosition().getValueAsDouble());
    }

    public Command stop(double pitch) {
        return Commands.run(
            () -> {
                m_rightFly.setControl(new VelocityVoltage(0));
                m_leftFly.setControl(new VelocityVoltage(0));
                m_pitch.setControl(new PositionVoltage(pitch).withSlot(0));
            }, this
        );
    }
}