package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;

/**
 * The Intake subsystem controls the intake mechanism of the robot, which
 * consists of an intake roller
 * and two intake arms.
 */
public class Intake implements Subsystem {
    // TODO: Get real CAN IDs for the motors
    private final CANBus kCANBus = Constants.getRIOCANBus();
    private final TalonFXS intakeRoller = new TalonFXS(0, kCANBus);
    private final TalonFXS intakeArmLeft = new TalonFXS(0, kCANBus);
    private final TalonFXS intakeArmRight = new TalonFXS(0, kCANBus);

    // TODO: Get real PID values for the intake motors
    private final TalonFXSConfiguration intakeRollerConfig = new TalonFXSConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)) // TODO: Check the positive direction
            .withSlot0(new Slot0Configs()
                    .withKP(5))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40));

    private final TalonFXSConfiguration intakeArmLeftConfig = new TalonFXSConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)) // TODO: Check the positive direction
            .withSlot0(new Slot0Configs()
                    .withKP(5))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(60));

    private final TalonFXSConfiguration intakeArmRightConfig = new TalonFXSConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)) // TODO: Check the positive direction
            .withSlot0(new Slot0Configs()
                    .withKP(5))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(60));

    public Intake() {
        intakeRoller.getConfigurator().apply(intakeRollerConfig);
        intakeArmLeft.getConfigurator().apply(intakeArmLeftConfig);
        intakeArmRight.getConfigurator().apply(intakeArmRightConfig);
    }

    /**
     * Set the pitch of the intake arm to the given pitch value in rotations
     * 
     * @param pitch the desired pitch of the intake arm in rotations
     * @return a command that sets the pitch of the intake arm to the given pitch
     *         value
     */
    public Command setPitch(double pitch) {
        return Commands.runOnce(() -> {
            intakeArmLeft.setControl(new PositionVoltage(pitch));
            intakeArmRight.setControl(new PositionVoltage(pitch));
        },
                this);
    }

    /**
     * Set the speed of the intake roller to the given speed value in rotations per
     * minute
     * 
     * @param speed the desired speed of the intake roller in rotations per minute
     * @return a command that sets the speed of the intake roller to the given speed
     *         value
     */
    public Command setRollerSpeed(double speed) {
        return Commands.runOnce(() -> {
            intakeRoller.setControl(new PositionVoltage(speed));
        },
                this);
    }

    /**
     * Set the pitch of the intake arm and the speed of the intake roller at the
     * same time
     * 
     * @param pitch the desired pitch of the intake arm in rotations
     * @param speed the desired speed of the intake roller in rotations per minute
     * @return a command that sets both the pitch and speed
     */
    public Command setPitchAndSpeed(double pitch, double speed) {
        return Commands.runOnce(() -> {
            intakeRoller.setControl(new VelocityVoltage(speed));
            intakeArmLeft.setControl(new PositionVoltage(pitch));
            intakeArmRight.setControl(new PositionVoltage(pitch));
        },
                this);
    }

    /**
     * Stop the intake roller
     * 
     * @return a command that stops the intake roller
     */
    public Command stopRoller() {
        return Commands.runOnce(() -> {
            intakeRoller.setControl(new CoastOut());
        },
                this);
    }

    /**
     * Stop the intake arms
     * 
     * @return a command that stops the intake arms
     */
    public Command stopArms() {
        return Commands.runOnce(() -> {
            intakeArmLeft.setControl(new CoastOut());
            intakeArmRight.setControl(new CoastOut());
        },
                this);
    }

}