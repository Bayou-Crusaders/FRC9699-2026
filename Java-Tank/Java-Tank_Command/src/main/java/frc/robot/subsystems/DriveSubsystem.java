package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import edu.wpi.first.math.util.Units; // Provides functions to convert units
import edu.wpi.first.math.controller.SimpleMotorFeedforward; // Provides a Class to compute Feedforwards for a simple motor setup
import edu.wpi.first.math.geometry.Pose2d; // Provides a Class to use poses (sets of cooridinates) on a 2d plane
import edu.wpi.first.math.geometry.Rotation2d; // 
import edu.wpi.first.math.kinematics.ChassisSpeeds; // 
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics; // 
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds; // 
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog; // 
import edu.wpi.first.wpilibj2.command.Command; // 
import edu.wpi.first.wpilibj2.command.Commands; // 
import edu.wpi.first.wpilibj2.command.SubsystemBase; // 
import edu.wpi.first.wpilibj2.command.sysid.*; // 
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*; // 
import edu.wpi.first.units.measure.Voltage; // 

import com.revrobotics.spark.*; // 
import com.revrobotics.spark.SparkBase.ControlType; // 
import com.revrobotics.spark.SparkBase.PersistMode; // 
import com.revrobotics.spark.SparkBase.ResetMode; // 
import com.revrobotics.spark.SparkLowLevel.MotorType; // 
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor; // 
import com.revrobotics.spark.config.SparkMaxConfig; // 
import com.revrobotics.AbsoluteEncoder; // 

/**
 * Class that can interface with a differental drivechain
 */
public class DriveSubsystem extends SubsystemBase {

  //Initilaze variables
  private int krightLeadId;
  private int krightFollowId;
  private int kleftLeadId;
  private int kleftFollowId;
  private SparkMax m_rightLead;
  private SparkMax m_leftLead;
  private SparkMaxConfig krightConfig;
  private SparkMaxConfig kleftConfig;
  private SparkMaxConfig kglobalConfig;
  private AbsoluteEncoder m_rightEncoder;
  private AbsoluteEncoder m_leftEncoder;
  private SparkClosedLoopController m_rightController;
  private SparkClosedLoopController m_leftController;

  private final SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (voltage) -> voltageDrive(voltage), 
      (log) -> new SysIdRoutineLog(log), 
      this)
  );

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
    Units.inchesToMeters(18)
  );

  private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(
    0.0,
    0.0,
    0.0
  );

  Pose2d pose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  /**
   * Use if one motor per side 
   * 
   * @param krightId CAN ID of right side motor
   * @param kleftId CAN ID of left side motor
  */
  public DriveSubsystem(int krightId, int kleftId) {
    //Initilaze the two SPARKMax motors
    final SparkMax m_rightLead = new SparkMax(krightId, MotorType.kBrushed);
    final SparkMax m_leftLead = new SparkMax(kleftId, MotorType.kBrushed);
  }
  
  /**
   * Use if two motors per side 
   * 
   * @param krightLeadId CAN ID of right side lead motor
   * @param krightFollowId CAN Id of right side follow motor
   * @param kleftLeadId CAN ID of left side lead motor
   * @param kleftFollowId CAN ID of left side follow motor
  */
  public DriveSubsystem(int krightLeadId, int krightFollowId, int kleftLeadId, int kleftFollowId) {
    //Initilaze the four SPARKMax motors
    final SparkMax m_rightLead = new SparkMax(krightLeadId, MotorType.kBrushed);
    final SparkMax m_rightFollow = new SparkMax(krightFollowId, MotorType.kBrushed);
    final SparkMax m_leftLead = new SparkMax(kleftLeadId, MotorType.kBrushed);
    final SparkMax m_leftFollow = new SparkMax(kleftFollowId, MotorType.kBrushed);

    final SparkClosedLoopController m_rightController = m_rightLead.getClosedLoopController();
    final SparkClosedLoopController m_leftController = m_leftLead.getClosedLoopController();

    // Create the configs for the motors
    final SparkMaxConfig krightFollowConfig = new SparkMaxConfig();
    final SparkMaxConfig kleftFollowConfig = new SparkMaxConfig();
    final SparkMaxConfig kglobalConfig = new SparkMaxConfig();
    final SparkMaxConfig krightConfig = new SparkMaxConfig();
    
    // Makes the motors follow another motor
    krightFollowConfig.follow(krightLeadId);
    kleftFollowConfig.follow(kleftLeadId);

    // Sets the Closed Loop configs for the motors
    kglobalConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(1.0, 0.0, 0.01, ClosedLoopSlot.kSlot0)
    .outputRange(-1, 1);

    krightConfig
    .apply(kglobalConfig)
    .inverted(true);

    // Apply the configs to the follow motors
    m_rightFollow.configure(krightFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftFollow.configure(kleftFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // Applies the global config to both lead motors
    m_rightLead.configure(krightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftLead.configure(kglobalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Drive using Tank-Style Commands
   *
   * @param leftSpeed Speed of left side, 1 = 100% Speed
   * @param rightSpeed Speed of right side, 1 = 100% Speed
   * 
   * @return A command to run
   */
  public Command tankDriveCommand(double leftSpeed, double rightSpeed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.runOnce(
      () -> {
        m_leftController.setReference(
          leftSpeed, 
          ControlType.kVelocity, 
          ClosedLoopSlot.kSlot0,
          m_feedForward.calculate(leftSpeed));

        m_rightController.setReference(
          rightSpeed, 
          ControlType.kVelocity, 
          ClosedLoopSlot.kSlot0,
          m_feedForward.calculate(rightSpeed));
      },
      this
    );
  }

  /**
   * Drive using Arcade-Style Commands
   *
   * @param speed Speed in the forward/backward plane, 1 = 1m/s
   * @param rot Speed to rotate in the x/y plane, 1 = 1rad/s
   * 
   * @return A command to run
   */
  public Command arcadeDriveCommand(double speed, double rot) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0.0, rot));
    
    return Commands.runOnce(
        () -> {
          m_leftController.setReference(
            wheelSpeeds.leftMetersPerSecond, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            m_feedForward.calculate(wheelSpeeds.leftMetersPerSecond));

          m_rightController.setReference(
            wheelSpeeds.rightMetersPerSecond, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            m_feedForward.calculate(wheelSpeeds.rightMetersPerSecond));
        },
        this
    );
  }
 
 /**
  * Drive the robot using pure Voltage, used by SysId
  * 
  * @param voltage Voltage provided by a SysId Routine
  * 
  * @return A command to run
  */
  private Command voltageDrive(Voltage voltage) {
    return Commands.run(
      () -> {
        m_leftLead.setVoltage(voltage);
        m_rightLead.setVoltage(voltage);
      }
    );
  }

  @Override
  public void periodic() {
    
  }
}
