// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  SparkMax leftLeader;
  SparkMax leftFollower;
  SparkMax rightLeader;
  SparkMax rightFollower;
  XboxController leftJoystick;
  XboxController rightJoystick;

  //Use these variables to change motor assignment; CAN Ids can be found in the Rev Hardware Client
  //Order does not matter, as long as the 'left' motors are set to left, and the 'right' motors are set to right, it will work
  int left1 = 4;
  int left2 = 3;
  int right1 = 1;
  int right2 = 2;

  public Robot() {
    // Initialize the SPARKMaxes
    leftLeader = new SparkMax(left1, MotorType.kBrushed);
    leftFollower = new SparkMax(left2, MotorType.kBrushed);
    rightLeader = new SparkMax(right1, MotorType.kBrushed);
    rightFollower = new SparkMax(right2, MotorType.kBrushed);

    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    globalConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
      .apply(globalConfig)
      .inverted(true);

    // Apply the follow parameter to the follow motors
    rightFollowerConfig
      .follow(rightLeader);
    
    leftFollowerConfig
      .follow(leftLeader);

    /*
     * Apply the configuration to the SPARKMaxes
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize joysticks; Use driver station to find which port the controlers are on.
    leftJoystick = new XboxController(0);
    rightJoystick = new XboxController(1);

  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    /*
     * Apply values to left and right side. We will only need to set the leaders
     * since the other motors are in follower mode.
     */
    leftLeader.set(-0.15);
    rightLeader.set(0.15);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    
    Double leftSpeed = leftJoystick.getLeftY();
    Double rightSpeed = rightJoystick.getRightY();

    leftLeader.setVoltage(leftSpeed);
    rightLeader.setVoltage(rightSpeed);

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
