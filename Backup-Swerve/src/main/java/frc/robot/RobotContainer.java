// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandClimber;

public class RobotContainer {
    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick joystick = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final CommandClimber climber = new CommandClimber();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                fieldDrive.withVelocityX(-joystick.getY() * (MaxSpeed * ((-joystick.getThrottle()+1)/2))) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getX() * (MaxSpeed * ((-joystick.getThrottle()+1)/2))) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getTwist() * (MaxAngularRate * ((-joystick.getThrottle()+1)/2))) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> brake).ignoringDisable(true)
        );

        joystick.button(3).whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.button(8).and(joystick.button(5)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.button(8).and(joystick.button(6)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.button(7).and(joystick.button(5)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.button(7).and(joystick.button(6)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.button(2).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.povUp().whileTrue(climber.climberUp());
        joystick.povDown().whileTrue(climber.climberDown());
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auto
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric()),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                fieldDrive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0)

            // While driving forward, also run the climber homing command to prepare for teleop climbing.
            .alongWith(climber.homeClimber()),

            // Finally idle for the rest of auto
            drivetrain.applyRequest(() -> idle)
        );
    }
}
