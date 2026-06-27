// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandShooter;
import frc.robot.Targeting;

@SuppressWarnings("unused")
public class RobotContainer {
    private double MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.FieldCentricFacingAngle rotate = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(5, 0, 0) // Proportional control to rotate to the target angle, with a kP of 5 and no kI or kD
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withMaxAbsRotationalRate(MaxAngularRate); // Limit the max rotational rate to our defined MaxAngularRate to prevent overshooting

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final CommandShooter shooter = new CommandShooter();
    public final Targeting targeting = new Targeting();
    public final Vision vision = new Vision(drivetrain);
    
    public RobotContainer() {
        configureBindingsXbox();
    }

    private void configureBindingsXbox() {

        final CommandXboxController joystick = new CommandXboxController(0);
        final CommandXboxController operator = new CommandXboxController(1);
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Brake while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> brake).ignoringDisable(true)
        );

        // Brake while holding the A button
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Shoot to the target when the right bumper is 
        operator.rightBumper().and(joystick.b()).onTrue(Commands.runOnce(() -> {
            drivetrain.applyRequest(() -> rotate
                .withTargetDirection(new Rotation2d(
                    Units.degreesToRadians(targeting.angleToTarget(drivetrain.getState().Pose, targeting.getTargetPose()))
                ))
            );
            shooter.setSpeedAndPitch(
                targeting.getRPMForDistance(
                    targeting.distanceToTarget(new Pose2d(targeting.getTargetPose(), new Rotation2d()))
                ), 
                targeting.getAngleForDistance(
                    targeting.distanceToTarget(new Pose2d(targeting.getTargetPose(), new Rotation2d()))
                )
            );
        }, drivetrain, shooter)).onFalse(Commands.runOnce(
            () -> {
                drivetrain.applyRequest(() -> new SwerveRequest.Idle());
                shooter.stop();
            }, drivetrain, shooter));

        // Shoot on the Hub
        operator.leftBumper().and(joystick.b()).onTrue(Commands.startEnd(
            () -> {
                java.util.Collection<Pose2d> selectedPoses = DriverStation.getAlliance()
                    .orElse(null) == DriverStation.Alliance.Blue
                    ? Constants.shootToBlueHubPoses
                    : Constants.shootToRedHubPoses;
                drivetrain.pathFindToPose(
                    drivetrain.getState().Pose.nearest(selectedPoses)
                ).andThen(
                    shooter.setSpeedAndPitch(
                        targeting.getRPMForDistance(
                            targeting.distanceFromHub(drivetrain.getState().Pose)
                        ), targeting.getAngleForDistance(
                            targeting.distanceFromHub(drivetrain.getState().Pose)
                        )
                    )
                );
            }, 
            () -> {
                drivetrain.getCurrentCommand().cancel(); 
                shooter.stop();
            }, drivetrain, shooter));
        
        // Allow the operator to move the target pose with the left joystick
        new Trigger(() -> true).whileTrue(Commands.run(() -> {
            targeting.moveTargetPose(operator.getLeftX(), operator.getLeftY());
            logger.acceptTargetPose(new Pose2d(targeting.getTargetPose(), new Rotation2d(0.0)));
        }));

        vision.applyEstimatedPose();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindingsFlight() {

        final CommandJoystick joystick = new CommandJoystick(0);
        final CommandJoystick operator = new CommandJoystick(1);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative twist (left)
            )
        );

        // Brake while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> brake).ignoringDisable(true)
        );

        // Brake while holding button 0
        joystick.button(0).whileTrue(drivetrain.applyRequest(() -> brake));

        // Reset the field-centric heading on button 1 press.
        joystick.button(1).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Shoot to the target when the right bumper is held
        operator.button(0).and(joystick.button(2)).onTrue(Commands.runOnce(() -> {
            drivetrain.applyRequest(() -> rotate
                .withTargetDirection(new Rotation2d(
                    Units.degreesToRadians(targeting.angleToTarget(drivetrain.getState().Pose, targeting.getTargetPose()))
                )
                )
            );
            shooter.setSpeedAndPitch(
                targeting.getRPMForDistance(
                    targeting.distanceToTarget(new Pose2d(targeting.getTargetPose(), new Rotation2d()))
                ), 
                targeting.getAngleForDistance(
                    targeting.distanceToTarget(new Pose2d(targeting.getTargetPose(), new Rotation2d()))
                )
            );
        }, drivetrain, shooter)).onFalse(Commands.runOnce(
            () -> {
                drivetrain.applyRequest(() -> new SwerveRequest.Idle());
                shooter.stop();
            }, drivetrain, shooter));

        // Shoot on the Hub
        operator.button(1).and(joystick.button(2)).onTrue(Commands.startEnd(
            () -> {
                java.util.Collection<Pose2d> selectedPoses = DriverStation.getAlliance()
                    .orElse(null) == DriverStation.Alliance.Blue
                    ? Constants.shootToBlueHubPoses
                    : Constants.shootToRedHubPoses;
                drivetrain.pathFindToPose(
                    drivetrain.getState().Pose.nearest(selectedPoses)
                ).andThen(
                    shooter.setSpeedAndPitch(
                        targeting.getRPMForDistance(
                            targeting.distanceFromHub(drivetrain.getState().Pose)
                        ), targeting.getAngleForDistance(
                            targeting.distanceFromHub(drivetrain.getState().Pose)
                        )
                    )
                );
            }, 
            () -> {
                drivetrain.getCurrentCommand().cancel(); 
                shooter.stop();
            }, drivetrain, shooter));

        // Allow the operator to move the target pose with the joystick
        new Trigger(() -> operator.isConnected()).whileTrue(Commands.runOnce(() -> 
            targeting.moveTargetPose(operator.getX(), operator.getY())
        ));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auto
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(1)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auto
            drivetrain.applyRequest(() -> brake)
        );
    }
}
