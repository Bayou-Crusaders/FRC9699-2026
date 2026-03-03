// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.nio.file.Path;
import java.text.ParseException;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.commands.shootToHub;
import frc.robot.commands.shootToTarget;




@SuppressWarnings("unused")
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 1/4 of a rotation per second max angular velocity

    private PathPlannerPath path;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.065) // Add a 6.5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Targeting targeting = new Targeting();

    private final Shooter shooter = new Shooter(0, 0, 0);

    private final Telemetry logger = new Telemetry(MaxSpeed, targeting); 

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    final CommandXboxController driver = new CommandXboxController(0);
    final CommandJoystick shooterJoystick = new CommandJoystick(1);

    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        3.0, 3.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    public RobotContainer() {
        drivetrain.configureAutoBuilder();
        configureBindings();

        // Use to warmup the library (Yes, this is needed)
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Brake while the robot is disabled. This moves the wheels into
        // an 'x' arrangment to resist all force from all angles
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(
                () -> new SwerveRequest.SwerveDriveBrake()
            ).ignoringDisable(true)
        );

        // While 'A' is pressed, make the robot brake
        driver.a().whileTrue(
            drivetrain.applyRequest(
                () -> new SwerveRequest.SwerveDriveBrake()
            )
        );
        // Reset the field-centric heading on Right bumper press
        driver.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // When both 'x' and Button 5 on the shooter joystick are pressed, run the shootToHub command
        driver.x().and(shooterJoystick.button(5)).whileTrue(
            new shootToHub(drivetrain, shooter, targeting)
        );

        // When both 'x' and Button 6 on the shooter joystick are pressed, run the shootToTarget command
        driver.x().and(shooterJoystick.button(6)).whileTrue(
            new shootToTarget(drivetrain, shooter, targeting)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        PathPlannerPath path = null;
        try {
            // Load the path we want to pathfind to and follow
            path = PathPlannerPath.fromPathFile("test");
        } catch (IOException | org.json.simple.parser.ParseException | FileVersionException e) {
            System.out.println("Failed to load path file");
        }

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);

        return pathfindingCommand;
    }
}
