package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.Util;

public class shootToTarget extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    Shooter m_shooter;
    Targeting m_targeting;

    public shootToTarget(
        CommandSwerveDrivetrain drivetrain, 
        Shooter shooter, 
        Targeting targeting
    ) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_targeting = targeting;
    }

    @Override
    public void initialize() {
        // Get the robot's current pose
        Pose2d robotPose = m_drivetrain.getState().Pose;

        Double angleToRotate = m_targeting.angleToTarget(robotPose, m_targeting.getTargetPose().getTranslation())

        m_drivetrain.rotateToAngle(angleToRotate);
    }

    @Override
    public void execute() {
        if (m_drivetrain.getState().Pose.getRotation() == new Rotation2d(Units.degressToRadians(angleToRotate))) {
            double distanceToTarget = targeting.distanceFromHub(m_drivetrain.getState().Pose);

            // Use the distance and angle to set the shooter speed and angle
            shooter.setFlywheelRPM(targeting.getRPMForDistance(distanceToTarget));
            shooter.setPitchAngle(targeting.getAngleForDistance(distanceToTarget));
        } else {
            // do nothing while driving towards the shooting pose
        }
    }
}
