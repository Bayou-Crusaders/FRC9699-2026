package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;

public class shootToHub extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Targeting targeting;
    
    public shootToHub(CommandSwerveDrivetrain drivetrain, Shooter shooter, Targeting targeting) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.targeting = targeting;
    }

    @Override
    public void initialize() {
        // Get the distance to the target and calculate the desired RPM and angle for shooting
        double distanceToHub = targeting.distanceFromHub(drivetrain.getState().Pose);
        double desiredRPM = targeting.getRPMForDistance(distanceToHub);
        double desiredAngle = targeting.getAngleForDistance(distanceToHub);
        
        // Set the shooter to the desired RPM and angle
        shooter.setFlywheelRPM(desiredRPM);
        shooter.setPitchAngle(desiredAngle);
    }

}
