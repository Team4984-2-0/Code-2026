package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ChangeSpeedRegular extends Command {
        private final SwerveSubsystem swerveSubsystem;
        
        public ChangeSpeedRegular(SwerveSubsystem swerveSubsystem){
            this.swerveSubsystem = swerveSubsystem;
            addRequirements(swerveSubsystem);
        }
        @Override
        public void execute(){
            swerveSubsystem.maxspeed(1);
        }

        // This should tripple the speed of the robot
        @Override
        public void end(boolean interrupted){
            
        }
}

