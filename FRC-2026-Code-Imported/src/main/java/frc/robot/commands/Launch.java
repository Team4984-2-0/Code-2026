package frc.robot.commands;

/** Runs the launcher outward at a fixed speed while held. */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Launch extends Command {
        private Launcher LauncherSubsystem;
        
        /** Stores reference and declares subsystem requirement. */
        public Launch(Launcher LauncherSubsystem){
            this.LauncherSubsystem = LauncherSubsystem;
            addRequirements(LauncherSubsystem);
        }
        @Override
        public void execute(){
            LauncherSubsystem.ShootRun(-0.20);
            LauncherSubsystem.HopperRun(-0.20);
            LauncherSubsystem.FeedRun(-0.20);
        }
        @Override
        public void end(boolean interrupted){
            LauncherSubsystem.Killswitch();
        }
}
