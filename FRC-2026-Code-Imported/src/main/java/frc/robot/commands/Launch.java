package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Launch extends Command {
        private Launcher LauncherSubsystem;
        private static int ShooterCounter = 0;
        private static int StartHopperRunCount = 30;
        private static int StartFeedRunCount = 30;
        
        public Launch(Launcher LauncherSubsystem){
            this.LauncherSubsystem = LauncherSubsystem;
            addRequirements(LauncherSubsystem);
        }
        @Override
        public void execute(){
            LauncherSubsystem.ShootRun(-0.20);
            ShooterCounter++;
            if (ShooterCounter > StartHopperRunCount) // 1 second
              LauncherSubsystem.HopperRun(-0.20);
            if (ShooterCounter > StartFeedRunCount)  //1.5 seconds
                LauncherSubsystem.FeedRun(-0.20);
        }
        @Override
        public void end(boolean interrupted){
            LauncherSubsystem.Killswitch();
        }
        
}