package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Launch extends Command {
        private Launcher LauncherSubsystem;
        private static int ShooterCounter = 0;  // 30 = 1 second
        private static int StartHopperRunCount = 30;
        private static int StartFeedRunCount = 45;
        private static double LauncherPower = 0.5;
        private static double HopperPower = 0.5;
        private static double FeedPower = 0.5;
        
        public Launch(Launcher LauncherSubsystem){
            this.LauncherSubsystem = LauncherSubsystem;
            addRequirements(LauncherSubsystem);
        }
        @Override
        public void execute(){


            LauncherSubsystem.ShootRun(LauncherPower);
            ShooterCounter++;
            if (ShooterCounter > StartHopperRunCount) 
              LauncherSubsystem.HopperRun(HopperPower);
            if (ShooterCounter > StartFeedRunCount)  
                LauncherSubsystem.FeedRun(FeedPower);
        }
        @Override
        public void end(boolean interrupted){
            LauncherSubsystem.Killswitch();
        }
        
        
}
