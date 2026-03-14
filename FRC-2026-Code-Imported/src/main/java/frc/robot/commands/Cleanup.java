
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Cleanup extends Command {
        private Launcher LauncherSubsystem;
        private static int ShooterCounter = 0;  // 30 = 1 second
        private static int StartHopperRunCount = 10;
        private static int StartFeedRunCount = 15;
        private static double LauncherPower = -0.5;
        private static double HopperPower = -0.5;
        private static double FeedPower = -0.5;
        
        public Cleanup(Launcher LauncherSubsystem){
            this.LauncherSubsystem = LauncherSubsystem;
            addRequirements(LauncherSubsystem);
        }
        @Override
           public void execute() {
      //  LauncherSubsystem.ShootRun(0.7);
      
   
           LauncherSubsystem.HopperRun(.2);
      
           // LauncherSubsystem.FeedRun(-.60);
    }
        @Override
        public void end(boolean interrupted){
            LauncherSubsystem.Killswitch();
        }       
        
        
        
        
}