package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class atLaunch extends Command {
    private Launcher LauncherSubsystem;
    private static int ShooterCounter = 0;
    private static int StartHopperRunCount = 40;
    private static int StartFeedRunCount = 55;
            private boolean Finished;


    public atLaunch(Launcher LauncherSubsystem) {

        this.LauncherSubsystem = LauncherSubsystem;
        addRequirements(LauncherSubsystem);
    }

    @Override
    public void execute() {
        LauncherSubsystem.ShootRun(0.85);
        ShooterCounter++;
        if (ShooterCounter > StartHopperRunCount) // 1 second
          {  LauncherSubsystem.HopperRun(.2);}
        if (ShooterCounter > StartFeedRunCount) // 1.5 seconds
           { LauncherSubsystem.FeedRun(-.80);}

        }
        
    @Override
    public boolean isFinished() {
        if(ShooterCounter > 230){
            return true;
        }
        else {
            return false;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        ShooterCounter = 0;
        LauncherSubsystem.Killswitch();

    }
  

}