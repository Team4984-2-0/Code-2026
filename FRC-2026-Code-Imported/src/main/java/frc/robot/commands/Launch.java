package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Launch extends Command {
    private Launcher LauncherSubsystem;
    private static int ShooterCounter = 0;
    private static int StartHopperRunCount = 40;
    private static int StartFeedRunCount = 55;

    public Launch(Launcher LauncherSubsystem) {
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
    public void end(boolean interrupted) {
        ShooterCounter = 0;
        LauncherSubsystem.Killswitch();
    }

}