package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;


public class thingspin extends Command {
    
    private Launcher launcher;


    public thingspin(Launcher launcher) {
        this.launcher = launcher;
        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.HopperRun(.2);
    }

    @Override
    public void end(boolean interrupted) {
                launcher.HopperStop();

    }
}