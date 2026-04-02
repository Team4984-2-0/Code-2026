package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class RollerSlow extends Command {
    
    private Intake intake;


    public RollerSlow(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.SpinRoller(-0.25);
    }

    @Override
    public void end(boolean interrupted) {
        intake.StopRoller();

    }
}