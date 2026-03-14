package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RollerOn extends Command {
    
    private Intake intake;

    public RollerOn(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.SpinRoller(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
        intake.StopRoller();
    }
}