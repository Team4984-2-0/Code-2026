package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class DropArm extends Command {
    private Intake intakesub;

    public DropArm(Intake intakesub) {
        this.intakesub = intakesub;
        addRequirements(intakesub);
    }

    @Override
    public void execute() {
        intakesub.IntakeDrop();
    }

    @Override
    public void end(boolean interrupted) {
        intakesub.IntakeDropStop();
    }
}