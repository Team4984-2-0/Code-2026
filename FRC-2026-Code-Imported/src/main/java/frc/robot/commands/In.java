package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class In extends Command {
    
    private Intake climber;

    public In(Intake climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.Spin(-0.4);
    }

    @Override
    public void end(boolean interrupted) {
        climber.Spin(0);
    }
}