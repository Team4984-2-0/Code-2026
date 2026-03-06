package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
    
    private Climber climber;

    public Climb(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.Spin(0.4);
    }

    @Override
    public void end(boolean interrupted) {
        climber.Spin(0);
    }
}