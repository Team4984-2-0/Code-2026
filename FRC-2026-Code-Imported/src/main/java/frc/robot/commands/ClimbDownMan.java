package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbDownMan extends Command {
    private Climber climber;

    public ClimbDownMan(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.manSpin(-0.35);
    }

    @Override
    public void end(boolean interrupted) {
        climber.Spin(0);
    }
}