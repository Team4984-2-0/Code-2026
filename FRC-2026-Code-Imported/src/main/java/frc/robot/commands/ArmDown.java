package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmDown extends Command {
    
    private Arm climber;

    public ArmDown(Arm climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.Spin(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        climber.Spin(0);
    }
}