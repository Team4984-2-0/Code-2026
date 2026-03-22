package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ArmUp extends Command {
    
    private Intake arm;

    public ArmUp(Intake intake) {
        this.arm = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        arm.MoveArm(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        arm.StopArm();
    }
}