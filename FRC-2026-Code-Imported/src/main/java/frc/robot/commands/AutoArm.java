package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoArm extends Command {
    
    private Intake arm;
     private static int ShooterCounter = 0;

    public AutoArm(Intake intake) {
        this.arm = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        ShooterCounter++;

        arm.MoveArm(0.1);
    }

     @Override
    public boolean isFinished() {
        if(ShooterCounter > 40){
            return true;
        }
        else {
            return false;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.StopArm();
                ShooterCounter = 0;

    }
}