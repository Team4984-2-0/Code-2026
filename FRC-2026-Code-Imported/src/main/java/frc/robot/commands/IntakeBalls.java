package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeBalls extends Command {
        private Intake intakesub;
        
        public IntakeBalls(Intake intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            intakesub.Spin();
        }
        @Override
        public void end(boolean interrupted){
            intakesub.SpinStop();
        }
}
