package frc.robot.commands;

/** Timed auto routine for spinning the arm shooter for a brief window. */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ShootAuto extends Command {
        private Intake intakesub;
        public float timer;
        private boolean Finished = false;

        
        /** Stores arm reference and registers requirement. */
        public ShootAuto(Intake intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            if (Finished) {
                if(timer >= 10000){
                    Finished = true;
                }
                else{
                    intakesub.Spin();
                    timer += 0.00001;

                } 
            }
          

        }
        @Override
        public void end(boolean interrupted){
            intakesub.SpinStop();
        }
}
