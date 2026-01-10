package frc.robot.commands;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
public class ArmAuto1 extends Command {
      private boolean Finished;
        private Launcher intakesub;

        
        public ArmAuto1(Launcher intakesub){
            
            this.intakesub = intakesub;
            addRequirements(intakesub);
            Finished = false;

        }
        
        
         @Override
        public void execute(){
            if (intakesub.get_encoder() > 0.5 && intakesub.get_encoder() < 1.5) {
                Finished = true;
            }
            else {
                if (intakesub.get_encoder() > 1.5){
                    intakesub.Spin(0.3);
                }
                if (intakesub.get_encoder() < 0.5){
                    intakesub.Spin(-0.3);
                }
             }
            }
            
        
        @Override
        public boolean isFinished() {
            return Finished;
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin();
            Finished = false;
        }
    
    }