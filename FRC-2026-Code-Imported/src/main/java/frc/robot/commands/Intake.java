package frc.robot.commands;

/** Simple manual command to spin the launcher inward for intaking notes. */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Intake extends Command {
        private Launcher intakesub;
        
        /** Store subsystem reference and declare requirement. */
        public Intake(Launcher intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            intakesub.HopperRun(0.20);
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Killswitch();
        }
}
