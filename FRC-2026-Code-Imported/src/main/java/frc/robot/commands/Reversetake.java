package frc.robot.commands;

/** Simple manual command to spin the launcher inward for intaking notes. */
// Can Also prevent jamming when balls get stuck
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Reversetake extends Command {
        private Launcher intakesub;
        
        /** Store subsystem reference and declare requirement. */
        public Reversetake(Launcher intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override //For refernce the Hopperrun is from the launcher subsystem
        public void execute(){
            intakesub.HopperRun(-0.20);
        } 
        @Override // Also from the launcher subsystem this basically says that hopper run will be turned off should the button be released
        public void end(boolean interrupted){
            intakesub.Killswitch();
        }
}
