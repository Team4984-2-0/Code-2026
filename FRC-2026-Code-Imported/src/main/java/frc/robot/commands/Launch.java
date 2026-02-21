package frc.robot.commands;

/** Runs the launcher outward at a fixed speed while held. */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;

public class Launch extends Command {
        private Launcher Launchsub;
        
        /** Stores reference and declares subsystem requirement. */
        public Launch(Launcher Launchsub){
            this.Launchsub = Launchsub;
            addRequirements(Launchsub);
        }
        @Override
        public void execute(){
            Launchsub.HopperRun(-0.20);
        }
        @Override
        public void end(boolean interrupted){
            Launchsub.Killswitch();
        }
}
