
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Momentary command bound to the driver button that zeroes the NavX yaw so
 * field-relative driving matches the driver's perspective.
 */
public class resetheading extends Command {

    private SwerveSubsystem swerveSubsystem;

    public resetheading(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        swerveSubsystem.zeroHeading();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}