package frc.robot.commands;

/**
 * Autonomous climb command that uses Limelight vision to align with the climb
 * bar AprilTag, drive toward it, and then engage the climber motor.
 *
 * <p>Sequence:
 * <ol>
 *   <li>Switch Limelight to the climb pipeline.</li>
 *   <li>Use the horizontal offset (TX) to rotate the robot toward the target.</li>
 *   <li>Drive forward at a fixed approach speed until the target area (TA)
 *       indicates the robot is close enough.</li>
 *   <li>Once aligned (|TX| &lt; tolerance) <em>and</em> close enough (TA &ge;
 *       threshold), engage the climber motor at full speed.</li>
 * </ol>
 *
 * <p>The command runs until interrupted (e.g. the driver releases the button).
 */

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoClimbCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Climber climber;

    /**
     * @param swerveSubsystem the swerve drive used to align with the target
     * @param climber         the climber motor used to latch on to the bar
     */
    public AutoClimbCmd(SwerveSubsystem swerveSubsystem, Climber climber) {
        this.swerveSubsystem = swerveSubsystem;
        this.climber = climber;
        addRequirements(swerveSubsystem, climber);
    }

    /** Switches the Limelight to the dedicated climb pipeline. */
    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", ClimbConstants.kClimbPipeline);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV("limelight");

        SmartDashboard.putBoolean("AutoClimb/HasTarget", hasTarget);

        if (!hasTarget) {
            // No target visible; stop everything and wait.
            swerveSubsystem.stopModules();
            climber.Spin();
            return;
        }

        double tx = LimelightHelpers.getTX("limelight"); // horizontal error (degrees)
        double ta = LimelightHelpers.getTA("limelight"); // target area (distance proxy)

        SmartDashboard.putNumber("AutoClimb/TX", tx);
        SmartDashboard.putNumber("AutoClimb/TA", ta);

        // Rotation: proportional correction toward the target (clamp to max speed).
        double rotationSpeed = -tx * ClimbConstants.kAlignKp;
        rotationSpeed = Math.max(-ClimbConstants.kMaxRotationSpeed,
                Math.min(ClimbConstants.kMaxRotationSpeed, rotationSpeed));

        // Forward drive: approach until close enough (TA threshold).
        double forwardSpeed = (ta < ClimbConstants.kClimbAreaThreshold)
                ? ClimbConstants.kApproachSpeed
                : 0.0;

        swerveSubsystem.driveRobotRelative(
                new ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed));

        // Engage the climber only when aligned and close enough.
        boolean aligned = Math.abs(tx) <= ClimbConstants.kAlignTolerance;
        boolean closeEnough = ta >= ClimbConstants.kClimbAreaThreshold;

        SmartDashboard.putBoolean("AutoClimb/Aligned", aligned);
        SmartDashboard.putBoolean("AutoClimb/CloseEnough", closeEnough);

        if (aligned && closeEnough) {
            climber.Spin(ClimbConstants.kClimbSpeed);
        } else {
            climber.Spin();
        }
    }

    /** Stops both the drive and the climber when the command ends. */
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        climber.Spin();
    }

    /** The command runs until interrupted by the driver releasing the button. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
