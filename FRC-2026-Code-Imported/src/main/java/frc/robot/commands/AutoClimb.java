package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Two-phase Limelight command: first strafes until the camera-space X error is
 * within tolerance, then drives forward/backward until the camera-space Z value
 * (distance to the tag) matches the target standoff.
 */
public class AutoClimb extends Command {

    private enum Phase { Climb_U ,ALIGN_X, DRIVE_Z, DRIVE_Stop, COMPLETE }

    private final SwerveSubsystem swerveSubsystem;
    private final String limelightName;

    private final PIDController xController;
    private final PIDController zController;

    private Phase phase = Phase.ALIGN_X;
    private double lastPoseTimestamp = 0.0;

    public AutoClimb(SwerveSubsystem subsystem) {
        this(subsystem, VisionConstants.kPrimaryLimelightName);
    }

    public AutoClimb(SwerveSubsystem subsystem, String limelightName) {
        this.swerveSubsystem = subsystem;
        this.limelightName = limelightName;

        xController = new PIDController(VisionConstants.kAlignAdvanceXP, 0.0, 0.0);
        zController = new PIDController(VisionConstants.kAlignAdvanceZP, 0.0, 0.0);

        xController.setTolerance(VisionConstants.kAlignAdvanceXToleranceMeters);
        zController.setTolerance(VisionConstants.kAlignAdvanceZToleranceMeters);

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        phase = Phase.Climb_U;
        xController.reset();
        zController.reset();
        lastPoseTimestamp = 0.0;
        SmartDashboard.putString("AlignAdvance/Phase", phase.name());
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(limelightName)) {
            swerveSubsystem.stopModules();
            SmartDashboard.putBoolean("AlignAdvance/HasTarget", false);
            return;
        }
        SmartDashboard.putBoolean("AlignAdvance/HasTarget", true);

        Pose3d pose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
        double x = pose.getX(); // camera X axis (left/right)
        double z = pose.getZ(); // camera Z axis (forward)
        SmartDashboard.putNumber("AlignAdvance/X", x);
        SmartDashboard.putNumber("AlignAdvance/Z", z);
        SmartDashboard.putString("AlignAdvance/Phase", phase.name());

        double strafeMetersPerSecond = 0.0;
        double forwardMetersPerSecond = 0.0;

        switch (phase) {   
            // need to add climbing and lifting the arm to a specified point
            // case 

            case Climb_U:
                //climber up
                
                // check if climber can go any higher
                break;

            case ALIGN_X:
                strafeMetersPerSecond = MathUtil.clamp(
                        -xController.calculate(x, 0.0),
                        -VisionConstants.kAlignAdvanceMaxSpeedMetersPerSecond,
                        VisionConstants.kAlignAdvanceMaxSpeedMetersPerSecond);
                    
                if (x <= 0.2 && x >= -0.2) {
                    phase = Phase.DRIVE_Z;
                    zController.reset();
                }
                break;
            case DRIVE_Z:
                forwardMetersPerSecond = MathUtil.clamp(
                        -zController.calculate(z, VisionConstants.kAlignAdvanceGoalZMeters),
                        -VisionConstants.kAlignAdvanceMaxSpeedMetersPerSecond,
                        VisionConstants.kAlignAdvanceMaxSpeedMetersPerSecond);

                // Keep a small corrective strafe to handle drift while approaching.
                strafeMetersPerSecond = MathUtil.clamp(
                        -x * 0.5,
                        -0.6,
                        0.6);

                if (z <= 1.3 && z >= 1.2) {
                    phase = Phase.DRIVE_Stop;
                }
                break;
            case DRIVE_Stop:
                swerveSubsystem.stopModules();
                // climber down

                // check if climber is at the bottom

                phase = Phase.COMPLETE;
                lastPoseTimestamp = Timer.getFPGATimestamp();
            case COMPLETE:

                return;
        }

        swerveSubsystem.driveRobotRelative(
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                        forwardMetersPerSecond,
                        strafeMetersPerSecond,
                        0.0));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        if (phase != Phase.COMPLETE) {
            return false;
        }
        //phase = Phase.ALIGN_X;
        // Allow one cycle after completion to ensure the robot actually stopped.
        return Timer.getFPGATimestamp() - lastPoseTimestamp > 0.05;
    }
}