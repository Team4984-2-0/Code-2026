package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToTagCustomValues extends Command {
    private final SwerveSubsystem swerve;

    // Hard lock to tag 11
    private static final int REQUIRED_TAG_ID = 11;

    private final double targetTx;
    private final double targetTy;
    private final double targetTa;

    // Make translation stronger than rotation so it drives to values, not orbit
    private final PIDController rotPID = new PIDController(0.015, 0.0, 0.0008);
    private final PIDController fwdPID = new PIDController(0.20, 0.0, 0.0);      // ta -> forward (stronger)
    private final PIDController strafePID = new PIDController(0.030, 0.0, 0.0);   // tx -> strafe (stronger)
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2.0);

    private static final double MAX_FWD = 0.55;
    private static final double MAX_STRAFE = 0.50;
    private static final double MAX_ROT = 0.20; // lower rotation cap prevents spinning around tag

    // Keep tag in view
    private static final double TX_KEEP_IN_VIEW_DEG = 23.0;
    private static final double SEARCH_ROT = 0.16;
    private double lastSeenTx = 0.0;

    private static final double TX_TOL = 0.8;
    private static final double TY_TOL = 2.0;
    private static final double TA_TOL = 0.20;
    private static final double ON_TARGET_SEC = 0.30;
    private double onTargetStart = Double.NaN;

    public AlignToTagCustomValues(SwerveSubsystem swerve, int ignoredTagId, double targetTx, double targetTy, double targetTa) {
        this.swerve = swerve;
        this.targetTx = targetTx;
        this.targetTy = targetTy;
        this.targetTa = targetTa;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        rotPID.reset();
        fwdPID.reset();
        strafePID.reset();
        rotLimiter.reset(0.0);
        onTargetStart = Double.NaN;
    }

    @Override
    public void execute() {
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = table.getEntry("tv").getDouble(0.0);
        double tid = table.getEntry("tid").getDouble(-1.0);

        /*  If lost tag, slow scan to reacquire
        if (tv < 1.0 || (int) Math.round(tid) != REQUIRED_TAG_ID) {
            double searchDir = (lastSeenTx >= 0.0) ? 1.0 : -1.0;
            driveNormalized(0.0, 0.0, searchDir * SEARCH_ROT);
            onTargetStart = Double.NaN;
            return;
        } */

        double tx = table.getEntry("tx").getDouble(0.0);
        double ty = table.getEntry("ty").getDouble(0.0);
        double ta = table.getEntry("ta").getDouble(0.0);
        lastSeenTx = tx;

        double errTx = targetTx - tx;
        double errTy = targetTy - ty;
        double errTa = targetTa - ta;

        // Priority: get to values (strafe + forward), keep only enough rotation to hold view
        double str = strafePID.calculate(tx, targetTx);
        str = clamp(str, -MAX_STRAFE, MAX_STRAFE);

        double fwd = fwdPID.calculate(ta, targetTa);
        fwd = clamp(fwd, -MAX_FWD, MAX_FWD);

        double rot = rotPID.calculate(tx, targetTx);
        rot = clamp(rot, -MAX_ROT, MAX_ROT);
        rot = -rotLimiter.calculate(rot);

        // When very off-center, pause forward but continue strafe/rotate to keep target visible
        if (Math.abs(tx) > TX_KEEP_IN_VIEW_DEG) {
            fwd = 0.0;
        }

        boolean within = Math.abs(errTx) <= TX_TOL
                && Math.abs(errTy) <= TY_TOL
                && Math.abs(errTa) <= TA_TOL;

        if (within) {
            if (Double.isNaN(onTargetStart)) onTargetStart = Timer.getFPGATimestamp();
            driveNormalized(0.0, 0.0, 0.0);
        } else {
            onTargetStart = Double.NaN;
            driveNormalized(fwd, str, rot);
        }
    }

    @Override
    public boolean isFinished() {
        return !Double.isNaN(onTargetStart)
                && (Timer.getFPGATimestamp() - onTargetStart) >= ON_TARGET_SEC;
    }

    @Override
    public void end(boolean interrupted) {
        driveNormalized(0.0, 0.0, 0.0);
    }

    private void driveNormalized(double forward, double strafe, double rotation) {
        double vx = forward * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double vy = strafe * DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        double omega = rotation * Math.PI;
        swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, omega));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}