package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Robust toggle for Limelight pipelines (tries multiple common table names and logs results).
 * Use this to debug why pipeline changes aren't taking effect on your Limelight 2.
 */
public class ToggleLimelightPipelineDebug extends InstantCommand {
    private static final String[] CANDIDATE_TABLES = {
        "limelight", "limelight2", "limelight-2", "limelight:5800" // a few common variants
    };

    @Override
    public void initialize() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        boolean changed = false;

        for (String tableName : CANDIDATE_TABLES) {
            NetworkTable table = nt.getTable(tableName);
            NetworkTableEntry entry = table.getEntry("pipeline");

            // read with a sentinel default so we can detect missing key
            double before = entry.getDouble(Double.NaN);
            if (Double.isNaN(before)) {
                System.out.println("ToggleLimelightPipelineDebug: table '" + tableName + "' has no 'pipeline' entry.");
                continue;
            }

            int cur = (int) Math.round(before);
            int next = (cur == 1) ? 0 : 1;
            entry.setNumber(next);

            // read back to confirm
            double afterD = entry.getDouble(Double.NaN);
            int after = Double.isNaN(afterD) ? -999 : (int) Math.round(afterD);

            System.out.println("ToggleLimelightPipelineDebug: table='" + tableName + "' pipeline: " + cur + " -> " + after + " (set requested: " + next + ")");
            SmartDashboard.putNumber("Limelight/LastToggleTableIndex", java.util.Arrays.asList(CANDIDATE_TABLES).indexOf(tableName));
            SmartDashboard.putNumber("Limelight/Pipeline", after);

            changed = true;
            break; // stop after first table that appears valid
        }

        if (!changed) {
            System.out.println("ToggleLimelightPipelineDebug: no limelight table found with 'pipeline' entry. Confirm Limelight is connected and table name.");
            SmartDashboard.putString("Limelight/ToggleDebug", "no-table-found");
        } else {
            SmartDashboard.putString("Limelight/ToggleDebug", "toggled");
        }
    }
}