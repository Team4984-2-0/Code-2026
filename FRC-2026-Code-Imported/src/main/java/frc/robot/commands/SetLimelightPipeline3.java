package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Set Limelight pipeline to 3. */
public class SetLimelightPipeline3 extends InstantCommand {
    @Override
    public void initialize() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry entry = table.getEntry("pipeline");
        entry.setNumber(3);
        SmartDashboard.putNumber("Limelight/Pipeline", 3);
        System.out.println("SetLimelightPipeline -> 3");
    }
}