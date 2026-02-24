package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Set Limelight pipeline to 2. */
public class SetLimelightPipeline2 extends InstantCommand {
    @Override
    public void initialize() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry entry = table.getEntry("pipeline");
        entry.setNumber(2);
        SmartDashboard.putNumber("Limelight/Pipeline", 2);
        System.out.println("SetLimelightPipeline -> 2");
    }
}