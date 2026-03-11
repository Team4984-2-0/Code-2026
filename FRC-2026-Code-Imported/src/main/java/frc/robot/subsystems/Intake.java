package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Winch subsystem reserved for endgame climbing. */

// Removed unused imports (commands.*, LiveWindow, Spark)
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase {
    private final RelativeEncoder LeftArmEncoder;
    private final RelativeEncoder RightArmEncoder;
    private SparkMax LeftArm;
    private SparkMax RightArm;
    private SparkMax RollerMotor;
    


    /** Initializes the climb motor and idle mode. */
    public Intake() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        RollerMotor = new SparkMax(12, MotorType.kBrushless);
        LeftArm = new SparkMax(10, MotorType.kBrushless);
        RightArm = new SparkMax(11, MotorType.kBrushless);
        LeftArmEncoder = LeftArm.getEncoder();
        RightArmEncoder = RightArm.getEncoder();

    }

    /** Spins the climb motor at the provided percent output, with safety and readjustment logic. */

    public void MoveArm(double value) {
        LeftArm.set(value);
        RightArm.set(value);
    }
    public void SpinRoller(double value) {
        RollerMotor.set(value);         
    }

    public void StopRoller() {
        RollerMotor.set(0);         
    }       

    public void StopArm() {
        LeftArm.set(0);
        RightArm.set(0);
    }   

    public double get_encoderLeft(){
    return LeftArmEncoder.getPosition();

    }
    public double get_encoderRight(){
    return RightArmEncoder.getPosition();       

    }
}