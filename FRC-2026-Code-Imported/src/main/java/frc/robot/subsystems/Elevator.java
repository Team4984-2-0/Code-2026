package frc.robot.subsystems;

/** Dual motor elevator subsystem for raising game-piece mechanisms. */

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase {


private final RelativeEncoder motor1Encoder;
    private final RelativeEncoder motor2Encoder;
    private double pidvalue;
    private SparkMax motor1;
    private SparkMax motor2;
    private final PIDController elevatorPidController;

    /** Configures motors, encoders, and the PID stub for future closed-loop work. */
    public Elevator() {


        
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motor1 = new SparkMax(9, MotorType.kBrushless);
        motor2 = new SparkMax(10, MotorType.kBrushless);
        motor1Encoder = motor1.getEncoder();
        motor2Encoder = motor2.getEncoder();
        motor1.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        motor2.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        elevatorPidController = new PIDController(0.4, 0, 0);
    }
    // Commenting out this for now, working on elevator auto move up / down. (2/12/25)

/** Spins the elevator motors in opposite directions so the stage rises/lowers. */
public void Rotate(double value) {
    SmartDashboard.putNumber("encoder elevator", get_encoderElev());
    motor1.set(value);
    motor2.set(-value);
}

/** Stops both elevator motors. */
public void RotateStop() {
    motor1.set(0);
    motor2.set(0);
}
/** @return encoder position for diagnostics. */
public double get_encoderElev(){
    return motor2Encoder.getPosition();
}

}
