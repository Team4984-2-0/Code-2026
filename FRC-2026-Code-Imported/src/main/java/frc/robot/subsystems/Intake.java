package frc.robot.subsystems;

/** Simple wrapper around the REV brushless arm motor. */
import edu.wpi.first.math.controller.PIDController;
// Removed unused imports: LiveWindow, Spark
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Arm extends SubsystemBase {

    private SparkMax motor3;     
    private final PIDController elevatorPidController;
    private final RelativeEncoder motor1Encoder;

       

    /** Configures the brushless arm motor and PID stub. */
    public Arm() {
        motor3 = new SparkMax(11, MotorType.kBrushless);       
        motor1Encoder = motor3.getEncoder();


        elevatorPidController = new PIDController(0.4, 0, 0);
        
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    }

/** Spins the arm motor at the provided percent output. */
public void Spin(double value) {
    SmartDashboard.putNumber("encoder arm", get_encoder());

    motor3.set(value);
}

/** Convenience helper for zeroing the arm output. */
public void Spin() {
    motor3.set(0);
   
}

/** @return arm encoder position in rotations. */
public double get_encoder(){
    return motor1Encoder.getPosition();
}
}
