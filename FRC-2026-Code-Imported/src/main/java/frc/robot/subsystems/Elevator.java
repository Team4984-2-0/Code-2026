package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase {

    private final RelativeEncoder motor2Encoder;
    private SparkMax motor1;
    private SparkMax motor2;

    public Elevator() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        motor1 = new SparkMax(9, MotorType.kBrushless);
        motor2 = new SparkMax(10, MotorType.kBrushless);
        motor2Encoder = motor2.getEncoder();
        motor1.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        motor2.configure(elevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

public void Rotate(double value) {
    SmartDashboard.putNumber("encoder elevator", get_encoderElev());
    motor1.set(value);
    motor2.set(-value);
}

public void RotateStop() {
    motor1.set(0);
    motor2.set(0);
}
public double get_encoderElev(){
    return motor2Encoder.getPosition();
}

}
