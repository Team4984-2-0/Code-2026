package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Launcher extends SubsystemBase {

    private SparkMax motor4;
    private final RelativeEncoder motor1Encoder;

    public Launcher() {
        motor4 = new SparkMax(12, MotorType.kBrushless);
        motor1Encoder = motor4.getEncoder();
    }

public void Spin(double value) {
         SmartDashboard.putNumber("encoder arm", get_encoder());

    motor4.set(-value);
}

public void Spin() {
    motor4.set(0);
   
}

public double get_encoder(){
    return motor1Encoder.getPosition();
}

}
