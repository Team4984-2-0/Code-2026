package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Arm extends SubsystemBase {

    private SparkMax motor3;     
    private final RelativeEncoder motor1Encoder;

       

    public Arm() {
        motor3 = new SparkMax(11, MotorType.kBrushless);       
        motor1Encoder = motor3.getEncoder();
    }

public void Spin(double value) {
    SmartDashboard.putNumber("encoder arm", get_encoder());

    motor3.set(value);
}

public void Spin() {
    motor3.set(0);
   
}

public double get_encoder(){
    return motor1Encoder.getPosition();
}
}
