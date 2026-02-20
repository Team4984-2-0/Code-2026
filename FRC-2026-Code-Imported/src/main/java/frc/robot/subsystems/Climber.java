package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

    private SparkMax climbMotor;

    public Climber() {
        climbMotor = new SparkMax(13, MotorType.kBrushless);
    }

public void Spin(double value) {
    climbMotor.set(value);
  
}


public void Spin() {
    climbMotor.set(0);
   
   
}


}
