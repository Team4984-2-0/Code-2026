package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase {

    //SparkMax 10
    private SparkMax motor;     
    private final RelativeEncoder motor1Encoder;

    //SparkMax 11
    private SparkMax motor1;
    private final RelativeEncoder motor2Encoder;

    public Intake() {
        motor = new SparkMax(10, MotorType.kBrushless);       
        motor1Encoder = motor.getEncoder();

        motor1 = new SparkMax(11, MotorType.kBrushless);       
        motor2Encoder = motor1.getEncoder();


        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    }

    //spins motor (Spin)
    public void Spin() {
        motor.set(0.5);
    }

    //spins motor (IntakeDrop)
    public void IntakeDrop() {
        motor1.set(0.5);
    }

    //stops motor (Spin)
    public void SpinStop() {
        motor.set(0);
    }

    //stops motor (IntakeDrop)
    public void IntakeDropStop() {
        motor1.set(0);
    }

    public double get_encoder(){
        return motor1Encoder.getPosition();
    }
}