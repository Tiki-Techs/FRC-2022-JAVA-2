// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double targetVelocity = 3000;

  private CANSparkMax m_upperShooterMotor = new CANSparkMax(ShooterConstants.UPPER_SHOOTER_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_upperpidController;
  private RelativeEncoder m_upperEncoder;

  public Flywheel() {
    //Set up motor, PIDController, and encoder
    SmartDashboard.putNumber("shooter speed", 0);
    m_shooterMotor.restoreFactoryDefaults();
    m_pidController = m_shooterMotor.getPIDController();
    m_encoder = m_shooterMotor.getEncoder();

    m_upperShooterMotor.restoreFactoryDefaults();
    m_upperpidController = m_upperShooterMotor.getPIDController();
    m_upperEncoder = m_upperShooterMotor.getEncoder();

    //Set PID Constants
    m_pidController.setP(ShooterConstants.kP);
    m_pidController.setFF(ShooterConstants.kFeedForward);
    m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    m_upperpidController.setP(ShooterConstants.ukP);
    m_upperpidController.setFF(ShooterConstants.ukFeedForward);
    m_upperpidController.setOutputRange(ShooterConstants.ukMinOutput, ShooterConstants.ukMaxOutput);
  } 

  //sets target RPM for PID loop

  public void setVel(double velocity, double speed) {
    targetVelocity = velocity;
    m_pidController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
    m_upperShooterMotor.set(speed);  
  }

  public void setVelocity(double velocity) {
    targetVelocity = velocity;
    m_pidController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
    m_upperpidController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
  }

  //sets static speed for motor testing
  public void setSpeed(double speed) {
    m_shooterMotor.set(speed);
    m_upperShooterMotor.set(-speed);
  }

  //stops motor
  public void stop(){
    m_shooterMotor.set(0);
    m_upperShooterMotor.set(0);
  }

  //Checks if motor is at Target Velocity
  public BooleanSupplier isOnTarget() {
    BooleanSupplier onTarget = () -> (Math.abs(targetVelocity - m_encoder.getVelocity()) <= ShooterConstants.PIDTolerance); 
                                      //&& (m_encoder.getVelocity() > ShooterConstants.PIDTolerance); 
    return onTarget;
  }

  public BooleanSupplier isNotOnTarget() {
    BooleanSupplier onTarget = () -> Math.abs(targetVelocity - m_encoder.getVelocity()) >= ShooterConstants.PIDTolerance; 
    return onTarget;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("mainVel", m_encoder.getVelocity());
    SmartDashboard.putNumber("secondVel", m_upperShooterMotor.get());

    SmartDashboard.putBoolean("isOnTarget",isOnTarget().getAsBoolean());
    //SmartDashboard.putBoolean("isNotOnTarget",isNotOnTarget().getAsBoolean());

    SmartDashboard.putNumber("RPM Offset", Math.abs(targetVelocity - m_encoder.getVelocity()));

  }
}
