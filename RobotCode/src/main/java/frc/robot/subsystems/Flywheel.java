// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
  private double targetVelocity = 3000;

  public Flywheel() {
    //Set up motor, PIDController, and encoder
    SmartDashboard.putNumber("shooter speed", 0);
    m_shooterMotor.restoreFactoryDefaults();

    m_pidController = m_shooterMotor.getPIDController();

    m_encoder = m_shooterMotor.getEncoder();

    //Set PID Constants
    m_pidController.setP(ShooterConstants.kP);
    m_pidController.setI(ShooterConstants.kI);
    m_pidController.setD(ShooterConstants.kD);
    m_pidController.setIZone(ShooterConstants.kIZone);
    m_pidController.setFF(ShooterConstants.kFeedForward);
    m_pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  } 

  //sets target RPM for PID loop

  public void setVelocity(double velocity) {
    targetVelocity = velocity;
    m_pidController.setReference(targetVelocity, CANSparkMax.ControlType.kVelocity);
    //SmartDashboard.putData("targetvel", m_pidController);
  }

  //sets static speed for motor testing
  public void setSpeed(double speed) {
    m_shooterMotor.set(speed);
  }

  //stops motor
  public void stop(){
    m_shooterMotor.set(0);
  }

  //Checks if motor is at Target Velocity
  public BooleanSupplier isOnTarget() {
    BooleanSupplier onTarget = () -> Math.abs(targetVelocity - m_encoder.getVelocity()) <= ShooterConstants.PIDTolerance; 
    return onTarget;
  }

  public BooleanSupplier isNotOnTarget() {
    BooleanSupplier onTarget = () -> Math.abs(targetVelocity - m_encoder.getVelocity()) >= ShooterConstants.PIDTolerance; 
    return onTarget;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("vel", m_encoder.getVelocity());
    //shooterSpeed = SmartDashboard.getNumber("shooter speed", 0);
    SmartDashboard.putBoolean("isOnTarget",isOnTarget().getAsBoolean());
    SmartDashboard.putBoolean("isNotOnTarget",isNotOnTarget().getAsBoolean());

    SmartDashboard.putNumber("test", Math.abs(targetVelocity - m_encoder.getVelocity()));
  }
}
