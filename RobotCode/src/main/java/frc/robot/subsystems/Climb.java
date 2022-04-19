// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, MotorType.kBrushless);

  private RelativeEncoder leftClimbEncoder = leftClimbMotor.getEncoder();
  private RelativeEncoder rightClimbEncoder = rightClimbMotor.getEncoder();

  private double rightPos;
  private double leftPos;


  // private DoubleSolenoid climbPancake = 
  //   new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, ClimbConstants.CLIMB_FORWARD_CHANNEL, ClimbConstants.CLIMB_REVERSE_CHANNEL);
  public Climb() {
    //setup encoders
    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();

    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);

    rightClimbMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftClimbMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    rightClimbMotor.follow(leftClimbMotor, true);
      //set climb pancake to forward
  }

  public void runClimb(double speed){
    leftClimbMotor.set(speed);
  }

  public void stopClimb(){
    leftClimbMotor.set(0);
    rightClimbMotor.set(0);
  }

  public void changeClimbMode(){
    leftClimbMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //rightClimbMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void resetClimbEncoders(){
    leftClimbEncoder.setPosition(0);
    rightClimbEncoder.setPosition(0);
  }

  public double getAvgClimbPos(){
    //rightPos = rightClimbEncoder.getPosition();
    leftPos = leftClimbEncoder.getPosition();
    return (-rightPos + leftPos) / 2;
  }

  public boolean climbAtPos(){
    return getAvgClimbPos() > ClimbConstants.CLIMB_STOP_POINT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("rightClimbEncoder", rightClimbEncoder.getPosition());
    SmartDashboard.putNumber("leftClimbEncoder", leftClimbEncoder.getPosition());
    // SmartDashboard.putNumber("climbCurrent", leftClimbMotor.getOutputCurrent());
    // SmartDashboard.putNumber("climbVoltage", leftClimbMotor.getBusVoltage());
  }
}
