// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkMax leftClimbMotor = new CANSparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax rightClimbMotor = new CANSparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_ID, MotorType.kBrushless);

  public Climb() {
    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();
  }

  public void climbDown(double speed){
    //leftClimbMotor.set(speed);
    rightClimbMotor.set(-speed);
  }

  public void climbUp(double speed){
    //leftClimbMotor.set(-speed);
    rightClimbMotor.set(speed);
  }

  public void stopClimb(){
    leftClimbMotor.set(0);
    rightClimbMotor.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
