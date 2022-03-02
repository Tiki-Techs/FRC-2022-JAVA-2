// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Index extends SubsystemBase {
  /** Creates a new Index. */
  private CANSparkMax m_indexMotor = new CANSparkMax(IntakeConstants.INDEX_MOTOR_ID, MotorType.kBrushless);

  public Index() {
    m_indexMotor.restoreFactoryDefaults();
  }

  public void startIndexMotor(double speed){
    m_indexMotor.set(speed);
  }

  public void stopIndexMotor(){
    m_indexMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
