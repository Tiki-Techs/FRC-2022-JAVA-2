// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveBase extends SubsystemBase {
  //Declare Master and follower motors for each side of drivetrain
  private WPI_TalonFX m_rightMaster = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_1_ID);
  private WPI_TalonFX m_rightFollower = new WPI_TalonFX(DriveConstants.RIGHT_MOTOR_2_ID);
  private WPI_TalonFX m_leftMaster = new WPI_TalonFX(DriveConstants.LEFT_MOTOR_1_ID);
  private WPI_TalonFX m_leftFollower = new WPI_TalonFX(DriveConstants.LEFT_MOTOR_2_ID);
  
  private final DifferentialDrive m_drive = new DifferentialDrive(m_rightMaster,m_leftMaster);
    /** Creates a new DriveBase. */
  public DriveBase() {
    //Sets followers to follow masters
    m_rightFollower.follow(m_rightMaster);
    m_leftFollower.follow(m_leftMaster);

    //Set masters to run opposite on each side and follower to run opposite their master
    m_rightMaster.setInverted(TalonFXInvertType.Clockwise);
    m_leftMaster.setInverted(TalonFXInvertType.Clockwise);
    m_leftFollower.setInverted(true);
    m_rightFollower.setInverted(true);

    //Setup Integrated Encoders
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  //Differential tankDrive function for calling in Container
  public void arcadeDrive(double speed, double angle){
    m_drive.arcadeDrive(speed, angle);
  }

  public double getRightEncoderPos(){
    return m_rightMaster.getSelectedSensorPosition();
  }
public double getLeftEncoderPos(){
    return m_leftMaster.getSelectedSensorPosition();
  }

  public void resetEncoderPos(){
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance(){
    return (m_rightMaster.getSelectedSensorPosition() + m_leftMaster.getSelectedSensorPosition()) / 2;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("right master", m_rightMaster.get());
    SmartDashboard.putNumber("right follow", m_rightFollower.get());
    SmartDashboard.putNumber("left  follow", m_leftFollower.get());
    SmartDashboard.putNumber("left master", m_leftMaster.get());
    
    SmartDashboard.putNumber("right encoder", m_rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("left encoder", m_leftMaster.getSelectedSensorPosition());

  }
}
