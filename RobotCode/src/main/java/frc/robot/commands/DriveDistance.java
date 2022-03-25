// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveBase;

public class DriveDistance extends CommandBase {
  /** Creates a new DriveDistance. */
  private final DriveBase driveBase;
  private final double distanceToDrive;
  private final double speedToDrive;
  private final double angleOfDrive;

  public DriveDistance(double inches, double speed, double angle, DriveBase subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    distanceToDrive = inches;
    angleOfDrive = angle;
    speedToDrive = speed;
    driveBase = subsystem;
    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveBase.resetEncoderPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.arcadeDrive(-speedToDrive, angleOfDrive);
    SmartDashboard.getNumber("distancetodrive", distanceToDrive);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(driveBase.getLeftEncoderPos() * DriveConstants.ENCODER_DISTANCE_PER_PULSE) >= distanceToDrive;
            //|| Math.abs(driveBase.getRightEncoderPos() * DriveConstants.ENCODER_DISTANCE_PER_PULSE) <= distanceToDrive;
  }
}
