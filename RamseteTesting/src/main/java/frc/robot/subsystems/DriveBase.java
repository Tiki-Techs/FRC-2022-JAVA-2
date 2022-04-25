// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class DriveBase extends SubsystemBase {
  
    private WPI_TalonFX frontLeft = new WPI_TalonFX(DriveConstants.LEFT_FRONT);
    private WPI_TalonFX backLeft = new WPI_TalonFX(DriveConstants.LEFT_BACK);
    private WPI_TalonFX frontRight = new WPI_TalonFX(DriveConstants.RIGHT_FRONT);
    private WPI_TalonFX backRight = new WPI_TalonFX(DriveConstants.RIGHT_BACK);

    // The motors on the left side of the drive.
    private final MotorControllerGroup left = new MotorControllerGroup(frontLeft, backLeft);

    // Define right Speed Controller
    private final MotorControllerGroup right = new MotorControllerGroup(frontRight, backRight);

    private final DifferentialDrive m_drive = new DifferentialDrive(left, right);

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /** Creates a new DriveSubsystem. */
    public DriveBase() {
      backRight.follow(frontRight);
      backLeft.follow(frontLeft);

      frontRight.setInverted(TalonFXInvertType.Clockwise);
      frontLeft.setInverted(TalonFXInvertType.Clockwise);
      backLeft.setInverted(true);
      backRight.setInverted(true);

      frontRight.configFactoryDefault();
      backRight.configFactoryDefault();
      frontLeft.configFactoryDefault();
      backRight.configFactoryDefault();

      configureTalon(frontRight);
      configureTalon(backRight);
      configureTalon(frontLeft);
      configureTalon(frontRight);

      // Sets the distance per pulse for the encoders
      resetEncoders();
      m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    @Override
    public void periodic() {
      // Update the odometry in the periodic block

      m_odometry.update(m_gyro.getRotation2d(),
          frontLeft.getSelectedSensorPosition() / (DriveConstants.ENCODER_DISTANCE_PER_PULSE * DriveConstants.GEAR_RATIO)
              * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_DIAMETER * Math.PI),
          frontRight.getSelectedSensorPosition() / (DriveConstants.ENCODER_DISTANCE_PER_PULSE * DriveConstants.GEAR_RATIO)
              * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_DIAMETER * Math.PI));

      SmartDashboard.putNumber("Left Encoder", frontLeft.getSelectedSensorPosition());
      SmartDashboard.putNumber("Right Encoder", frontRight.getSelectedSensorPosition());
    }

    private void configureTalon(TalonFX talonFX){
      talonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 50);
      talonFX.setNeutralMode(NeutralMode.Coast);
      talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
      talonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 40);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(
          (frontLeft.getSelectedSensorPosition() * 10) / (DriveConstants.ENCODER_DISTANCE_PER_PULSE * DriveConstants.GEAR_RATIO)
              * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_DIAMETER * Math.PI),
          (frontRight.getSelectedSensorVelocity() * 10) / (DriveConstants.ENCODER_DISTANCE_PER_PULSE * DriveConstants.GEAR_RATIO)
              * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_DIAMETER * Math.PI));
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**%
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
      m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
      left.setVoltage(leftVolts);
      right.setVoltage(rightVolts);
      m_drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      frontLeft.setSelectedSensorPosition(0);
      frontRight.setSelectedSensorPosition(0);
}

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
}

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftDistance() {
    return (frontLeft.getSelectedSensorPosition(0));
}

public double getRightDistance() {
    return (frontRight.getSelectedSensorPosition(0));// * 0.001;
}

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
