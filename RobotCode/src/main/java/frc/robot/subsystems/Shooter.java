// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import frc.robot.Constants.ShooterConstants;

// public class Shooter extends PIDSubsystem {
//   private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);

//   //private final SimpleMotorFeedforward m_shooterFeedforward = 
//     //new SimpleMotorFeedforward(ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
//   public Shooter() {
//     super(
//         new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
//         //getController().setTolerance(ShooterConstants.kShooterToleranceRPS);
//         //setSetpoint(ShooterConstants.kShooterTargetRPS);
//   }

//   @Override
//   public void useOutput(double output, double setpoint) {
//     //m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
//   }

//   @Override
//   public double getMeasurement() {
//     // Return the process variable measurement here
//     return 0;
//   }
//   public boolean atSetpoint(){
//     return m_controller.atSetpoint();
//   }
// }
