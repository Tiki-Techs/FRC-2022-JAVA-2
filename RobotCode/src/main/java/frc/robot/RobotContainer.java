// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.management.Descriptor;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SetFlywheelVelocity;
import frc.robot.commands.StopFlywheel;
import frc.robot.commands.stopIntakeMotor;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ShiftSpeed;
import frc.robot.commands.ShiftTorque;
import frc.robot.commands.StartIntakeMotor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Flywheel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveBase m_robotDrive = new DriveBase();
  private final Flywheel m_flywheel = new Flywheel();
  private final Intake m_intake = new Intake();
  private final Shifter m_shifter = new Shifter();


  private XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_ID);
      
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_flywheel.setDefaultCommand(new StopFlywheel(m_flywheel));
    m_intake.setDefaultCommand(new stopIntakeMotor(m_intake));    

    m_robotDrive.setDefaultCommand(
          new RunCommand(
            () -> m_robotDrive.arcadeDrive(-driverController.getLeftX(), driverController.getRightY()), m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(driverController, 1) //A Button
      .whenHeld(new SetFlywheelVelocity(m_flywheel));
    new JoystickButton(driverController, 2) //B Button
      .whenHeld(new StartIntakeMotor(m_intake));

    new JoystickButton(driverController, 3) //X Button
      .whenPressed(new ExtendIntake(m_intake));
    new JoystickButton(driverController, 4) //Y Button
      .whenPressed(new RetractIntake(m_intake));

    new JoystickButton(driverController, 5) //LB (left trigger button)
      .whenPressed(new ShiftSpeed(m_shifter));
    new JoystickButton(driverController, 6) //RB (right trigger button)
      .whenPressed(new ShiftTorque(m_shifter));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return blank auto command
    return new InstantCommand();
  }
}
