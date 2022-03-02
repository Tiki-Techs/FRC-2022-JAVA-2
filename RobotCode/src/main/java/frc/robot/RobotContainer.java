// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.StopClimb;
import frc.robot.commands.StopFlywheel;
import frc.robot.commands.StopIndex;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Index;

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
  private final Climb m_climb = new Climb();
  private final Index m_index = new Index();


  private XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_ID);
  private XboxController mechController = new XboxController(Constants.MECH_CONTROLLER_ID);

  //auto commands(too complicated to make actual commands for everything so doing this complicated inlining stuff)
  private final Command m_simpleAuto = 
    new DriveDistance(250, .5, m_robotDrive); //drive off tarmac :)
  
  private final Command m_singleBallAuto = 
    new SequentialCommandGroup(
      new DriveDistance(1, .4, m_robotDrive), //drive into wall
      new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel).withInterrupt(m_flywheel.isOnTarget()), //start shooter
      new ParallelCommandGroup( //shoot ball
        new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel),
        new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake),
        new RunCommand(() -> m_index.startIndexMotor(-0.7), m_intake)).withInterrupt(m_flywheel.isNotOnTarget()),
      new DriveDistance(1, .4, m_robotDrive) //drive off tarmac
    );
  
  private final Command m_doubleBallAuto = 
    new SequentialCommandGroup(
      new SequentialCommandGroup( //extends and runs intake
        new InstantCommand(m_intake::extendIntake, m_intake), 
        new RunCommand(() -> m_intake.startIntakeMotor(.9), m_intake)),
      new DriveDistance(1, .5, m_robotDrive), //Drive into ball
      new DriveDistance(1, -0.5, m_robotDrive),//drive back into wall
      new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel).withInterrupt(m_flywheel.isOnTarget()), //start shooter
      new ParallelCommandGroup( //shoot balls
        new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel),
        new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake),
        new RunCommand(() -> m_index.startIndexMotor(-0.7), m_intake)).withInterrupt(m_flywheel.isNotOnTarget()),
      new InstantCommand(()-> m_index.stopIndexMotor(), m_intake),
      new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel).withInterrupt(m_flywheel.isOnTarget()),
      new ParallelCommandGroup(
        new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel),
        new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake),
        new RunCommand(() -> m_index.startIndexMotor(-0.7), m_intake)).withInterrupt(m_flywheel.isNotOnTarget()),
      new DriveDistance(1, .4, m_robotDrive) //drive off tarmac
    );
      
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_flywheel.setDefaultCommand(new StopFlywheel(m_flywheel));
    m_intake.setDefaultCommand(new StopIntake(m_intake));
    m_climb.setDefaultCommand(new StopClimb(m_climb)); 
    m_index.setDefaultCommand(new StopIndex(m_index));

    //Main drive code, left joystick for speed and right for angle
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.arcadeDrive(-driverController.getRightX(), driverController.getLeftY()), m_robotDrive));
    
    //Run climb off second controller's joysticks
    m_climb.setDefaultCommand(
      new RunCommand(
        () -> m_climb.runClimb(mechController.getRightY(), mechController.getLeftY()), m_climb));
      
    // new PerpetualCommand(new ConditionalCommand(()->m_intake.startIntakeMotor(.9), ()->m_intake.stopIntakeMotor(), m_intake.isIntakeOut()));
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(driverController, 7) //Back Button
      .whileActiveOnce(new RunCommand(() -> m_flywheel.setVelocity(-3000), m_flywheel));
      
    new JoystickButton(driverController, 1) //A Button
      .whileActiveOnce(new SequentialCommandGroup(
        new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel).withInterrupt(m_flywheel.isOnTarget()), //speed up shooter
        new ParallelCommandGroup(
          new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel), //continue running shooter
          new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake), //run intake and index until shoots
          new RunCommand(() -> m_index.startIndexMotor(-0.7), m_intake)).withInterrupt(m_flywheel.isNotOnTarget()),
        new InstantCommand(()-> m_index.stopIndexMotor(), m_intake), //stop index then repeat last few steps for second ball
        new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel).withInterrupt(m_flywheel.isOnTarget()),
        new ParallelCommandGroup(
          new RunCommand(()-> m_flywheel.setVelocity(-3000), m_flywheel),
          new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake),
          new RunCommand(() -> m_index.startIndexMotor(-0.7), m_intake)).withInterrupt(m_flywheel.isNotOnTarget())
        ));

    new JoystickButton(driverController, 5) //LB (left trigger button)
      .whileActiveOnce(new RunCommand(() -> m_intake.startIntakeMotor(.9), m_intake));

    new JoystickButton(mechController, 1) // A Button
      .whileActiveOnce(new RunCommand(() -> m_intake.startIntakeMotor(-.9), m_intake));

    new JoystickButton(driverController, 6) //RB (right trigger button)
      .whileActiveOnce(new RunCommand(() -> m_index.startIndexMotor(-0.7), m_intake));

    new JoystickButton(driverController, 8) //Start Button
      .whileActiveOnce(new RunCommand(() -> m_index.startIndexMotor(0.7), m_intake));

    new JoystickButton(driverController, 3) //X Button
      .whenPressed(new SequentialCommandGroup(
        new InstantCommand(m_intake::extendIntake, m_intake), 
        new RunCommand(() -> m_intake.startIntakeMotor(.9), m_intake)));

    new JoystickButton(driverController, 4) //Y Button
      .whenPressed(new InstantCommand(m_intake::retractIntake, m_intake));

    new JoystickButton(driverController, 9) //left joystick button
      //.whenPressed(new InstantCommand(m_shifter::setSpeed, m_shifter));
      .whenPressed(new ConditionalCommand(new InstantCommand(m_shifter::setTorque,m_shifter),
        new InstantCommand(m_shifter::setSpeed,m_shifter), m_shifter.isInSpeed()));

    new JoystickButton(driverController, 10) //right joystick button
      .whenPressed(new InstantCommand(m_shifter::setTorque, m_shifter));
    
    
    //Extra code for climb in case need second controllers joysticks(just so i dont have to rewrite)
    // new JoystickButton(mechController, 1) //A Button
    //   .whileActiveOnce(new RunCommand(() -> m_climb.rightClimbDown(.3), m_climb));
    // new JoystickButton(mechController, 2) //B Button
    //   .whileActiveOnce(new RunCommand(() -> m_climb.rightClimbUp(.3), m_climb));
    // new JoystickButton(mechController, 3) //x Button
    //   .whileActiveOnce(new RunCommand(() -> m_climb.leftClimbDown(.3), m_climb));
    // new JoystickButton(mechController, 4) //Y Button
    //   .whileActiveOnce(new RunCommand(() -> m_climb.leftClimbUp(.3), m_climb));
    }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return blank auto command
    return m_simpleAuto;
  }
}
