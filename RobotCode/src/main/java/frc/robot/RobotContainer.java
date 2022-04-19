// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  //auto commands(too compo*/9/////////licated to make actual commands for everything so doing this complicated inlining stuff)
  private final Command m_simpleAuto = 
    new DriveDistance(1000, 0, -.7, m_robotDrive); //Drive off tarmac, bot facing forward
    // new InstantCommand();
  
  private final Command m_singleBallAuto = 
    new SequentialCommandGroup(
      new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel).withTimeout(4), //start shooter
      // new RunCommand(()-> m_flywheel.setVelocity(-2600), m_flywheel).withInterrupt(m_flywheel.isOnTarget()), //start shooter
      new ParallelCommandGroup( //shoot ball
        new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel),
        // new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake),
        // new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index)).withInterrupt(m_flywheel.isNotOnTarget()),
        new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index)).withTimeout(2),
      new InstantCommand(()-> m_index.stopIndexMotor(), m_index),
      new DriveDistance(1400, 0, .6, m_robotDrive) //drive off tarmac
     );

  private final Command m_doubleBallAuto = 
    new SequentialCommandGroup(
      new InstantCommand(m_intake::extendIntake,m_intake),
      new ParallelCommandGroup(
        new DriveDistance(700, 0, -.7, m_robotDrive),
        new RunCommand(()->m_intake.startIntakeMotor(-0.9)).withTimeout(3)
      ),
      new InstantCommand(m_intake::retractIntake,m_intake),
      new InstantCommand(m_intake::stopIntakeMotor, m_intake),
      new DriveDistance(1000, 0, .8, m_robotDrive),
      new DriveDistance(1050, .6, 0, m_robotDrive), //Turn around bot
      new DriveDistance(700, 0, -.7, m_robotDrive), //Turn around bot
      new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel).withTimeout(2), //start shooter
      new ParallelCommandGroup( //shoot ball
        new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel),
        new RunCommand(()-> m_intake.startIntakeMotor(-0.7), m_intake),
        new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index)).withTimeout(.3),
      new InstantCommand(m_index::stopIndexMotor, m_index),
      new InstantCommand(m_intake::stopIntakeMotor, m_intake),
      new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel).withTimeout(2), //start shooter
      new ParallelCommandGroup( //+pytt2ttshoot ball
        new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel),
        new RunCommand(()-> m_intake.startIntakeMotor(-0.7), m_intake),
        new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index)).withTimeout(.3),
      new InstantCommand(m_index::stopIndexMotor, m_index),
      new DriveDistance(1250, 0, .6, m_robotDrive)
    );

    // private final Command m_safeTwoBallAuto = 
    // new SequentialCommandGroup(
    //   new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel).withTimeout(3), //start shooter
    //   new ParallelCommandGroup( //shoot ball
    //     new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel),
    //     new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index)).withTimeout(.5),
    //   new InstantCommand(m_index::stopIndexMotor, m_index), 
    //   new InstantCommand(m_flywheel::stop,m_flywheel),
    //   new DriveDistance(500, 0, .8, m_robotDrive), //away from fender
    //   new DriveDistance(950, .6, 0, m_robotDrive), //Turn around bot
    //   new InstantCommand(m_intake::extendIntake, m_intake),
    //   new RunCommand(()-> m_intake.startIntakeMotor(-0.7), m_intake),
    //   new DriveDistance(800, 0, .7, m_robotDrive),//drive to ball
    //   new InstantCommand(m_intake::retractIntake,m_intake),
    //   new InstantCommand(m_intake::stopIntakeMotor, m_intake),
    //   new DriveDistance(1000, 0, .8, m_robotDrive), //drive backward toward
    //   new DriveDistance(1050, .6, 0, m_robotDrive), //Turn around bot
    //   new DriveDistance(700, 0, -.7, m_robotDrive), //
    //   new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel).withTimeout(2), //start shooter
    //   new ParallelCommandGroup( //shoot ball
    //     new RunCommand(()-> m_flywheel.setVelocity(-2700), m_flywheel),
    //     new RunCommand(()-> m_intake.startIntakeMotor(-0.7), m_intake),
    //     new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index)).withTimeout(.3),
    //   new InstantCommand(m_index::stopIndexMotor, m_index),
    //   new InstantCommand(m_intake::stopIntakeMotor, m_intake)    
    //   );
      
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(m_flywheel);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_climb);

    // Configure the button bindings
    configureButtonBindings();

    m_flywheel.setDefaultCommand(new StopFlywheel(m_flywheel));
    m_intake.setDefaultCommand(new StopIntake(m_intake));
    m_climb.setDefaultCommand(new StopClimb(m_climb)); 
    m_index.setDefaultCommand(new StopIndex(m_index));

    //Main drive code, left joystick for speed and right for angle
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.arcadeDrive(-driverController.getLeftX(), driverController.getRightY()), m_robotDrive));
    
    //Run climb off second controller's joysticks
    m_climb.setDefaultCommand(
       new RunCommand(
         () -> m_climb.runClimb(mechController.getLeftY()), m_climb));
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick}w or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(mechController, 10)
    //   .whenPressed(new SequentialCommandGroup(
    //     new InstantCommand(m_climb::resetClimbEncoders, m_climb),
    //     new RunCommand(()-> m_climb.runClimb(0, 0), m_climb).withInterrupt(()->m_climb.climbAtPos())
    //   ));

    new JoystickButton(mechController, 5) 
      .whileActiveOnce(new RunCommand(()-> m_flywheel.setVel(-2600, 0.9), m_flywheel)); 

    new JoystickButton(driverController, 6) 
      .whileActiveOnce(new RunCommand(()-> m_flywheel.setVel(-2600, 0.9), m_flywheel));

    new JoystickButton(mechController, 6)
      .whileActiveOnce(new RunCommand(()-> m_flywheel.setSpeed(-0.4), m_flywheel));

    // new JoystickButton(driverController, 5)
    //   .whileActiveOnce(new RunCommand(()-> m_flywheel.setSpeed(0.9), m_flywheel ));

      new JoystickButton(driverController, 5) //Driver LB
      .whileActiveOnce(new ParallelCommandGroup(
        new RunCommand(() -> m_index.startIndexMotor(0.7), m_index),//.withInterrupt(m_flywheel.isNotOnTarget()),
        new RunCommand(()-> m_flywheel.setSpeed(0.9), m_flywheel )));
      
    new JoystickButton(mechController, 7) //Mech Back Button, Poop ball
      .whileActiveOnce(new SequentialCommandGroup(
        new RunCommand(()-> m_flywheel.setSpeed(-0.4), m_flywheel).withTimeout(.3),
        new ParallelCommandGroup(
          new RunCommand(()-> m_flywheel.setSpeed(-0.4), m_flywheel), //continue running shooter
          new RunCommand(() -> m_intake.startIntakeMotor(-0.9), m_intake), //run intake and index until shoots
          new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index)).withTimeout(.1),
        new RunCommand(() -> m_index.startIndexMotor(0), m_index),
        new WaitCommand(1),
        new ParallelCommandGroup(
          new RunCommand(()-> m_flywheel.setSpeed(-0.4), m_flywheel), //continue running shooter
          new RunCommand(() -> m_intake.startIntakeMotor(-0.9), m_intake), //run intake and index until shoots
        new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index))));
    
    new JoystickButton(mechController, 8) //Mech Start Button, Throw up ball 
      .whileActiveOnce(new ConditionalCommand( 
        //runs all the wheels to spit out ball only if intake is out
        new ParallelCommandGroup(
          new RunCommand(()-> m_flywheel.setSpeed(0.2), m_flywheel), 
          new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake),
          new RunCommand(() -> m_index.startIndexMotor(0.7), m_index)),
        //if intake is in nothing happens, thus the instant command 
        new InstantCommand(),
        ()-> m_intake.isIntakeOut()));

    new JoystickButton(mechController, 1) //Mech A Button
      .whileActiveOnce(//new SequentialCommandGroup(
        new ParallelCommandGroup(
          new RunCommand(() -> m_intake.startIntakeMotor(-0.7), m_intake),
          new RunCommand(() -> m_index.startIndexMotor(-0.3), m_index)));
        //new RunCommand(() -> m_intake.startIntakeMotor(-0.7), m_intake)
  
          // new JoystickButton(mechController, 1) //A Button
    //   .whileActiveContinuous(new ConditionalCommand(
    //     new ParallelCommandGroup(
    //       new RunCommand(() -> m_index.startIndexMotor(-0.7), m_index),
    //       new RunCommand(() -> m_intake.startIntakeMotor(-0.7), m_intake)).withInterrupt(m_flywheel.isNotOnTarget()),
    //     new InstantCommand(), 
    //     m_flywheel.isOnTarget()));

    new JoystickButton(mechController, 2) //B Button
      .whileActiveOnce(new RunCommand(() -> m_index.startIndexMotor(0.9), m_intake));

    new JoystickButton(driverController, 1) //A Button
      .whileActiveOnce(new RunCommand(() -> m_intake.startIntakeMotor(-0.9), m_intake));

    new JoystickButton(driverController, 2) //B Button
      .whileActiveOnce(new RunCommand(() -> m_intake.startIntakeMotor(0.9), m_intake));

    new JoystickButton(mechController, 3) //Mech X Button
          .whenPressed(new SequentialCommandGroup(
        new InstantCommand(m_intake::extendIntake, m_intake),
        new RunCommand(()->m_intake.startIntakeMotor(-0.9), m_intake)));

    new JoystickButton(mechController, 4) //Mech Y Button
      .whenPressed(new SequentialCommandGroup(
        new InstantCommand(m_intake::retractIntake, m_intake),
        new RunCommand(m_intake::stopIntakeMotor, m_intake)));
    
    new JoystickButton(driverController, 9) //left joystick button
      .whenPressed(new ConditionalCommand(new InstantCommand(m_shifter::setTorque,m_shifter),
        new InstantCommand(m_shifter::setSpeed,m_shifter), ()->m_shifter.isInSpeed()));

    new JoystickButton(mechController, 10)
      .whenPressed(new InstantCommand(m_climb::changeClimbMode,m_climb));

    // new JoystickButton(mechController, 5) //LB (Left Trigger Button)
    //   .whenPressed(new ConditionalCommand(
    //     //retracts intake if intake is out
    //     new InstantCommand(m_intake::retractIntake, m_intake),
    //     //extends intake and starts intake motor if intake is retracted
    //     new InstantCommand(m_intake::extendIntake, m_intake).andThen(
    //     new RunCommand(() -> m_intake.startIntakeMotor(-0.9), m_intake)), 
    //     //checks if intake is extended or retracted
    //     ()-> m_intake.isIntakeOut()));
    }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return blank auto command
    return m_singleBallAuto;
  }
}