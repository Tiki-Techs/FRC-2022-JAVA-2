// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  //Creates two DoubleSolenoids for each side of intake
  private DoubleSolenoid intake = 
      new DoubleSolenoid(2, PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_FORWARD_CHANNEL, IntakeConstants.INTAKE_REVERSE_CHANNEL);
  
  //Initalize pistons as closed
  public Intake() {
    intake.set(Value.kReverse);
  }

  //Extend Intake
  public void extendIntake(){
    intake.set(Value.kForward);
  }

  //Retracts Intake
  public void retractIntake(){
    intake.set(Value.kReverse);
  }

  //Call to check if pistons are open(checks right side)
  public boolean intakeOut(){
    if(intake.get().equals(Value.kForward)){
      return true;
    }
    else{
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.put("Right Piston", rightPiston.get());
    //SmartDashboard.putData("Left Piston", leftPiston.get());
    SmartDashboard.putBoolean("IntakeOut?", intakeOut());
  }
}
