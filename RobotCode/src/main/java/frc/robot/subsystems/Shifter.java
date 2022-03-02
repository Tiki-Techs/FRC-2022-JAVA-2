// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShiftConstants;

public class Shifter extends SubsystemBase {
  /** Creates a new Shifter. */
  private DoubleSolenoid shift = 
    new DoubleSolenoid(2, PneumaticsModuleType.REVPH, ShiftConstants.SHIFTER_FORWARD_CHANNEL, ShiftConstants.SHIFTER_REVERSE_CHANNEL);

  public Shifter() {
    shift.set(Value.kForward);
  }

  public void setSpeed(){
    shift.set(Value.kForward);
    //SmartDashboard.putBoolean("InSpeed?", true);
  }

  public void setTorque(){
    shift.set(Value.kReverse);
    //SmartDashboard.putBoolean("InSpeed?", false);

  }

  public void setOff(){
    shift.set(Value.kOff);
  }

  public BooleanSupplier isInSpeed(){
    if(shift.get().equals(Value.kForward)){
      return ()-> true;
    }
    else{
      return ()-> false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("InSpeed?", isInSpeed().getAsBoolean());
  }
}
