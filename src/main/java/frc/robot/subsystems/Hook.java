/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hook extends Subsystem {
  private static DoubleSolenoid extendSolenoid = new DoubleSolenoid(RobotMap.hookExtendAPort, RobotMap.hookExtendBPort);
  private static DoubleSolenoid grabSolenoid = new DoubleSolenoid(RobotMap.hookGrabAPort, RobotMap.hookGrabBPort);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Value getExtendSolenoid() {
    return extendSolenoid.get();
  }

  public Value getGrabSolenoid() {
    return grabSolenoid.get();
  }

  /**
   * Sets the value of the extension solenoid
   * @param value kForward is extended, kReverse is retracted
   */
  public void setExtendSolenoid(Value value) {
    extendSolenoid.set(value);
  }

  /**
   * Sets the value of the grabbing solenoid
   * @param value kForward is expanded, kReverse is contracted
   */
  public void setGrabSolenoid(Value value) {
    grabSolenoid.set(value);
  }
}
