/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hook extends Subsystem {
  private static DoubleSolenoid pusherSolenoid = new DoubleSolenoid(RobotMap.hookPusherAPort, RobotMap.hookPusherBPort);
  private static Solenoid vacuumSolenoid = new Solenoid(RobotMap.hookVacuumAPort);

  public Hook() {
    vacuumSolenoid.set(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Value getPusherSolenoid() {
    return pusherSolenoid.get();
  }

  public boolean getVacuumSolenoid() {
    return vacuumSolenoid.get();
  }

  /**
   * Sets the value of the pusher solenoid
   * @param value kForward is extended, kReverse is retracted
   */
  public void setPusherSolenoid(Value value) {
    pusherSolenoid.set(value);
  }

  /**
   * Sets the value of the vacuum solenoid.
   * @param value true is enabled, false is disabled
   */
  public void setVacuumSolenoid(boolean value) {
    vacuumSolenoid.set(value);
  }
}
