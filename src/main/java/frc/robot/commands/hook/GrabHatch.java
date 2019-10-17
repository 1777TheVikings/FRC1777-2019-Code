/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hook;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class GrabHatch extends Command {
  /**
   * Add your docs here.
   */
  public GrabHatch() {
    requires(Robot.hook);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.hook.setPusherSolenoid(Value.kForward);
    Robot.hook.setVacuumSolenoid(Value.kForward);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}
