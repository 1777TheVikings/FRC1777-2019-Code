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

public class GrabHatch extends Command {
  public GrabHatch() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.hook);
    setTimeout(0.6);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.hook.setGrabSolenoid(Value.kReverse);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.hook.setExtendSolenoid(Value.kForward);
    if (timeSinceInitialized() >= 0.3)
      Robot.hook.setGrabSolenoid(Value.kForward);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
