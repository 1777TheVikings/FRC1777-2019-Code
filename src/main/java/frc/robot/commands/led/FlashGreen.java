/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.led;

import com.mach.LightDrive.Color;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FlashGreen extends Command {
  private double lastTimestamp = 0.0;
  private static final double DELAY = 0.25;  // time between toggling
  private boolean wasOn = true;

  public FlashGreen() {
    requires(Robot.lightDrive);
    setTimeout(2);
    setName("FlashGreen");
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lightDrive.setColor(Color.GREEN);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if ((timeSinceInitialized() - lastTimestamp) > DELAY) {
      Robot.lightDrive.setColor(wasOn ? Color.GREEN : Color.OFF);
      wasOn = !wasOn;
      lastTimestamp = timeSinceInitialized();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Command command = new StaticTeamColor();
    command.start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
