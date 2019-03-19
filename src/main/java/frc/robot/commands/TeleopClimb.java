/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TeleopClimb extends Command {
  public TeleopClimb() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // redundancy is good
    if (!Robot.m_oi.getClimbConfirmation())
    {
      Robot.climber.setLift(0.0);
      Robot.climber.setSlide(0.0);
    }
    else
    {
      // if the limit switch is pressed and we're trying to go higher, we should not
      if (!(Robot.climber.getLimitSwitch() && Robot.m_oi.getClimbVertical() > 0.0))
        Robot.climber.setLift(Robot.m_oi.getClimbVertical());
      else
        Robot.climber.setLift(0.0);
      Robot.climber.setSlide(Robot.m_oi.getClimbSlide());
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.setLift(0.0);
    Robot.climber.setSlide(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
