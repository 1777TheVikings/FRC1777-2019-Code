/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TeleopHeadUnit extends Command {
  public TeleopHeadUnit() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.headUnit);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.headUnit.resetPid();
    // Robot.headUnit.seedEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.headUnit.setHands(Robot.m_oi.getHandPosition());
    Robot.headUnit.setHook(Robot.m_oi.getHookPosition());
    Robot.headUnit.setHeadTilt(Robot.m_oi.getHeadUnitTilt());

    // Robot.headUnit.pidTick();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.headUnit.setHands(Robot.m_oi.getHandPosition());
    Robot.headUnit.setHook(Robot.m_oi.getHookPosition());
    Robot.headUnit.stopHeadUnit();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
