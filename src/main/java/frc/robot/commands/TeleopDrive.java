/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TeleopDrive extends Command {
  public TeleopDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // allows for robot to go perfectly sideways or forward/backward without joystick having to be at perfect angle
    double x = Robot.m_oi.getDriveX();
    double y = -Robot.m_oi.getDriveY();
    double angle = Math.atan(y/x);
    // System.out.println("Angle: " + angle);
    if (Math.abs(angle) < Math.toRadians(5)){
      y = 0;
    }
    if (Math.abs(angle) > Math.toRadians(85)){
      x = 0;
    }

    // System.out.println("Y: " + y + ", X: " + x);
    Robot.driveTrain.drive(y, x, Robot.m_oi.getDriveTwist());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
