/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TeleopLift extends Command {
  public TeleopLift() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.manualArm.setBrake(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double y = Robot.m_oi.getLift();
    Robot.manualArm.drive(y);
    //switches Solenoids utility based on right joystick user input
    if (y > 0.1){
      Robot.manualArm.setSolenoids(Value.kForward);
    }
    else if (y < -0.1){
      Robot.manualArm.setSolenoids(Value.kReverse);
    }
    else {
      Robot.manualArm.setSolenoids(Value.kOff);
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
    Robot.manualArm.setBrake(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
