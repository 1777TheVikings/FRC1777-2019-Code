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

public class ControlLift extends Command {
  private static double desiredSetpoint;

  private double finishTime = -1.0;
  private boolean wasOnTarget = false;

  public ControlLift(double setpoint) {
    desiredSetpoint = setpoint;
    requires(Robot.lift);
    setTimeout(2);  // for safety
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lift.setBrake(false);
    Robot.lift.enable();
    Robot.lift.setSetpoint(desiredSetpoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.lift.onTarget()) {
      if (!wasOnTarget)
        finishTime = timeSinceInitialized() + 0.25;  // gives the lift time to settle
      wasOnTarget = true;
      Robot.lift.setSolenoids(Value.kOff);
    } else {
      finishTime = -1.0;
      wasOnTarget = false;
      if (Robot.lift.getPosition() < desiredSetpoint)
        Robot.lift.setSolenoids(Value.kForward);
      else
        Robot.lift.setSolenoids(Value.kReverse);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // true when reached safety timeout OR when finished and past settling time
    return isTimedOut() || (finishTime != -1 && timeSinceInitialized() >= finishTime);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.disable();
    Robot.lift.setBrake(true);
    Robot.lift.setSolenoids(Value.kOff);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
