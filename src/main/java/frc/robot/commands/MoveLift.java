/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static org.junit.Assert.assertNotNull;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.led.FlashGreen;

public class MoveLift extends Command {
  private final double setpoint;

  public MoveLift(double setpoint) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.lift);

    this.setpoint = setpoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lift.setSetpoint(setpoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lift.updateCounters();
    Robot.lift.pidTick();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.lift.isPidDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.useMotors(0.0);
    if (Robot.lightDrive.getCurrentCommand() != null) {
      Robot.lightDrive.getCurrentCommand().cancel();
    }
    Command command = new FlashGreen();
    command.start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
