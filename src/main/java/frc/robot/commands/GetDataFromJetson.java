/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.Socket;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Jetson;

public class GetDataFromJetson extends Command {
  private Socket socket;
  private BufferedReader in;
  private boolean successfulConnection;

  public GetDataFromJetson() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.jetson);
    setRunWhenDisabled(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    try {
      socket = new Socket(Jetson.ADDRESS, Jetson.PORT);
      in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
      this.successfulConnection = true;
    } catch (ConnectException ex) {
      this.successfulConnection = false;
      System.out.println("ConnectException: " + ex.getLocalizedMessage());
    } catch (IOException ex) {
      this.successfulConnection = false;
      System.out.println("IOException: " + ex.getLocalizedMessage());
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!this.successfulConnection) {
      SmartDashboard.putBoolean("Jetson connected", false);
      return;
    }
    SmartDashboard.putBoolean("Jetson connected", true);

    try {
      String inLine = in.readLine();
      if (inLine != null) {
        String[] inSplit = inLine.split(Jetson.DATA_SEPARATOR);
        Robot.jetson.setAngle(Double.valueOf(inSplit[0]));
        Robot.jetson.setYDistance(Double.valueOf(inSplit[1]));
      } else {
        this.successfulConnection = false;
      }
    } catch (IOException ex) {
      DriverStation.reportWarning("Jetson disconnected", ex.getStackTrace());
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !this.successfulConnection;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (!this.successfulConnection) return;

    try {
      if (!socket.isClosed()) socket.close();
    } catch (Exception ex) {
      DriverStation.reportError("Error when ending GetDataFromJetson", ex.getStackTrace());
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
