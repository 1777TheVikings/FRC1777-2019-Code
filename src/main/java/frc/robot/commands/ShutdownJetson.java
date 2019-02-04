/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ConnectException;
import java.net.Socket;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Jetson;

/**
 * Add your docs here.
 */
public class ShutdownJetson extends InstantCommand {
  private Socket socket;
  private DataOutputStream out;

  public ShutdownJetson() {
    super();
    requires(Robot.jetson);
    setRunWhenDisabled(true);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    try {
      socket = new Socket(Jetson.ADDRESS, Jetson.PORT);
      out = new DataOutputStream(socket.getOutputStream());
      out.writeBytes("shutdown");
      out.flush();
      socket.close();
    } catch (ConnectException ex) {
      DriverStation.reportError("Could not connect to Jetson", ex.getStackTrace());
    } catch (IOException ex) {
      DriverStation.reportError("IOException", ex.getStackTrace());
    }
  }
}
