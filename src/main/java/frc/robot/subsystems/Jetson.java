/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.GetDataFromJetson;

/**
 * Add your docs here.
 */
public class Jetson extends Subsystem {
  public static final String ADDRESS = "tegra-ubuntu.local";
  public static final int PORT = 5810;

  private double angle = 0.0;
  private Object angle_lock = new Object();

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new GetDataFromJetson());
  }

  public void setAngle(double angle) {
    synchronized (angle_lock) {
      this.angle = angle;
      SmartDashboard.putNumber("Jetson angle", angle);
    }
  }

  public double getAngle() {
    synchronized (angle_lock) {
      return this.angle;
    }
  }
}
