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
  public static final String DATA_SEPARATOR = ",";

  private double angle = 0.0;
  private double y_distance = 0.0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new GetDataFromJetson());
  }

  public void setAngle(double angle) {
    this.angle = angle;
    SmartDashboard.putNumber("Jetson angle", angle);
  }

  public double getAngle() {
    return this.angle;
  }

  public void setYDistance(double y_distance) {
    this.y_distance = y_distance;
    SmartDashboard.putNumber("Jetson Y distance", y_distance);
  }

  public double getYDistance() {
    return this.y_distance;
  }
}
