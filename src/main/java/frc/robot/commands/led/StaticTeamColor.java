/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.led;

import com.mach.LightDrive.Color;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class StaticTeamColor extends InstantCommand {
  /**
   * Add your docs here.
   */
  public StaticTeamColor() {
    super();
    requires(Robot.lightDrive);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (DriverStation.getInstance().getAlliance() == Alliance.Red)
      Robot.lightDrive.setColor(Color.RED);
    else if (DriverStation.getInstance().getAlliance() == Alliance.Blue)
      Robot.lightDrive.setColor(Color.BLUE);
    else
      Robot.lightDrive.setColor(Color.YELLOW);
  }

}
