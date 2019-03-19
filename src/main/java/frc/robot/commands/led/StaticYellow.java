/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import com.mach.LightDrive.Color;

/**
 * Add your docs here.
 */
public class StaticYellow extends InstantCommand {
  /**
   * Add your docs here.
   */
  public StaticYellow() {
    super();
    requires(Robot.lightDrive);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.lightDrive.setColor(Color.YELLOW);
  }

}
