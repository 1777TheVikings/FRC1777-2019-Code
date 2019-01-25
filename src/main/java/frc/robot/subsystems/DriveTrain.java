/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import frc.robot.RobotMap;
import frc.robot.commands.TeleopDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static VictorSP frontLeftMotor = new VictorSP(RobotMap.frontLeftMotor);
  private static VictorSP frontRightMotor = new VictorSP(RobotMap.frontRightMotor);
  private static VictorSP backLeftMotor = new VictorSP(RobotMap.backLeftMotor);
  private static VictorSP backRightMotor = new VictorSP(RobotMap.backRightMotor);
  
  private static MecanumDrive robotDrive;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new TeleopDrive());
    robotDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  }

  public void drive(double x, double y, double turn) {
    robotDrive.driveCartesian(y, x, turn);
  }

}
