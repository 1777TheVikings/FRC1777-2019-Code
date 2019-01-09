/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static TalonSRX frontLeftMotor = new TalonSRX(RobotMap.frontLeftMotor);
  private static TalonSRX frontRightMotor = new TalonSRX(RobotMap.frontRightMotor);
  private static TalonSRX backLeftMotor = new TalonSRX(RobotMap.backLeftMotor);
  private static TalonSRX backRightMotor = new TalonSRX(RobotMap.backRightMotor);
  
  private static MecanumDrive robotDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void drive(double y, double x, double turn) {
    robotDrive.driveCartesian(y, x, turn);
  }

}
