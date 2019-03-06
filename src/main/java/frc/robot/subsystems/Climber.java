/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopClimb;

public class Climber extends Subsystem {
  public static WPI_TalonSRX bottomLeftMotor = new WPI_TalonSRX(RobotMap.climberBottomLeftMotor);  // master
  public static WPI_TalonSRX topLeftMotor = new WPI_TalonSRX(RobotMap.climberTopLeftMotor);
  public static WPI_TalonSRX topRightMotor = new WPI_TalonSRX(RobotMap.climberTopRightMotor);
  public static WPI_TalonSRX bottomRightMotor = new WPI_TalonSRX(RobotMap.climberBottomRightMotor);

  public static Spark slideMotor = new Spark(RobotMap.climberSlideMotor);

  public Climber() {
    topLeftMotor.follow(bottomLeftMotor);
    topRightMotor.follow(bottomLeftMotor);
    bottomRightMotor.follow(bottomLeftMotor);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopClimb());
  }

  public void setLift(double value) {
    bottomLeftMotor.set(ControlMode.PercentOutput, value);
  }

  public void setSlide(double value) {
    slideMotor.set(value);
  }
}
