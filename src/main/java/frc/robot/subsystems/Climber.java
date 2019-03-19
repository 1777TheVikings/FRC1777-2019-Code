/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopClimb;
import frc.utils.MB1013;

public class Climber extends Subsystem {
  private static WPI_TalonSRX bottomLeftMotor = new WPI_TalonSRX(RobotMap.climberBottomLeftMotor);  // master
  private static WPI_TalonSRX topLeftMotor = new WPI_TalonSRX(RobotMap.climberTopLeftMotor);
  private static WPI_TalonSRX topRightMotor = new WPI_TalonSRX(RobotMap.climberTopRightMotor);
  private static WPI_TalonSRX bottomRightMotor = new WPI_TalonSRX(RobotMap.climberBottomRightMotor);

  private static WPI_VictorSPX slideMotor = new WPI_VictorSPX(RobotMap.climberSlideMotor);

  private static DigitalInput limitSwitch = new DigitalInput(RobotMap.climberLimitSwitch);

  private static MB1013 distanceSensor = new MB1013(RobotMap.climberDistanceSensor);

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

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public double getHeight() {
    return distanceSensor.getDistance();
  }
}
