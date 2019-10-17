/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class PressureSwitch {
    private static AnalogInput analogIn = new AnalogInput(RobotMap.pressureSwitchIn);

    public double getPressure() {
        return (250 * (analogIn.getVoltage() / 5.0)) - 20;
    }
}
