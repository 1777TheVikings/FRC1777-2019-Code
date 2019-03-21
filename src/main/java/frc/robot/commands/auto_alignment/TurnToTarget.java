/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto_alignment;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.commands.led.FlashBlue;

/**
 * Add your docs here.
 */
public class TurnToTarget extends PIDCommand {
    private ArrayList<Double> previousValues = new ArrayList<>();

    public TurnToTarget() {
        // gives p, i, and d values for the PID subystem used in vision processing
        super(0.015, 0.00, 0.0);
        requires(Robot.driveTrain);
        setTimeout(3.0);  // failsafe
    }

    @Override
    protected void initialize() {
        super.initialize();

        setSetpoint(0);
        // DriverStation.reportWarning("End angle: " + endAngle, false);
    }

    @Override
    protected double returnPIDInput() {
        //gets necessary angle in order to align robot with vision tape
        double angle = -Robot.jetson.getAngle();
        if (previousValues.size() >= 10)
            previousValues.remove(0);
        previousValues.add(angle);
        return angle;
    }

    @Override
    protected void usePIDOutput(double output) {
        Robot.driveTrain.drive(-Robot.m_oi.getDriveY(), 0, output);
    }

    @Override
    protected boolean isFinished() {
        // Commented code can be used if you want to use the Pigeon as feedback. Written by Colby Gallup - the master programmer.
        // System.out.println("Last error: " + Robot.jetson.getAngle());

        // double sum = 0;
        // for (Double val : previousValues)
        //     sum += val;
        // double averageError = sum / previousValues.size();

        // if (Math.abs(endAngle - averageError) < 0.4){
        if ((Math.abs(Robot.jetson.getAngle()) < 0.4) || isTimedOut()){
            // DriverStation.reportWarning("Finished" , false);
            if (Robot.lightDrive.getCurrentCommand() != null) {
                Robot.lightDrive.getCurrentCommand().cancel();
              }
            Command command = new FlashBlue();
            command.start();
            return true;
        }
        return false;
    }
}
