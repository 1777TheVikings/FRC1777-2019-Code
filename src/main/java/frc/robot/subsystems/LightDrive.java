package frc.robot.subsystems;

import com.mach.LightDrive.*;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.led.StaticYellow;

public class LightDrive extends Subsystem {
    private static Servo servoA = new Servo(RobotMap.lightDriveAPort);
    private static Servo servoB = new Servo(RobotMap.lightDriveBPort);
    
    private static LightDrivePWM lightDrive = new LightDrivePWM(servoA, servoB);

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new StaticYellow());
    }

    public void setColor(Color color) {
        lightDrive.SetColor(1, color);
        lightDrive.SetColor(2, color);
        lightDrive.Update();
    }
}