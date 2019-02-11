package frc.robot.vision;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.Socket;
import java.net.SocketTimeoutException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Jetson;

public class JetsonVision implements Runnable {
    private Thread thread;
    private String threadName;

    private Socket socket;
    private BufferedReader in;

    public JetsonVision(String name) {
        threadName = name;
        System.out.println("Creating " + threadName);
    }

    public void run() {
        System.out.println("Running " + threadName);
        try {
            boolean connected = false;
            SmartDashboard.putBoolean("Jetson connected", false);
            
            while (!connected) {
                try {
                    socket = new Socket(Jetson.ADDRESS, Jetson.PORT);
                    socket.setSoTimeout(10);
                    in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                    connected = true;
                    SmartDashboard.putBoolean("Jetson connected", true);
                } catch (ConnectException ex) {
                    connected = false;
                    SmartDashboard.putBoolean("Jetson connected", false);
                    Thread.sleep(5000);
                } catch (IOException ex) {
                    connected = false;
                    SmartDashboard.putBoolean("Jetson connected", false);
                    DriverStation.reportWarning("IOException when connecting to Jetson", ex.getStackTrace());
                    Thread.sleep(5000);
                }
            }

            // System.out.println("Connected and running");

            while (true) {
                // System.out.println("tick");
                try {
                    String inLine = in.readLine();
                    if (inLine != null) {
                        // System.out.println("detected");
                        Robot.jetson.setAngle(Double.valueOf(inLine));
                    }
                } catch (SocketTimeoutException ex) {
                } catch (IOException ex) {
                    DriverStation.reportWarning("Jetson disconnected", ex.getStackTrace());
                }
                // System.out.println("tock");
                Thread.sleep(10);
            }
        } catch (InterruptedException ex) {
            System.out.println("Thread " + threadName + " was interrupted");
        }
        System.out.println("Thread " + threadName + " is exiting");
        SmartDashboard.putBoolean("Jetson connected", false);
    }

    public void start() {
        System.out.println("Starting " + threadName);
        if (thread == null) {
            thread = new Thread(this, threadName);
            thread.start();
        }
    }
}