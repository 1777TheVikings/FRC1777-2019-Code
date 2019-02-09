package frc.robot.vision;

public class JetsonVision implements Runnable {
    private Thread thread;
    private String threadName;

    public JetsonVision(String name) {
        threadName = name;
        System.out.println("Creating " + threadName);
    }

    public void run() {
        System.out.println("Running " + threadName);
        try {
            while (true) {
                System.out.println("Running!");
                Thread.sleep(1000);
            }
        } catch (InterruptedException ex) {
            System.out.println("Thread " + threadName + " was interrupted");
        }
        System.out.println("Thread " + threadName + " is exiting");
    }

    public void start() {
        System.out.println("Starting " + threadName);
        if (thread == null) {
            thread = new Thread(this, threadName);
            thread.start();
        }
    }
}