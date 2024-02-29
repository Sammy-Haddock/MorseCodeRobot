import lejos.hardware.motor.Motor;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.port.SensorPort;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.hardware.Button;

public class Driver {
    public static void main(String[] args) {

        System.out.println("Welcome to Our Morse Code Robot!");
    	System.out.println("Authors: Samuel Haddock, Yash Kumar and Miski Hussein");
    	System.out.println("Version: 1.0");

    	System.out.println("Press Enter button to continue...");
    	Button.ENTER.waitForPress();
        
        @SuppressWarnings("deprecation")
		MovePilot pilot = new MovePilot(5.6f, 12.0f, Motor.A, Motor.B);

        Behavior forwardBehavior = new Trundle(pilot);
        Behavior avoidWallBehavior = new Backup(SensorPort.S3, pilot);
        Behavior lowBatteryBehavior = new LowBattery();

        Behavior[] behaviors = {forwardBehavior, avoidWallBehavior, lowBatteryBehavior};

        Arbitrator arbitrator = new Arbitrator(behaviors);
        
        Button.ESCAPE.addButtonListener(new ButtonListener() {
            public void buttonPressed(Button b) {
                System.exit(0);
            }

            public void buttonReleased(Button b) {
                // Do nothing on release
            }
        });

	arbitrator.go()  
    }
}





