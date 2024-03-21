import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.subsumption.Behavior;
import lejos.robotics.navigation.MovePilot;
import java.util.Random;

public class Backup implements Behavior {
    private boolean suppressed = false;
    private EV3UltrasonicSensor ultrasonicSensor;
    private float[] sample;
    private MovePilot pilot;
    private Random random;

    public Backup(Port sensorPort, MovePilot pilot) {
    	this.ultrasonicSensor = new EV3UltrasonicSensor(sensorPort);
        this.sample = new float[ultrasonicSensor.sampleSize()];
        this.pilot = pilot;
        this.random = new Random();
    }

    // Checks if the wall is too close
    public boolean takeControl() {
        ultrasonicSensor.fetchSample(sample, 0);
        float distance = sample[0] * 100; // Convert distance to centimeters
        return distance < 20; // Return true if the distance is less than 20cm
    }

    // Notifies the action method to exit
    public void suppress() {
        suppressed = true;
    }

    // Goes backwards 20cm and turns either left or right at random
    public void action() {
        suppressed = false;
        // Move backwards 20cm
        pilot.travel(-20);
        // Turn left or right at random
        if (random.nextBoolean()) {
            pilot.rotate(-90); // Turn left
        } else {
            pilot.rotate(90); // Turn right
        }
    }
}
