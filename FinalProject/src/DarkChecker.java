import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.Port;
import lejos.robotics.subsumption.Behavior;
import lejos.robotics.navigation.MovePilot;

public class SpeedControlBehavior implements Behavior {
    private boolean suppressed = false;
    private MovePilot pilot;
    private EV3ColorSensor colorSensor;
    private float[] sample;

    public SpeedControlBehavior(MovePilot pilot, Port sensorPort) {
        this.pilot = pilot;
        this.colorSensor = new EV3ColorSensor(sensorPort);
        this.sample = new float[colorSensor.sampleSize()];
    }

    // Returns true if the robot is going fast and the light level is below a certain level
    public boolean takeControl() {
        float linearVelocity = pilot.getLinearVelocity();
        colorSensor.fetchSample(sample, 0);
        float lightLevel = sample[0]; // assuming light sensor measures light level
        return (linearVelocity > 100 && lightLevel < 0.5); // adjust threshold values as needed
    }

    // Just notifies the action method by setting a field to true
    public void suppress() {
        suppressed = true;
    }

    // Use setLinearSpeed based on certain conditions
    public void action() {
        suppressed = false;
        // Set desired speed based on conditions
        pilot.setLinearSpeed(200); // Example speed setting, adjust as needed
    }
}
