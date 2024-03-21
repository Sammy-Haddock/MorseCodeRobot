import lejos.robotics.subsumption.Behavior;
import lejos.hardware.Sound;
import lejos.hardware.Battery;

public class LowBattery implements Behavior {
    private boolean suppressed = false;

    // Returns true if the battery level is low
    public boolean takeControl() {
        return Battery.getBatteryCurrent() < 0.05; // You may need to adjust this threshold based on your battery and system setup
    }

    // Just notifies the action method by setting a field to true
    public void suppress() {
        suppressed = true;
    }

    // Flash the battery low message and beep
    public void action() {
        suppressed = false;
        // Beep to alert user
        Sound.beep();
        // Wait a bit for the message to be visible and the beep to be heard
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
