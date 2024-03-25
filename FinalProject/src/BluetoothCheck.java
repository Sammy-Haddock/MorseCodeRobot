import lejos.robotics.subsumption.Behavior;
import lejos.hardware.Bluetooth;
import java.io.IOException;


public class BluetoothCheck implements Behavior {
    private boolean messageReceived = false;
    private boolean suppressed = false;
    private String address = null;


    public boolean takeControl() {
        if (address == null) {
			address = Bluetooth.getLocalDevice().getBluetoothAddress();
		}
		if (address.length() > 0) {
		    return true;
		}
        return false;
    }

    public void action() {
        suppressed = false;
        messageReceived = true; // Set state variable to indicate message received


        // Wait for the message to be processed or suppressed
        while (!suppressed && messageReceived) {
            Thread.yield(); // Allow other threads to run
        }
    }


    public void suppress() {
        suppressed = true;
    }


    // Method to check if a Bluetooth message has been received
    public boolean isMessageReceived() {
        return messageReceived;
    }


    // Method to reset the state variable after processing
    public void resetMessageReceived() {
        messageReceived = false;
    }
}





