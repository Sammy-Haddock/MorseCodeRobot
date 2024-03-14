import lejos.hardware.Button;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTSoundSensor;
import lejos.hardware.sensor.SensorMode;
import java.util.ArrayList;
import java.util.HashMap;

public class TestMorseCode {
    public static void main(String[] args) throws InterruptedException {
    	final HashMap<String, String> morseAlphabet = new HashMap<>();
    	morseAlphabet.put(".-", "A");
        morseAlphabet.put("-...", "B");
        morseAlphabet.put("-.-.", "C");
        morseAlphabet.put("-..", "D");
        morseAlphabet.put(".", "E");
        morseAlphabet.put("..-.", "F");
        morseAlphabet.put("--.", "G");
        morseAlphabet.put("....", "H");
        morseAlphabet.put("..", "I");
        morseAlphabet.put(".---", "J");
        morseAlphabet.put("-.-", "K");
        morseAlphabet.put(".-..", "L");
        morseAlphabet.put("--", "M");
        morseAlphabet.put("-.", "N");
        morseAlphabet.put("---", "O");
        morseAlphabet.put(".--.", "P");
        morseAlphabet.put("--.-", "Q");
        morseAlphabet.put(".-.", "R");
        morseAlphabet.put("...", "S");
        morseAlphabet.put("-", "T");
        morseAlphabet.put("..-", "U");
        morseAlphabet.put("...-", "V");
        morseAlphabet.put(".--", "W");
        morseAlphabet.put("-..-", "X");
        morseAlphabet.put("-.--", "Y");
        morseAlphabet.put("--..", "Z");

        BaseRegulatedMotor mLeft = new EV3LargeRegulatedMotor(MotorPort.A);
        BaseRegulatedMotor mRight = new EV3LargeRegulatedMotor(MotorPort.B);

        NXTSoundSensor ss = new NXTSoundSensor(SensorPort.S2);
        SensorMode sound = (SensorMode) ss.getDBAMode();
        SoundDetection clap = new SoundDetection(sound, 0.6f, 200, 500);

        float[] level = new float[1];
        StringBuilder morseList = new StringBuilder(); // List to store dots and dashes
        ArrayList<String> letterList = new ArrayList<>();

        boolean dashInProgress = false; // Track whether a dash is in progress
        System.out.print("Morse Code Heard:");
        while (true) {
            if (Button.ESCAPE.isDown()) {
            	break;
            }

            clap.fetchSample(level, 0);
            if (Button.ENTER.isDown()){
                // If no sound is detected for 3 seconds, save all the dots and dashes as a word
                for(String key : morseAlphabet.keySet()) {
                	if(morseAlphabet.get(key) == morseList.toString()) {
                		letterList.add(key);
                	}
                }
                morseList.setLength(0); // Clear the list for the next word
            }

            if (level[0] == 1.0) {
                // Single clap (potential dot)
                Thread.sleep(300); // Wait for potential second clap within dash time gap
                clap.fetchSample(level, 0); // Fetch sample again
                if (level[0] == 2.0) {
                    morseList.append("-"); // Add dash to list
                    dashInProgress = true; // Set dash in progress when a dash is detected
                } else {
                    morseList.append("."); // Add dot to list
                    dashInProgress = false; // Reset dash in progress
                }
            } else if (level[0] == 2.0 && !dashInProgress) {
                // Only detect dash if not already in progress
                morseList.append("-"); // Add dash to list
                dashInProgress = true; // Set dash in progress when a dash is detected
            } else {
                dashInProgress = false; // Reset dash in progress if no clap is detected
            }
            
       
        }
        for(String letter : letterList) {
        	System.out.print(letter);
        }
        
        mLeft.close();
        mRight.close();
    }

}