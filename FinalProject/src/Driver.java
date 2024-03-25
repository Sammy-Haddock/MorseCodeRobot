import java.util.ArrayList;
import java.util.HashMap;

import lejos.hardware.Battery;
import lejos.hardware.Button;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.sensor.NXTSoundSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

public class Driver {
    public static void main(String[] args) throws InterruptedException {
        final float WHEEL_DIAMETER = 56;
        final float AXLE_LENGTH = 44;
        final float ANGULAR_SPEED = 200;
        final float LINEAR_SPEED = 200;
        final BaseRegulatedMotor mL = new EV3LargeRegulatedMotor(MotorPort.A);
        final BaseRegulatedMotor mR = new EV3LargeRegulatedMotor(MotorPort.B);
        final HashMap<String, String> morseAlphabet = new HashMap<>();
        
        Wheel wLeft = WheeledChassis.modelWheel(mL, WHEEL_DIAMETER).offset(-AXLE_LENGTH / 2);
        Wheel wRight = WheeledChassis.modelWheel(mR, WHEEL_DIAMETER).offset(AXLE_LENGTH / 2);

        Wheel[] wheels = new Wheel[] {wRight, wLeft}; // Corrected the order of wheels

        WheeledChassis chassis = new WheeledChassis(wheels, WheeledChassis.TYPE_DIFFERENTIAL);
        final MovePilot pilot = new MovePilot(chassis);
        
    	LCD.drawString("Welcome to Our Morse Code Robot!", 0, 0);
    	LCD.drawString("Authors: Samuel Haddock, Yash Kumar", 0, 1);
    	LCD.drawString("Yash Kumar", 0, 2);
    	LCD.drawString("and Miski Hussein", 0, 3);
    	LCD.drawString("Version: 1.0", 0, 4);
    	LCD.drawString("Press Enter button to continue...", 0, 5);

        Button.ENTER.waitForPress();

        LCD.clear();
        
        NXTSoundSensor ss = new NXTSoundSensor(SensorPort.S2);
        SensorMode sound = (SensorMode) ss.getDBAMode();
        SoundDetection clap = new SoundDetection(sound, 0.6f, 200, 500);

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
        
        float[] level = new float[1];
        StringBuilder morseWord = new StringBuilder(); // List to store dots and dashes
        StringBuilder commandWord = new StringBuilder();

        boolean dashInProgress = false; // Track whether a dash is in progress
        LCD.drawString("Morse Code Heard:", 0, 0);
        while (true) {
            if (Button.LEFT.isDown()) {
                break;
            }
            
            if (Button.UP.isDown()) {
                if (morseWord.length() > 0) {
                    morseWord.deleteCharAt(morseWord.length() - 1); // Delete the last character
                    LCD.clear();
                    LCD.drawString("Morse Code Heard:", 0, 0);
                    LCD.drawString(commandWord.toString(), 0, 2);
                    LCD.drawString(morseWord.toString(), 0, 1);
                }
            }
                           	
            clap.fetchSample(level, 0);
            if (Button.ENTER.isDown()){
                // If no sound is detected for 3 seconds, save all the dots and dashes as a word
                for(String key : morseAlphabet.keySet()) {
                    if(key.equals(morseWord.toString())) {                 
                    	commandWord.append(morseAlphabet.get(key));
                    }
                }
                
                LCD.clear();
                morseWord.setLength(0);// Clear the list for the next word
                LCD.drawString("Morse Code Heard:", 0, 0);
                LCD.drawString(commandWord.toString(),0,2);
                LCD.drawString(morseWord.toString(), 0, 1);
                Thread.sleep(1000);
            }

            if (level[0] == 1.0) {
                // Single clap (potential dot)
                Thread.sleep(300); // Wait for potential second clap within dash time gap
                clap.fetchSample(level, 0); // Fetch sample again
                if (level[0] == 2.0) {
                    int length = morseWord.length();
                    if (length == 0) {
                        morseWord.setLength(0);
                        morseWord.append("-"); // Add dash to list
                        LCD.drawString(morseWord.toString(), 0, 1);
                    } else {
                        morseWord.replace(length - 1, length, "-"); // Replace last character with "-"
                        LCD.drawString(morseWord.toString(), 0, 1);
                    }
                    dashInProgress = true; // Set dash in progress when a dash is detected
                } else {
                    morseWord.append("."); // Add dot to list
                    LCD.drawString(morseWord.toString(), 0, 1);
                    dashInProgress = false; // Reset dash in progress
                } 
            } else if (level[0] == 2.0 && !dashInProgress) {
                int length = morseWord.length();
                if (length == 0) {
                    morseWord.setLength(0);
                    morseWord.append("-"); // Add dash to list
                    LCD.drawString(morseWord.toString(), 0, 1);
                } else {
                    morseWord.replace(length - 1, length, "-"); // Replace last character with "-"
                    LCD.drawString(morseWord.toString(), 0, 1);
                }
                dashInProgress = true; // Set dash in progress when a dash is detected
            } else {
                dashInProgress = false; // Reset dash in progress if no clap is detected
            }
        }
        
        String strCommandWord = commandWord.toString();
        LCD.clear();
        LCD.drawString(strCommandWord, 0, 0);
        Thread.sleep(2000);
        LCD.clear();

        Thread squareThread = null;
        Thread freeRoamThread = null;
        Thread circleThread = null;
        Thread danceThread = null;

        Thread backupThread = new Thread(new Runnable() {
            @Override
            public void run() {
                new Backup(SensorPort.S3, pilot).action();
            }
        });
        backupThread.start();


        Thread lowBatteryThread = new Thread(new Runnable() {
            @Override
            public void run() {
                new LowBattery().action();
            }
        });
        lowBatteryThread.start();


        Thread darkThread = new Thread(new Runnable() {
            @Override
            public void run() {
                new DarkChecker(pilot, SensorPort.S1).action();
            }
        });
        darkThread.start();
        
        Thread BluetoothThread = new Thread(new Runnable() {
            @Override
            public void run() {
                new BluetoothCheck().action();
            }
        });
        BluetoothThread.start();

        
        Thread.sleep(5000);
        if (strCommandWord.equals("SQ")) {
            squareThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    squareCommand(mL, mR, pilot, ANGULAR_SPEED, LINEAR_SPEED);
                }
            });
            squareThread.start();
        } else if (strCommandWord.equals("FR")) {
            freeRoamThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    freeRoamCommand(mR, mR, pilot);
                }
            });
            freeRoamThread.start();
        } else if (strCommandWord.equals("CI")) {
            circleThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        circleCommand(mL, mR);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            });
            circleThread.start();
        } else if (strCommandWord.equals("DA")) {
            danceThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    danceCommand(mL, mR);
                }
            });
            danceThread.start();
        } else {
            System.out.print("No Command found!");
        }

        // Wait for the corresponding thread to finish
        if (squareThread != null) {
            squareThread.join();
        } else if (freeRoamThread != null) {
            freeRoamThread.join();
        } else if (circleThread != null) {
            circleThread.join();
        } else if (danceThread != null) {
            danceThread.join();
        }
        
        
        mL.close();
        mR.close();
        
        System.exit(0);
    }

    public static void squareCommand(BaseRegulatedMotor mL, BaseRegulatedMotor mR, MovePilot pilot, float ANGULAR_SPEED, float LINEAR_SPEED) { // Fixed method name
    	LCD.drawString("Running square command...", 0, 0);
        for (int side = 0; side < 4; side++) {
        	if(Button.ENTER.isDown()) {
			    break;
		    }
            pilot.setAngularSpeed(ANGULAR_SPEED);
            pilot.setLinearSpeed(LINEAR_SPEED);
            pilot.travel(500);
            pilot.rotate(280); // Changed the angle to make a square
        }
    }

    public static void circleCommand(BaseRegulatedMotor mL, BaseRegulatedMotor mR) throws InterruptedException {
        mL.synchronizeWith(new BaseRegulatedMotor[] {mR});
        mL.setSpeed(1080);
        mR.setSpeed(360);
        
        // Start the motors
        mL.forward();
        mR.forward();

        // Set the start time
        long startTime = System.currentTimeMillis();

        // Run the motors for 4 seconds
        while (System.currentTimeMillis() - startTime < 4000) {
            if (Button.ENTER.isDown()) {
                break;
            }
        }

        // Stop the motors
        mL.stop();
        mR.stop();

        mL.close();
        mR.close();
    }

    public static void freeRoamCommand(BaseRegulatedMotor mL, BaseRegulatedMotor mR, MovePilot pilot ) {
    	LCD.drawString("Running free roam command...", 0, 0);
	
    	long startTime = System.currentTimeMillis();
        long elapsedTime = 0;

        while (elapsedTime < 20000) {
        	if(Button.ENTER.isDown()) {
        		break;
        	}
            pilot.forward();
            elapsedTime = System.currentTimeMillis() - startTime;
        }
        
        pilot.stop();

        mL.close();
        mR.close();
    }

    public static void danceCommand(BaseRegulatedMotor mL, BaseRegulatedMotor mR) {
	    LCD.drawString("Running dance command...", 0, 0);
	    
	    mL.synchronizeWith(new BaseRegulatedMotor[] {mR});
	    mL.setSpeed(600);
	    mR.setSpeed(600);
	    
	    while (!Button.ENTER.isDown()) {
		    for (int i = 0; i < 5; i++) {
		    	if(Button.ENTER.isDown()) {
		    		break;
		    	}
			    mL.rotate(360); 
			    mL.startSynchronization();
			    
			    
			    mL.rotate(720);
			    mR.rotate(720);
			    
			    mL.backward();
			    mR.forward();				
			    
			    
			    mL.rotate(540);
			    mR.rotate(360);
			    
			    mL.rotate(720);
			    mR.rotate(1080);


			    mL.rotate(600);
			    mR.rotate(900);

			    
			    mL.rotate(1080);
			    mR.rotate(540);

			    mL.rotate(720);
			    mR.rotate(1080);
			    
			    
			    mL.endSynchronization();
			    mL.waitComplete();
			    mR.waitComplete();
		    }	
		    mL.close();
		    mR.close();
		    break;
	    }
    }
}

