import lejos.hardware.Button;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.sensor.NXTSoundSensor;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

public class Driver {
    public static void main(String[] args) {
        final float WHEEL_DIAMETER = 56;
        final float AXLE_LENGTH = 44;
        final float ANGULAR_SPEED = 200;
        final float LINEAR_SPEED = 200;
        final BaseRegulatedMotor mL = new EV3LargeRegulatedMotor(MotorPort.A);
        final BaseRegulatedMotor mR = new EV3LargeRegulatedMotor(MotorPort.B);

        Wheel wLeft = WheeledChassis.modelWheel(mL, WHEEL_DIAMETER).offset(-AXLE_LENGTH / 2);
        Wheel wRight = WheeledChassis.modelWheel(mR, WHEEL_DIAMETER).offset(AXLE_LENGTH / 2);

        Wheel[] wheels = new Wheel[] {wRight, wLeft}; // Corrected the order of wheels

        WheeledChassis chassis = new WheeledChassis(wheels, WheeledChassis.TYPE_DIFFERENTIAL);
        MovePilot pilot = new MovePilot(chassis);
        
    	LCD.drawString("Welcome to Our Morse Code Robot!", 0, 0);
    	LCD.drawString("Authors: Samuel Haddock, Yash Kumar", 0, 1);
    	LCD.drawString("Yash Kumar", 0, 2);
    	LCD.drawString("and Miski Hussein", 0, 3);
    	LCD.drawString("Version: 1.0", 0, 4);
    	LCD.drawString("Press Enter button to continue...", 0, 5);

        Button.ENTER.waitForPress();

        LCD.clear();

        // Initialise the sound sensor
        NXTSoundSensor soundSensor = new NXTSoundSensor(SensorPort.S2);
        SampleProvider soundMode = soundSensor.getDBAMode();
        
        //squareCommand(mL, mR, pilot, LINEAR_SPEED, LINEAR_SPEED);
        //freeRoamCommand(mR, mR, pilot);
        //circleCommand(mL, mR);
        danceCommand(mL, mR);
        
        mL.close();
        mR.close();

        /*Behavior forwardBehavior = new Trundle(pilot);
        Behavior avoidWallBehavior = new Backup(SensorPort.S3, pilot);
        Behavior lowBatteryBehavior = new LowBattery();
        Behavior darkChecker = new DarkChecker(pilot, SensorPort.S1);
        Behavior[] behaviors = {avoidWallBehavior, lowBatteryBehavior, darkChecker};
        Arbitrator arbitrator = new Arbitrator(behaviors);

        while (!Button.ENTER.isDown()) {
            arbitrator.go();
        }

        // Stop the arbitrator and close the sound sensor when the enter button is pressed
        arbitrator.stop();
        soundSensor.close();*/
    }

    public static void squareCommand(BaseRegulatedMotor mL, BaseRegulatedMotor mR, MovePilot pilot, float ANGULAR_SPEED, float LINEAR_SPEED) { // Fixed method name
    	LCD.drawString("Running square command...", 0, 0);
        for (int side = 0; side < 4; side++) {
            pilot.setAngularSpeed(ANGULAR_SPEED);
            pilot.setLinearSpeed(LINEAR_SPEED);
            pilot.travel(500);
            pilot.rotate(275); // Changed the angle to make a square
        }
    }

    public static void circleCommand(BaseRegulatedMotor mL, BaseRegulatedMotor mR) {
    	mL.synchronizeWith(new BaseRegulatedMotor[] {mR});
    	mL.setSpeed(720);
		mR.setSpeed(720);
		  while (!Button.ENTER.isDown()) {	  
			  for (int i = 0; i < 4; i++) {
				  if(Button.ENTER.isDown()) {
					  break;
				  }
				  mL.rotate(360); 
				  mL.startSynchronization();
				  
				  mL.rotate(360);
				  mR.rotate(360);
				  
				  mL.endSynchronization();
				  mL.waitComplete();
				  mR.waitComplete();
			  }
			  mL.close();
			  mR.close();
			  break;
		  }
    }

    public static void freeRoamCommand(BaseRegulatedMotor mL, BaseRegulatedMotor mR, MovePilot pilot ) {
    	Behavior backupBehavior = new Backup(SensorPort.S3, pilot);
	
    	long startTime = System.currentTimeMillis();
        long elapsedTime = 0;

        while (elapsedTime < 60000) {
        	if(Button.ENTER.isDown()) {
        		break;
        	}
            if (backupBehavior.takeControl()) {
                backupBehavior.action();
            } else {
                pilot.forward();
            }

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
		    for (int i = 0; i < 3; i++) {
		    	if(Button.ENTER.isDown()) {
		    		break;
		    	}
			    mL.rotate(360); 
			    mL.startSynchronization();
			    
			    mL.forward();
			    mR.forward();
			    
			    mL.rotate(720);
			    mR.rotate(720);
			    
			    mL.backward();
			    mR.forward();				
			    
			    mL.forward();
			    mR.forward();
			    
			    mL.rotate(540);
			    mR.rotate(360);
			    
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

