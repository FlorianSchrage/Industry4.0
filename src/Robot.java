import java.io.IOException;
import java.io.StringReader;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Random;
import java.util.concurrent.Semaphore;

import lejos.hardware.BrickFinder;
import lejos.hardware.BrickInfo;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Robot {
	
// Constants:
	// Statuses:
	private final String STATUS_INITIALIZING = "INITIALIZING";
	private final String STATUS_IDLE = "IDLE";
	private final String STATUS_MOVING_UP = "MOVING_UP";
	private final String STATUS_MOVING_DOWN = "MOVING_DOWN";
	private final String STATUS_MOVING_LEFT = "MOVING_LEFT";
	private final String STATUS_MOVING_RIGHT = "MOVING_RIGHT";
	private final String STATUS_GRABBING = "GRABBING";
	private final String STATUS_RELEASING = "RELEASING";
	private final String STATUS_FAILED = "FAILED";
		
	// Actions:
	private final String[] actions = {
			"GRAB", 
			"MOVE_TO_DELIVERY_PLACE", 
			"MOVE_TO_DISCHARGE_CHUTE", 
			"MOVE_TO_DRIVING_POSITION", 
			"MOVE_TO_GRIPPING_POSITION",
			"MOVE_TO_BUILDING_SITE",
			"MOVE_TO_SOURCE",
			"MOVE_TO_STORAGE_LOCATION",
			"RELEASE",
			"PRESS_TIGHT",
			"MOVE_TO_BUIDING_POSITION",
			"MOVE_TO_OOO_PLACE"};

	// Variables:
	private String position;
	private String currentStatus;
	private String currentBrickColor;	
	private int id;  // 1 = SortRobot  2 = BuildRobot
	private Random randomGenerator = new Random();
	private boolean randomFailureEnabled = false;
		
	// Bridge Coordinates:
	private float coord_outOfOrderPlace_1;
	private float coord_source;
	private float coord_storageLocation_1;
	private float coord_storageLocation_2;
	private float coord_storageLocation_3;
	private float coord_storageLocation_4;
	private float coord_dischargeChute;
	private float coord_deliveryPlace;
	private float corrd_buildingSite;
	private float coord_outOfOrderPlace_2;
		
	// Height Coordinates:
	private final float height_drivingPosition = (float)0.13;
	private final float height_grippingPosition = (float)0.04;
	
	// Motors:
	private final UnregulatedMotor motorA = new UnregulatedMotor(MotorPort.A);
	private final UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.B);	
	private final UnregulatedMotor motorC = new UnregulatedMotor(MotorPort.C);
		
	// Sensors:
	private final EV3UltrasonicSensor sensor_distance_leftRight = new EV3UltrasonicSensor(SensorPort.S1);
	private final EV3UltrasonicSensor sensor_distance_upDown = new EV3UltrasonicSensor(SensorPort.S2);
	private final EV3ColorSensor sensor_color_brick = new EV3ColorSensor(SensorPort.S3);
	private final EV3ColorSensor sensor_color_initialize = new EV3ColorSensor(SensorPort.S4);
		
	// Sample Providers
	private final SampleProvider provider_distance_leftRight = sensor_distance_leftRight.getDistanceMode();
	private float[] sample_distance_leftRight = new float[provider_distance_leftRight.sampleSize()];
		
	private final SampleProvider provider_distance_upDown = sensor_distance_upDown.getDistanceMode();
	private float[] sample_distance_upDown = new float[provider_distance_leftRight.sampleSize()];
		
	private final SampleProvider provider_color_brick = sensor_color_brick.getColorIDMode();
	private float[] sample_color_brick = new float[provider_color_brick.sampleSize()];
		
	private final SampleProvider provider_color_initialize = sensor_color_initialize.getColorIDMode();
	private float[] sample_color_initialize = new float[provider_color_initialize.sampleSize()];
	
	public static final String HOST = "192.168.1.10";
	
	private static final boolean DEBUG_MODE = false;
//	private static final int PORT = 12345;
	
	private SenderThread sender;
	private ReceiverThread receiver;
//	private Socket socket;
	
	public static void main(String[] args) {
		
		Sound.setVolume(5);
		Sound.systemSound(true, 2);
		
		Robot robot = new Robot();
		robot.startWorking();
	}
	
	public void startWorking()
	{
		if(getAddress().equals("0.0.0.2")){
			initializeSortRobot();
		}
		else if (getAddress().equals("0.0.0.3")){
			initializeBuildRobot();
		}
		
		Semaphore sema = new Semaphore(1);
		
		sender = new SenderThread(this, sema);
		receiver = new ReceiverThread(this, sema);
		
		sender.start();
		receiver.start();
	}
	
	// ---------------------- HELPER FUNCTIONS ----------------------
	
	private String getAddress(){
		String address = "";
		try {
			address = InetAddress.getLocalHost().getHostAddress();
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return address;
	}
	
	private float getLeftRightDistance(){
		provider_distance_leftRight.fetchSample(sample_distance_leftRight, 0);
		return sample_distance_leftRight[0];
	}
	
	private float getUpDownDistance(){
		provider_distance_upDown.fetchSample(sample_distance_upDown, 0);
		return sample_distance_upDown[0];
	}
	
	private float returnNextPointOfInterestCoord(){
		provider_color_initialize.fetchSample(sample_color_initialize, 0);
		float firstColor = sample_color_initialize[0];
		while(true){
			provider_color_initialize.fetchSample(sample_color_initialize, 0);
			if(firstColor != sample_color_initialize[0] && (sample_color_initialize[0] == 0 || sample_color_initialize[0] == 2)){
				motorC.stop();
				currentStatus = STATUS_IDLE;
				break;
			}
			if(position == "LEFT"){
			motorC.forward();
			}else if(position == "RIGHT"){
				motorC.backward();
			}
			currentStatus = STATUS_MOVING_LEFT;
		}
		return getLeftRightDistance();
	}
	
	private void moveToCoordinate(float coord){
		float currentCoord;
		if(position == "LEFT"){
			while(true){
				currentCoord = getLeftRightDistance();
				if(Math.round(currentCoord*100 - coord*100) > 0){
					motorC.backward();
					currentStatus = STATUS_MOVING_RIGHT;
				}
				else if(Math.round(currentCoord*100 - coord*100) < 0){
					motorC.forward();
					currentStatus = STATUS_MOVING_LEFT;
				}
				else{
					motorC.stop();
					currentStatus = STATUS_IDLE;
					break;
				}
			}
		}
		else if(position == "RIGHT"){
			while(true){
				currentCoord = getLeftRightDistance();
				if(Math.round(currentCoord*100 - coord*100) > 0){
					motorC.forward();
					currentStatus = STATUS_MOVING_RIGHT;
				}
				else if(Math.round(currentCoord*100 - coord*100) < 0){
					motorC.backward();
					currentStatus = STATUS_MOVING_LEFT;
				}
				else{
					motorC.stop();
					currentStatus = STATUS_IDLE;
					break;
				}
			}
		}
	}
	
	private Boolean moveUpDown(float height){
		motorB.setPower(50);
		float currentCoord;
		while(true){
			currentCoord = getUpDownDistance();
			if(Math.round(currentCoord*100 - height*100) > 0){
				motorB.setPower(50);
				motorB.backward();
				currentStatus = STATUS_MOVING_DOWN;
			}
			else if(Math.round(currentCoord*100 - height*100) < 0){
				motorB.setPower(75);
				motorB.forward();
				currentStatus = STATUS_MOVING_UP;
			}
			else{
				motorB.stop();
				currentStatus = STATUS_IDLE;
				break;
			}
		}
		return true;
	}
	
	// ---------------------- MAIN FUNCTIONS ----------------------
	
	// ----- Initialization Methods -----
	
	/**
	 * Assigns a unique id and position and scans the environment to initialize the coordinates for the sorting robot.
	 * @return boolean indicates that the initialization has finished.
	 */
	public Boolean initializeSortRobot(){
		currentStatus = STATUS_INITIALIZING;
		
		id = 1;
		position = "RIGHT";
		motorC.setPower(25);
		
		coord_outOfOrderPlace_1 = getLeftRightDistance();
		LCD.drawString("ooo" + coord_outOfOrderPlace_1, 0, 0);
		coord_source = returnNextPointOfInterestCoord();
		LCD.clearDisplay();
		LCD.drawString("source" + coord_source, 0, 0);
		coord_storageLocation_1 = returnNextPointOfInterestCoord();
		LCD.clearDisplay();
		LCD.drawString("1st" + coord_storageLocation_1, 0, 0);
		coord_storageLocation_2 = returnNextPointOfInterestCoord();
		LCD.clearDisplay();
		LCD.drawString("2st" + coord_storageLocation_2, 0, 0);
		//coord_storageLocation_3 = returnNextPointOfInterestCoord();
		//coord_storageLocation_4 = returnNextPointOfInterestCoord();
		coord_dischargeChute = returnNextPointOfInterestCoord();
		LCD.clearDisplay();
		LCD.drawString("chute" + coord_dischargeChute, 0, 0);
		coord_deliveryPlace = returnNextPointOfInterestCoord();
		LCD.clearDisplay();
		LCD.drawString("delivery" + coord_deliveryPlace, 0, 0);
		corrd_buildingSite = returnNextPointOfInterestCoord();
		LCD.clearDisplay();
		LCD.drawString("build" + corrd_buildingSite, 0, 0);
		//moveToSource();
		currentStatus = STATUS_IDLE;
		return true;
	}
	
	/**
	 * Assigns a unique id and position and scans the environment to initialize the coordinates for the building robot.
	 * @return boolean indicates that the initialization has finished.
	 */
	public Boolean initializeBuildRobot(){
		currentStatus = STATUS_INITIALIZING;
		
		id = 2;
		position = "LEFT";
		motorC.setPower(25);
		
		coord_outOfOrderPlace_2 = getLeftRightDistance();
		corrd_buildingSite = returnNextPointOfInterestCoord();
		coord_deliveryPlace = returnNextPointOfInterestCoord();
		coord_dischargeChute = returnNextPointOfInterestCoord();
		coord_storageLocation_2 = returnNextPointOfInterestCoord();
		coord_storageLocation_1 = returnNextPointOfInterestCoord();
		coord_source = returnNextPointOfInterestCoord();
		currentStatus = STATUS_IDLE;
		return true;
	}
	
	// ----- Getter Methods -----
	
	/**
	 * Returns the robot ID.
	 * @return id unique robot indicator.
	 */
	public int getID(){
		return id;
	}
	
	/**
	 * Returns the current status of the robot.
	 * @return currentStatus status of the robot.
	 */
	public String getCurrentStatus(){
		return currentStatus;
	}
	
	/**
	 * Returns the position of the robot, indicating if it is the left or right robot on the bridge.
	 * @return position indicates "LEFT" or "RIGHT".
	 */
	public String getPosition(){
		return position;
	}
	
	/**
	 * Returns the color of the brick below the robot hand.
	 * @return currentBrickColor current scanned brick color, e.g. "BLUE" or "NONE".
	 */
	public String getCurrentBrickColor(){
		return currentBrickColor;
	}
	
	// ----- Moving Methods: Left/Right -----
	
	/**
	 * Robot moves to the delivery place.
	 * @return boolean indicates that the robot reached the target.
	 */
	public Boolean moveToDeliveryPlace(){
		moveToCoordinate(coord_deliveryPlace);
		return true;
	}
	
	/**
	 * Robot moves to the discharge chute.
	 * @return boolean indicates that the robot reached the target.
	 */
	public Boolean moveToDischargeChute(){
		moveToCoordinate(coord_dischargeChute);
		return true;
	}
	
	/**
	 * Robot moves to the indicated row of the building site.
	 * @param number of row to which the robot should move.
	 * @return boolean indicates that the robot reached the target.
	 */
	public Boolean moveToBuildingSite(int row){
		//moveToCoordinate(corrd_buildingSite + (float)(row*0.064));
		moveToCoordinate(corrd_buildingSite + (float)(row*0.016));
		return true;
	}
	
	/**
	 * Robot moves to the source.
	 * @return boolean indicates that the robot reached the target.
	 */
	public Boolean moveToSource(){
		moveToCoordinate(coord_source);
		return true;
	}
	
	/**
	 * Robot moves to its out of order place (outOfOrderPlace_1 if id=1, outOfOrderPlace_2 if id=2)
	 * @return boolean indicates that the robot reached the target
	 */
	public Boolean moveToOOOPlace(){
		if(id == 1){
			moveToCoordinate(coord_outOfOrderPlace_1);
		}
		else{
			moveToCoordinate(coord_outOfOrderPlace_2);
		}
		return true;
	}
	
	/**
	 * Robot moves to the storage location with the indicated number
	 * @param storageNumb indicates the number of the storage location
	 * @return boolean indicates that the robot reached the target
	 */
	public Boolean moveToStorageLocation(int storageNumb){
		switch(storageNumb){
		case 0: moveToCoordinate(coord_storageLocation_1); 
				break;
		case 1: moveToCoordinate(coord_storageLocation_2); 
				break; 
		case 2: moveToCoordinate(coord_storageLocation_3); 
				break;
		case 3: moveToCoordinate(coord_storageLocation_4); 
		}
		return true;
	}
	
	// ----- Moving Methods: Up/Down -----
	
	/**
	 * Robot moves to the driving position height
	 * @return boolean indicates that the robot reached the target
	 */
	public Boolean moveToDrivingPosition(){
		moveUpDown(height_drivingPosition);
		return true;
	}
	
	/**
	 * Robot moves to the gripping position height 
	 * @return boolean indicates that the robot reached the target
	 */
	public Boolean moveToGrippingPosition(){
		moveUpDown(height_grippingPosition);
		return true;
	}
	
	/**
	 * Robot moves to the building position height.
	 * @param height indicator for the height.
	 * @return boolean indicates that the robot reached the target.
	 */
	public Boolean moveToBuildingPosition(float height){
		moveUpDown(height);
		return true;
	}
	
	// ----- Robot Hand Methods -----
	
	/**
	 * Closes the robot hand to grab a brick.
	 * @return boolean which indicates that the robot closed the hand.
	 */
	public Boolean grab(){
		currentStatus = STATUS_GRABBING;
		motorA.setPower(70);
		motorA.forward();
		Delay.msDelay(1500);
		motorA.stop();
		setCurrentBrickColor();
		currentStatus = STATUS_IDLE;
		return true;
	}
	
	/**
	 * Opens the robot hand to release a brick.
	 * @return boolean which indicates that the robot opened the hand.
	 */
	public Boolean release(){
		currentStatus = STATUS_RELEASING;
		motorA.setPower(60);
		motorA.backward();
		Delay.msDelay(2000);
		motorA.stop();
		currentStatus = STATUS_IDLE;
		return true;
	}
	
	/**
	 * Closes the hand and moves the hand down to the ground to press a brick against the ground.
	 * @return boolean which indicates that the robot pressed a brick against the ground.
	 */
	public Boolean pressTight(){
		currentStatus = STATUS_GRABBING;
		motorA.setPower(50);
		motorA.forward();
		Delay.msDelay(2500);
		motorA.stop();
		motorB.setPower(100);
		moveUpDown((float)0.039);
		Delay.msDelay(2000);
		currentStatus = STATUS_IDLE;
		return true;
	}

	/**
	 * Scans the current brick which is located below the robot hand and sets the color as currentBrickColor.
	 */
	public void setCurrentBrickColor(){
		provider_color_brick.fetchSample(sample_color_brick, 0);
		switch((int)sample_color_brick[0]){
		case 0: currentBrickColor = "RED";
				break;
		case 1: currentBrickColor = "GREEN";
				break;
		case 2: currentBrickColor = "BLUE";
				break;
		case 3: currentBrickColor = "YELLOW";
				break;
		case 4: currentBrickColor = "MAGENTA";
				break;
		case 5: currentBrickColor = "ORANGE";
				break;
		case 6: currentBrickColor = "WHITE";
				break;
		case 7: currentBrickColor = "BLACK";
				break;
		case 8: currentBrickColor = "PINK";
				break;
		case 9: currentBrickColor = "GRAY";
				break;
		case 10: currentBrickColor = "LIGHT_GRAY";
				break;
		case 11: currentBrickColor = "DARK_GRAY";
				break;
		case 12: currentBrickColor = "CYAN";
				break;
		case 13: currentBrickColor = "BROWN";
				break;
		default: currentBrickColor = null;
		}
	}
	
	// ----- JSON Methods -----
	
	/**
	 * Creates and returns a JSON string containing information about the robot.
	 * @return json A JSON String containing information about POSITION, COLOR and STATUS of the robot.
	 */
	public String getJsonString(){
		String json = "{\"Robot_" + getID() + "\":{\"POSITION\":\"" + getPosition() + "\",\"COLOR\":\"" + getCurrentBrickColor() + "\",\"STATUS\":\"" + getCurrentStatus() + "\",\"IP\":\"" + sender.getSocket().getLocalAddress().getHostAddress() + "\"}}";		
		return json;
	}
	
	/**
	 * Interprets a given JSON String as an action. If randomFailureEnabled is set true, then the robot will fail with a probability of 10% and won't interpret the given JSON String.
	 * @param json JSON String containing an action and optionally a value.
	 */
	public void interpretJsonString(String json){
		if(randomFailureEnabled & (randomGenerator.nextInt() % 10 == 4)){
			currentStatus = STATUS_FAILED;
			stopCommunication();
			moveToOOOPlace();
			Delay.msDelay(120000);
			restartCommunication();
		}
		else{
			String action = "";
			
			for(int i=0; i<actions.length; i++){
				if(json.contains(actions[i])){
					action = actions[i];
				}
			}
			
			int number = -1;
			if(action == "MOVE_TO_BUILDING_SITE" || action == "MOVE_TO_STORAGE_LOCATION" || action == "MOVE_TO_BUIDING_POSITION"){
				number = Integer.valueOf(json.substring(11).replaceAll("[^0-9]+", ""));
			}
			
			switch(action){
			   case "GRAB": grab(); break;
			   case "RELEASE": release(); break;
			   case "PRESS_TIGHT": pressTight(); break;
			   case "MOVE_TO_DELIVERY_PLACE": moveToDeliveryPlace(); break;
			   case "MOVE_TO_DISCHARGE_CHUTE": moveToDischargeChute(); break;
			   case "MOVE_TO_DRIVING_POSITION": moveToDrivingPosition(); break;
			   case "MOVE_TO_GRIPPING_POSITION": moveToGrippingPosition(); break;
			   case "MOVE_TO_BUILDING_SITE": moveToBuildingSite(number); break;			
			   case "MOVE_TO_STORAGE_LOCATION": moveToStorageLocation(number); break;		
			   case "MOVE_TO_BUIDING_POSITION": moveToBuildingPosition(number);	break;	
			   case "MOVE_TO_SOURCE": moveToSource(); break;
			   case "MOVE_TO_OOO_PLACE": moveToOOOPlace();
			}
		}
	}
	
//	public Socket getSocket()
//	{
//		if(socket == null || socket.isClosed() || !socket.isConnected())
//		{
//			LCD.drawString("New Socket created", 1, 1);
//			
//			try
//			{
//				socket = new Socket(HOST, PORT);
//			} catch (UnknownHostException e)
//			{
//				e.printStackTrace();
//			} catch (IOException e)
//			{
//				e.printStackTrace();
//			}
//		}
//		
//		return socket;
//	}

	/**
	 * Display a message on the robot screen.
	 * @param message message to display.
	 */
	public void print(String message)
	{
		LCD.clear();
		LCD.drawString(message, 1, 1);
		if(DEBUG_MODE)
			Delay.msDelay(3000);
		LCD.clear();
	}
	
	/**
	 * Stops the communication with FESAS.
	 */
	private void stopCommunication()
	{
		print("Stopping Communication");
		sender.terminate();
		try
		{
			sender.join();
		} catch (InterruptedException e)
		{
			e.printStackTrace();
		}
		print("Sender terminated");
		receiver.terminate();
		try
		{
			receiver.join();
		} catch (InterruptedException e)
		{
			e.printStackTrace();
		}
		print("Receiver terminated");
	}
	
	/**
	 * Restarts the communication with FESAS.
	 */
	private void restartCommunication()
	{
		print("Restarting Communication");
		Semaphore sema = new Semaphore(1);
		
		sender = new SenderThread(this, sema);
		receiver = new ReceiverThread(this, sema);
		
		sender.start();
		receiver.start();
		print("Threads restarted");
	}
	
	// ---------------------- ONLY FOR TESTING! ----------------------
	
	private void write(){
		LCD.clearDisplay();
		LCD.drawString(" " + coord_deliveryPlace, 0, 0);
	}
	
	private void write2(){
		LCD.clearDisplay();
		LCD.drawString(" " + coord_dischargeChute, 0, 0);
	}
}
