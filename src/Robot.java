import java.io.IOException;
import java.io.InputStream;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Properties;
import java.util.Random;
import java.util.concurrent.Semaphore;
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
			"MOVE_TO_BUILDING_POSITION",
			"MOVE_TO_OOO_PLACE"
	};

	//Colors:
	private final List<String> colors = Arrays.asList(
		"RED",
		"GREEN",
		"BLUE",
		"YELLOW",
		"MAGENTA",
		"ORANGE",
		"WHITE",
		"BLACK",
		"PINK",
		"GRAY",
		"LIGHT_GRAY",
		"DARK_GRAY",
		"CYAN",
		"YELLOW"
	);
	private int color_initialization_1;
	private int color_initialization_2;
	
	// Horizontal Places:
	private List<String> places_horizontal;
	private HashMap<String, Float> coords_places_horizontal = new HashMap<String, Float>();
	
	// General unique or current information:
	private String position;
	private String currentStatus;
	private String currentBrickColor;	
	private int id;  // 1 = SortRobot  2 = BuildRobot
	
	// Simulated Failure:
	private Random randomGenerator = new Random();
	private boolean randomFailureEnabled;
	private int randomFailure_parameter;
	
	// Predefined Properties from Properties File:
	private Properties properties = new Properties();
		
	// Vertical Coordinates:
	private float height_initializingPosition_1;
	private float height_initializingPosition_2;
	private float height_drivingPosition;
	private float height_grippingPosition;
	
	// Building Parameters
	private float buildingPosition_height;
	private float buildingSite_width;
	
	// Motors:
	private final UnregulatedMotor motor_robotHand = new UnregulatedMotor(MotorPort.A);
	private final UnregulatedMotor motor_vertical = new UnregulatedMotor(MotorPort.B);	
	private final UnregulatedMotor motor_horizontal = new UnregulatedMotor(MotorPort.C);
	
	// Power:
	private int power_motor_horizontal_1;
	private int power_motor_horizontal_2;
	private int power_motor_horizontal_initialization_1;
	private int power_motor_horizontal_initialization_2;
	private int power_motor_vertical_up;
	private int power_motor_vertical_down;
	private int power_motor_robotHand_open;
	private int power_motor_robotHand_close;
	
	// Delays:
	private int delay_initialize;
	private int delay_robotHand_open;
	private int delay_robotHand_close;
	private int delay_restartCommunicationAfterFailure;
	private int delay_sending;
		
	// Sensors:
	private final EV3UltrasonicSensor sensor_distance_horizontal = new EV3UltrasonicSensor(SensorPort.S1);
	private final EV3UltrasonicSensor sensor_distance_vertical = new EV3UltrasonicSensor(SensorPort.S4);
	private final EV3ColorSensor sensor_color_brick = new EV3ColorSensor(SensorPort.S2);
	private final EV3ColorSensor sensor_color_initialize = new EV3ColorSensor(SensorPort.S3);
		
	// Sample Providers
	private final SampleProvider provider_distance_horizontal = sensor_distance_horizontal.getDistanceMode();
	private float[] sample_distance_horizontal = new float[provider_distance_horizontal.sampleSize()];
		
	private final SampleProvider provider_distance_vertical = sensor_distance_vertical.getDistanceMode();
	private float[] sample_distance_vertical = new float[provider_distance_horizontal.sampleSize()];
		
	private final SampleProvider provider_color_brick = sensor_color_brick.getColorIDMode();
	private float[] sample_color_brick = new float[provider_color_brick.sampleSize()];
		
	private final SampleProvider provider_color_initialize = sensor_color_initialize.getColorIDMode();
	private float[] sample_color_initialize = new float[provider_color_initialize.sampleSize()];
	
	// Local IPs
	private String ip_local_robot_right;
	private String ip_local_robot_left;
	
	// Communication
	public static String ip_host;
	private boolean debug_mode;	
	private SenderThread sender;
	private ReceiverThread receiver;
	Semaphore semaphore;
	
	
	
	public static void main(String[] args) {
		Sound.setVolume(5);
		Sound.systemSound(true, 2);
		
		Robot robot = new Robot();
		robot.startWorking();
	}
	
	// ---------------------- HELPER FUNCTIONS ----------------------
	
	/**
	 * Reads the user defined properties file.
	 */
	private void readProperties(){
		InputStream input = null;

    	try {
    		String filename = "robot.properties";
    		input = Robot.class.getClassLoader().getResourceAsStream(filename);
    		if(input==null){
    	        System.out.println("Sorry, unable to find " + filename);
    		    return;
    		}

    		properties.load(input);

    	} catch (IOException ex) {
    		ex.printStackTrace();
        } finally{
        	if(input!=null){
        		try {
				input.close();
        		} 
        		catch (IOException e) {
				e.printStackTrace();
        		}
        	}
        }
	}
	
	/**
	 * Initializes variables with information from the properties file.
	 */
	private void initializePropertiesBasedVariables(){
		readProperties();

		color_initialization_1 = colors.indexOf(properties.getProperty("color_initialization_1"));
		color_initialization_2 = colors.indexOf(properties.getProperty("color_initialization_2"));
		places_horizontal = Arrays.asList(properties.getProperty("places_horizontal").split(","));
		
		height_initializingPosition_1 = Float.valueOf(properties.getProperty("height_initializingPosition_1"));
		height_initializingPosition_2 = Float.valueOf(properties.getProperty("height_initializingPosition_2"));
		height_drivingPosition = Float.valueOf(properties.getProperty("height_drivingPosition"));
		height_grippingPosition = Float.valueOf(properties.getProperty("height_grippingPosition"));
		
		buildingPosition_height = Float.valueOf(properties.getProperty("buildingPosition_height"));
		buildingSite_width = Float.valueOf(properties.getProperty("buildingSite_width"));
		
		ip_host = properties.getProperty("ip_host");
		ip_local_robot_right = properties.getProperty("ip_local_robot_right");
		ip_local_robot_left = properties.getProperty("ip_local_robot_left");
		debug_mode = Boolean.valueOf(properties.getProperty("debug_mode"));
		
		randomFailureEnabled = Boolean.valueOf(properties.getProperty("randomFailureEnabled"));
		randomFailure_parameter = Integer.valueOf(properties.getProperty("randomFailure_parameter"));
		
		power_motor_horizontal_initialization_1 = Integer.valueOf(properties.getProperty("power_motor_horizontal_initialization_1"));
		power_motor_horizontal_initialization_2 = Integer.valueOf(properties.getProperty("power_motor_horizontal_initialization_2"));
		power_motor_horizontal_1 = Integer.valueOf(properties.getProperty("power_motor_horizontal_1"));
		power_motor_horizontal_2 = Integer.valueOf(properties.getProperty("power_motor_horizontal_2"));
		power_motor_vertical_up = Integer.valueOf(properties.getProperty("power_motor_vertical_up"));
		power_motor_vertical_down = Integer.valueOf(properties.getProperty("power_motor_vertical_down"));
		power_motor_robotHand_open = Integer.valueOf(properties.getProperty("power_motor_robotHand_open"));
		power_motor_robotHand_close = Integer.valueOf(properties.getProperty("power_motor_robotHand_close"));
		
		delay_robotHand_close = Integer.valueOf(properties.getProperty("delay_robotHand_close"));
		delay_robotHand_open = Integer.valueOf(properties.getProperty("delay_robotHand_open"));
		delay_restartCommunicationAfterFailure = Integer.valueOf(properties.getProperty("delay_restartCommunicationAfterFailure"));
		delay_initialize = Integer.valueOf(properties.getProperty("delay_initialize"));
		delay_sending = Integer.valueOf(properties.getProperty("delay_sending"));
	}
	
	/**
	 * Returns the local host address of the robot.
	 * @return address holds the local host address, e.g. "0.0.0.1".
	 */
	private String getAddress(){
		String address = "";
		try {
			address = InetAddress.getLocalHost().getHostAddress();
		} catch (UnknownHostException e) {
			e.printStackTrace();
		}
		return address;
	}
	
	/**
	 * Returns the current distance between the ultrasonic sensor sensor_distance_horizontal and the bridge border it is pointing at.
	 * @return the current distance to the bridge border.
	 */
	private float getHorizontalDistance(){
		provider_distance_horizontal.fetchSample(sample_distance_horizontal, 0);
		return sample_distance_horizontal[0];
	}
	
	/**
	 * Returns the current distance between the ultrasonic sensor sensor_distance_upDown and the floor it is pointing at.
	 * @return the current distance to the floor.
	 */
	private float getVerticalDistance(){
		provider_distance_vertical.fetchSample(sample_distance_vertical, 0);
		return sample_distance_vertical[0];
	}
	
	/**
	 * Returns the left/right coordinate of the next point of interest.
	 * Determines the coordinates by checking for red/yellow color changes: Checks the color of the initial position (either yellow or red) and searches for a color change to the other color than the initial color (if initial color is red, then it checks for blue and vice versa)).
	 * @return the horizontal distance of the determined point of interest.
	 */
	private float returnNextPointOfInterestCoord(){
		float firstColor = readInitializationColorSensor();
		while(true){
			moveToInitializingPosition();
			int currentColor = readInitializationColorSensor();
			if(firstColor != currentColor && (currentColor == color_initialization_1 || currentColor == color_initialization_2)){
				motor_horizontal.stop();
				currentStatus = STATUS_IDLE;
				break;
			}
			else if(position == "LEFT"){
				motor_horizontal.forward();
			}
			else if(position == "RIGHT"){
				motor_horizontal.backward();
			}
			currentStatus = STATUS_MOVING_LEFT;
		}
		return getHorizontalDistance();
	}
	
	/**
	 * Initializes the horizontal place coordinates by scanning the environment for places.
	 */
	private void initializehorizontalPlaceCoordinates(){
		moveToInitializingPosition();
		release();
		
		coords_places_horizontal.put(places_horizontal.get(0), getHorizontalDistance());
		LCD.drawString(places_horizontal.get(0) + coords_places_horizontal.get(places_horizontal.get(0)), 0, 0);
		
		if(id == 1){
			motor_horizontal.setPower(power_motor_horizontal_initialization_1);
		}
		else if(id == 2){
			motor_horizontal.setPower(power_motor_horizontal_initialization_2);
		}
		
		for(int i=1; i<places_horizontal.size()-1; i++){
			coords_places_horizontal.put(places_horizontal.get(i), returnNextPointOfInterestCoord());
			LCD.clearDisplay();
			LCD.drawString(coords_places_horizontal.get(places_horizontal.get(i)) + places_horizontal.get(i), 0, 0);
		}
		if(id == 1){
			motor_horizontal.setPower(power_motor_horizontal_1);
		}
		else if(id == 2){
			motor_horizontal.setPower(power_motor_horizontal_2);
		}
		moveToDrivingPosition();
		moveToOOOPlace();
	}
	
	/**
	 * Moves to the given coordinate.
	 * @param coord a float which describes the distance between the point of interest and the bridge border that the robot is pointing at.
	 */
	private void moveToCoordinate(float coord){
		float currentCoord;
		if(position == "LEFT"){
			while(true){
				currentCoord = getHorizontalDistance();
				if(Math.round(currentCoord*100 - coord*100) > 0){
					motor_horizontal.backward();
					currentStatus = STATUS_MOVING_RIGHT;
				}
				else if(Math.round(currentCoord*100 - coord*100) < 0){
					motor_horizontal.forward();
					currentStatus = STATUS_MOVING_LEFT;
				}
				else{
					motor_horizontal.stop();
					currentStatus = STATUS_IDLE;
					break;
				}
			}
		}
		else if(position == "RIGHT"){
			while(true){
				currentCoord = getHorizontalDistance();
				if(Math.round(currentCoord*100 - coord*100) > 0){
					motor_horizontal.forward();
					currentStatus = STATUS_MOVING_RIGHT;
				}
				else if(Math.round(currentCoord*100 - coord*100) < 0){
					motor_horizontal.backward();
					currentStatus = STATUS_MOVING_LEFT;
				}
				else{
					motor_horizontal.stop();
					currentStatus = STATUS_IDLE;
					break;
				}
			}
		}
	}
	
	/**
	 * Moves to the given height.
	 * @param height a float which describes the distance between the point of interest and the bridge border that the robot is pointing at.
	 */
	private void moveUpDown(float height){
		float currentCoord;
		while(true){
			currentCoord = getVerticalDistance();
			if(Math.round(currentCoord*100 - height*100) > 0){
				motor_vertical.setPower(power_motor_vertical_down);
				motor_vertical.backward();
				currentStatus = STATUS_MOVING_DOWN;
			}
			else if(Math.round(currentCoord*100 - height*100) < 0){
				motor_vertical.setPower(power_motor_vertical_up);
				motor_vertical.forward();
				currentStatus = STATUS_MOVING_UP;
			}
			else{
				motor_vertical.stop();
				currentStatus = STATUS_IDLE;
				break;
			}
		}
	}
	
	/**
	 * Moves to the height which it needs to correctly initialize itself.
	 */
	private void moveToInitializingPosition(){
		if(id == 1){
			moveUpDown(height_initializingPosition_1);
		}
		else{
			moveUpDown(height_initializingPosition_2);
		}
	}
	
	/**
	 * Simulates a random failure.
	 * The failed robot will stop the communication, move to the out of order place and restarts the communication after waiting for some time.
	 * A separate thread is used that waits for sender and receiver threads to stop.
	 */
	private void simulateFailure(){
		currentStatus = STATUS_FAILED;
		FailThread fail = new FailThread(this, sender, receiver, delay_restartCommunicationAfterFailure);
		fail.setPriority(Thread.MAX_PRIORITY);
		fail.start();
		playSound("FAIL");
	}
	
	/**
	 * Reads, stores and returns the current color id of the brick color sensor.
	 * @return current read color id of brick color sensor.
	 */
	private int readBrickColorSensor(){
		provider_color_brick.fetchSample(sample_color_brick, 0);
		return (int)sample_color_brick[0];
	}
	
	/**
	 * Reads, stores and returns the current color id of the initialization color sensor.
	 * @return current read color id of initialization color sensor.
	 */
	private int readInitializationColorSensor(){
		provider_color_initialize.fetchSample(sample_color_initialize, 0);
		return (int)sample_color_initialize[0];
	}
	
	/**
	 * Plays a sound based on the given event.
	 * @param event String that determines the event.
	 */
	private void playSound(String event){
		switch(event){
		case "FAIL": 
			Sound.playTone(100, 800);
			Delay.msDelay(200);
			Sound.playTone(100, 800);
			Delay.msDelay(200);
			Sound.playTone(100, 800);
			break;
		case "MOVE_TO_DELIVERY_PLACE":
			Sound.playTone(200, 100);
			Sound.playTone(300, 100);
			Sound.playTone(400, 100);
			Sound.playTone(500, 100);
			Sound.playTone(600, 100);
			break;
		case "MOVE_TO_DISCHARGE_CHUTE":
			Sound.playTone(1000, 150);
			Sound.playTone(100, 750);
			break;
		case "MOVE_TO_BUILDING_SITE":
			Sound.playTone(700, 100);
			Sound.playTone(800, 100);
			Sound.playTone(900, 100);
			Sound.playTone(1000, 100);
			Sound.playTone(1100, 100);
			break;
		case "MOVE_TO_SOURCE":
			Sound.playTone(1000, 300);
			break;
		case "MOVE_TO_STORAGE_LOCATION":
			Sound.playTone(100, 150);
			Sound.playTone(1000, 750);
			break;
		case "COMMUNICATION_RESTART":
			Sound.playTone(200, 100);
			Sound.playTone(300, 100);
			Sound.playTone(400, 100);
			Sound.playTone(500, 100);
			Sound.playTone(600, 100);
			Sound.playTone(700, 100);
			Sound.playTone(800, 100);
			Sound.playTone(900, 100);
			Sound.playTone(1000, 100);
			Sound.playTone(1100, 100);
			Sound.playTone(1200, 100);
			Sound.playTone(1300, 100);
			Sound.playTone(1400, 100);
			Sound.playTone(1500, 100);
		}		
	}
	
	// ---------------------- MAIN FUNCTIONS ----------------------
	
	// ----- Initialization Methods -----
	
	/**
	 * Starts robot initialization and sets up and starts a sender and receiver thread.
	 */
	public void startWorking()
	{
		initializePropertiesBasedVariables();
		
		if(getAddress().equals(ip_local_robot_right)){
			initializeSortRobot();
		}
		else if (getAddress().equals(ip_local_robot_left)){
			initializeBuildRobot();
		}
		else{
			throw new IllegalStateException("Local robot ip does not match an user defined robot ip. Given local ip: " + getAddress());
		}
		
		semaphore = new Semaphore(1);
		
		sender = new SenderThread(this, semaphore, delay_initialize, delay_sending);
		receiver = new ReceiverThread(this, semaphore);
		
		sender.start();
		receiver.start();
	}
	
	/**
	 * Assigns a unique id and position and scans the environment to initialize the coordinates for the sorting robot.
	 */
	public void initializeSortRobot(){
		currentStatus = STATUS_INITIALIZING;
		id = 1;
		position = "RIGHT";
		initializehorizontalPlaceCoordinates();
		currentStatus = STATUS_IDLE;
	}
	
	/**
	 * Assigns a unique id and position and scans the environment to initialize the coordinates for the building robot.
	 */
	public void initializeBuildRobot(){
		currentStatus = STATUS_INITIALIZING;
		id = 2;
		position = "LEFT";
		Collections.reverse(places_horizontal);
		initializehorizontalPlaceCoordinates();
		currentStatus = STATUS_IDLE;
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
	
	// ----- Moving Methods: Horizontal -----
	
	/**
	 * Moves to the delivery place.
	 */
	public void moveToDeliveryPlace(){
		playSound("MOVE_TO_DELIVERY_PLACE");
		moveToCoordinate(coords_places_horizontal.get("deliveryPlace"));
	}
	
	/**
	 * Moves to the discharge chute.
	 */
	public void moveToDischargeChute(){
		playSound("MOVE_TO_DISCHARGE_CHUTE");
		moveToCoordinate(coords_places_horizontal.get("dischargeChute"));
	}
	
	/**
	 * Moves to the indicated row of the building site.
	 * @param number of row to which the robot should move.
	 */
	public void moveToBuildingSite(int row){
		playSound("MOVE_TO_BUILDING_SITE");
		if(id == 1){
			moveToCoordinate(coords_places_horizontal.get("buildingSite") - (float)(row * buildingSite_width));
		}
		else if(id == 2){
			moveToCoordinate(coords_places_horizontal.get("buildingSite") + (float)(row * buildingSite_width));
		}
	}
	
	/**
	 * Moves to the source.
	 */
	public void moveToSource(){
		playSound("MOVE_TO_SOURCE");
		moveToCoordinate(coords_places_horizontal.get("source"));
	}
	
	/**
	 * Moves to its out of order place (outOfOrderPlace_1 if id=1, outOfOrderPlace_2 if id=2)
	 */
	public void moveToOOOPlace(){
		moveToDrivingPosition();
		release();
		moveToCoordinate(coords_places_horizontal.get("outOfOrderPlace_" + id));
	}
	
	/**
	 * Moves to the storage location with the indicated number
	 * @param storageNumb indicates the number of the storage location
	 */
	public void moveToStorageLocation(int storageNumb){
		playSound("MOVE_TO_STORAGE_LOCATION");
		moveToCoordinate(coords_places_horizontal.get("storageLocation_" + (storageNumb + 1)));
	}
	
	// ----- Moving Methods: Vertical -----
	
	/**
	 * Moves to the driving position height
	 */
	public void moveToDrivingPosition(){
		moveUpDown(height_drivingPosition);
	}
	
	/**
	 * Moves to the gripping position height 
	 */
	public void moveToGrippingPosition(){
		moveUpDown(height_grippingPosition);
	}
	
	/**
	 * Moves to the building position height.
	 * @param height row indicator for the height.
	 */
	public void moveToBuildingPosition(int row){
		moveUpDown((float)(height_grippingPosition + 0.001 + buildingPosition_height * ((float)row)));
	}
	
	// ----- Robot Hand Methods -----
	
	/**
	 * Closes the robot hand to grab a brick.
	 */
	public void grab(){
		currentStatus = STATUS_GRABBING;
		motor_robotHand.setPower(power_motor_robotHand_close);
		motor_robotHand.forward();
		Delay.msDelay(delay_robotHand_close);
		motor_robotHand.stop();
		setCurrentBrickColor();
		currentStatus = STATUS_IDLE;
	}
	
	/**
	 * Opens the robot hand to release a brick.
	 */
	public void release(){
		currentStatus = STATUS_RELEASING;
		motor_robotHand.setPower(power_motor_robotHand_open);
		motor_robotHand.backward();
		Delay.msDelay(delay_robotHand_open);
		motor_robotHand.stop();
		currentStatus = STATUS_IDLE;
	}

	/**
	 * Scans the current brick which is located below the robot hand and sets the color as currentBrickColor.
	 */
	public void setCurrentBrickColor(){
		int currentColor = readBrickColorSensor();
		if(0 <= currentColor && currentColor <= 13){
			currentBrickColor = colors.get(currentColor);
		}
		else{
			currentBrickColor = null;
		}
	}
	
	// ----- JSON Methods -----
	
	/**
	 * Creates and returns a JSON string containing information about the robot.
	 * @return json JSON String containing information about POSITION, COLOR and STATUS of the robot.
	 */
	public String getJsonString(){
		String json = "{\"Robot_" + getID() + "\":{\"POSITION\":\"" + getPosition() + "\",\"COLOR\":\"" + getCurrentBrickColor() + "\",\"STATUS\":\"" + getCurrentStatus() + "\",\"IP\":\"" + sender.getSocket().getLocalAddress().getHostAddress() + "\"}}";		
		return json;
	}
	
	/**
	 * Interprets a given JSON String as an action. If randomFailureEnabled is set true, then the robot will fail with a probability based on randomFailure_parameter (1 out of randomFailure_parameter times) after interpreting the given JSON String.
	 * @param json JSON String containing an action and optionally a value.
	 */
	public void interpretJsonString(String json){
		String action = "";
		for(int i=0; i<actions.length; i++){
			if(json.contains(actions[i])){
				action = actions[i];
			}
		}
		
		int number = -1;
		if(action.equals( "MOVE_TO_BUILDING_SITE") || action.equals("MOVE_TO_STORAGE_LOCATION") || action.equals("MOVE_TO_BUILDING_POSITION")){
			number = Integer.valueOf(json.substring(11).replaceAll("[^0-9]+", ""));
		}
		
		switch(action){
		   case "GRAB": grab(); break;
		   case "RELEASE": release(); break;
//		   case "PRESS_TIGHT": pressTight(); break;
		   case "MOVE_TO_DELIVERY_PLACE": moveToDeliveryPlace(); break;
		   case "MOVE_TO_DISCHARGE_CHUTE": moveToDischargeChute(); break;
		   case "MOVE_TO_DRIVING_POSITION": moveToDrivingPosition(); break;
		   case "MOVE_TO_GRIPPING_POSITION": moveToGrippingPosition(); break;
		   case "MOVE_TO_BUILDING_SITE": moveToBuildingSite(number); break;			
		   case "MOVE_TO_STORAGE_LOCATION": moveToStorageLocation(number); break;		
		   case "MOVE_TO_BUILDING_POSITION": moveToBuildingPosition(number); break;	
		   case "MOVE_TO_SOURCE": moveToSource(); break;
		   case "MOVE_TO_OOO_PLACE": moveToOOOPlace(); break;
		   default: throw new IllegalStateException("Action not recognized. Given action: " + action);
		}
		
		if(randomFailureEnabled & (randomGenerator.nextInt() % randomFailure_parameter == 4)){
			simulateFailure();
		}
	}

	/**
	 * Display a message on the robot screen.
	 * @param message message to display.
	 */
	public void print(String message)
	{
		LCD.clear();
		LCD.drawString(message, 1, 1);
		if(debug_mode)
			Delay.msDelay(3000);
		LCD.clear();
	}
	
	/**
	 * Restarts the communication by setting up and starting new sender and receiver threads.
	 */
	public void restartCommunication()
	{
		print("Restarting Communication");
		semaphore = new Semaphore(1);
		
		sender = new SenderThread(this, semaphore, 0, delay_sending);
		receiver = new ReceiverThread(this, semaphore);
		
		sender.start();
		receiver.start();
		print("Threads restarted");
		currentStatus = STATUS_IDLE;
		playSound("COMMUNICATION_RESTART");
	}
	
	// Further methods which were not included:
	
/*
	// Closes the hand and moves the hand down to the ground to press a brick against the ground.
	public void pressTight(){
		currentStatus = STATUS_GRABBING;
		motor_robotHand.setPower(50);
		motor_robotHand.forward();
		Delay.msDelay(2500);
		motor_robotHand.stop();
		motor_vertical.setPower(100);
		moveUpDown((float)0.039);
		Delay.msDelay(2000);
		currentStatus = STATUS_IDLE;
	}
	
	// Reads the brick color sensor multiple times and performs a majority vote to determine the correct value.
	private int getMajorityVotedBrickColor(){
	for(int i=0; i<=1; i++){
		provider_color_brick.fetchSample(sample_color_brick, i);
	}
	HashMap<Integer, Integer> candidates = new HashMap<Integer, Integer>();
	for(int i=0; i<sample_color_brick.length; i++){
		int currentCandidate = (int)sample_color_brick[i];
		if(candidates.containsKey(currentCandidate)){
			candidates.put(currentCandidate, candidates.get(currentCandidate)+1);
		}
		else{
			candidates.put(currentCandidate, 1);
		}
	}
	
	int maxValue = -1;
	int colorWithMaxValue = -1;
	Iterator<Entry<Integer, Integer>> it = candidates.entrySet().iterator();
	while (it.hasNext()) {
	    Entry<Integer, Integer> entry = it.next();
	    if(entry.getValue() > maxValue){
	    	maxValue = entry.getValue();
	    	colorWithMaxValue = entry.getKey();
	    }
	}		
	return colorWithMaxValue
}
	
	// Reads the initialization color sensor multiple times and performs a majority vote to determine the correct value.
	private int getMajorityVotedInitializationColor(){
	for(int i=0; i<=1; i++){
		provider_color_initialize.fetchSample(sample_color_initialize, i);
	}
	HashMap<Integer, Integer> candidates = new HashMap<Integer, Integer>();
	for(int i=0; i<sample_color_initialize.length; i++){
		int currentCandidate = (int)sample_color_initialize[i];
		if(candidates.containsKey(currentCandidate)){
			candidates.put(currentCandidate, candidates.get(currentCandidate)+1);
		}
		else{
			candidates.put(currentCandidate, 1);
		}
	}
	
	int maxValue = -1;
	int colorWithMaxValue = -1;
	Iterator<Entry<Integer, Integer>> it = candidates.entrySet().iterator();
	while (it.hasNext()) {
	    Entry<Integer, Integer> entry = it.next();
	    if(entry.getValue() > maxValue){
	    	maxValue = entry.getValue();
	    	colorWithMaxValue = entry.getKey();
	    }
	}
	return colorWithMaxValue;
}
*/
}
