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
	private final String[] colors = {
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
		"BROWN"
	};
	
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
	
	// User defined properties:
	private Properties properties_userDefined = new Properties();
		
	// Vertical Coordinates:
	private float height_initializingPosition;
	private float height_drivingPosition;
	private float height_grippingPosition;
	private float buildingPosition_height;
	private float buildingSite_width;
	
	// Motors:
	private final UnregulatedMotor motor_robotHand = new UnregulatedMotor(MotorPort.A);
	private final UnregulatedMotor motor_vertical = new UnregulatedMotor(MotorPort.B);	
	private final UnregulatedMotor motor_horizontal = new UnregulatedMotor(MotorPort.C);
	
	// Power:
	private int power_motor_horizontal;
	private int power_motor_horizontal_initialization;
	private int power_motor_vertical_up;
	private int power_motor_vertical_down;
	private int power_motor_robotHand_open;
	private int power_motor_robotHand_close;
	
	// Delays:
	private int delay_initialize;
	private int delay_robotHand_open;
	private int delay_robotHand_close;
	private int delay_restartCommunicationAfterFailure;
		
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
	
	// Local IPs
	private String ip_local_robot_right;
	private String ip_local_robot_left;
	
	// Communication
	public static String ip_host;
	private boolean debug_mode;
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
	
	/**
	 * Starts robot initialization and sets up and starts a sender and receiver thread.
	 */
	public void startWorking()
	{
		initializeUserDefinedVariables();
		
		if(getAddress().equals(ip_local_robot_right)){
			initializeSortRobot();
		}
		else if (getAddress().equals(ip_local_robot_left)){
			initializeBuildRobot();
		}
		else{
			throw new IllegalStateException("Local robot ip does not match an user defined robot ip. Given local ip: " + getAddress());
		}
		
		Semaphore sema = new Semaphore(1);
		
		sender = new SenderThread(this, sema, delay_initialize);
		receiver = new ReceiverThread(this, sema);
		
		sender.start();
		receiver.start();
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

    		properties_userDefined.load(input);

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
	 * Returns the local host address of the robot
	 * @return address holds the local host address, e.g. "0.0.0.1"
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
	 * Returns the current distance between the ultrasonic sensor sensor_distance_leftRight and the bridge border it is pointing at.
	 * @return the current distance to the bridge border.
	 */
	private float getLeftRightDistance(){
		provider_distance_leftRight.fetchSample(sample_distance_leftRight, 0);
		return sample_distance_leftRight[0];
	}
	
	/**
	 * Returns the current distance between the ultrasonic sensor sensor_distance_upDown and the floor it is pointing at.
	 * @return the current distance to the floor.
	 */
	private float getUpDownDistance(){
		provider_distance_upDown.fetchSample(sample_distance_upDown, 0);
		return sample_distance_upDown[0];
	}
	
	/**
	 * Returns the left/right coordinate of the next point of interest.
	 * Determines the coordinates by checking for red/blue color changes: Checks the color of the initial position (either blue or red) and searches for a color change to the other color than the initial color (if initial color is red, then it checks for blue and vice versa)).
	 * @return the left/right distance of the determined point of interest.
	 */
	private float returnNextPointOfInterestCoord(){
		provider_color_initialize.fetchSample(sample_color_initialize, 0);
		float firstColor = sample_color_initialize[0];
		while(true){
			provider_color_initialize.fetchSample(sample_color_initialize, 0);
			if(firstColor != sample_color_initialize[0] && (sample_color_initialize[0] == 0 || sample_color_initialize[0] == 2)){
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
		return getLeftRightDistance();
	}
	
	/**
	 * Moves to the given coordinate.
	 * @param coord a float which describes the distance between the point of interest and the bridge border that the robot is pointing at.
	 */
	private void moveToCoordinate(float coord){
		float currentCoord;
		if(position == "LEFT"){
			while(true){
				currentCoord = getLeftRightDistance();
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
				currentCoord = getLeftRightDistance();
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
			currentCoord = getUpDownDistance();
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
		moveUpDown(height_initializingPosition);
	}
	
	private void initializeUserDefinedVariables(){
		readProperties();

		places_horizontal = Arrays.asList(properties_userDefined.getProperty("places_horizontal").split(","));
		
		height_initializingPosition = Float.valueOf(properties_userDefined.getProperty("height_initializingPosition"));
		height_drivingPosition = Float.valueOf(properties_userDefined.getProperty("height_drivingPosition"));
		height_grippingPosition = Float.valueOf(properties_userDefined.getProperty("height_grippingPosition"));
		buildingPosition_height = Float.valueOf(properties_userDefined.getProperty("buildingPosition_height"));
		buildingSite_width = Float.valueOf(properties_userDefined.getProperty("buildingSite_width"));
		
		ip_host = properties_userDefined.getProperty("ip_host");
		ip_local_robot_right = properties_userDefined.getProperty("ip_local_robot_right");
		ip_local_robot_left = properties_userDefined.getProperty("ip_local_robot_left");
		debug_mode = Boolean.valueOf(properties_userDefined.getProperty("debug_mode"));
		randomFailureEnabled = Boolean.valueOf(properties_userDefined.getProperty("randomFailureEnabled"));
		
		power_motor_horizontal_initialization = Integer.valueOf(properties_userDefined.getProperty("power_motor_horizontal_initialization"));
		power_motor_horizontal = Integer.valueOf(properties_userDefined.getProperty("power_motor_horizontal"));
		power_motor_vertical_up = Integer.valueOf(properties_userDefined.getProperty("power_motor_vertical_up"));
		power_motor_vertical_down = Integer.valueOf(properties_userDefined.getProperty("power_motor_vertical_down"));
		power_motor_robotHand_open = Integer.valueOf(properties_userDefined.getProperty("power_motor_robotHand_open"));
		power_motor_robotHand_close = Integer.valueOf(properties_userDefined.getProperty("power_motor_robotHand_close"));
		
		delay_robotHand_close = Integer.valueOf(properties_userDefined.getProperty("delay_robotHand_close"));
		delay_robotHand_open = Integer.valueOf(properties_userDefined.getProperty("delay_robotHand_open"));
		delay_restartCommunicationAfterFailure = Integer.valueOf(properties_userDefined.getProperty("delay_restartCommunicationAfterFailure"));
		delay_initialize = Integer.valueOf(properties_userDefined.getProperty("delay_initialize"));
	}
	
	private void initializehorizontalPlaceCoordinates(){
		moveToInitializingPosition();
		release();
		motor_horizontal.setPower(power_motor_horizontal_initialization);
		
		coords_places_horizontal.put(places_horizontal.get(0), getLeftRightDistance());
		LCD.drawString(places_horizontal.get(0) + coords_places_horizontal.get(places_horizontal.get(0)), 0, 0);
		for(int i=1; i<places_horizontal.size()-1; i++){
			coords_places_horizontal.put(places_horizontal.get(i), returnNextPointOfInterestCoord());
			LCD.clearDisplay();
			LCD.drawString(coords_places_horizontal.get(places_horizontal.get(i)) + places_horizontal.get(i), 0, 0);
		}
		motor_horizontal.setPower(power_motor_horizontal);
		moveToDrivingPosition();
		moveToOOOPlace();
	}
	
	// ---------------------- MAIN FUNCTIONS ----------------------
	
	// ----- Initialization Methods -----
	
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
	
	// ----- Moving Methods: Left/Right -----
	
	/**
	 * Moves to the delivery place.
	 */
	public void moveToDeliveryPlace(){
		moveToCoordinate(coords_places_horizontal.get("deliveryPlace"));
	}
	
	/**
	 * Moves to the discharge chute.
	 */
	public void moveToDischargeChute(){
		moveToCoordinate(coords_places_horizontal.get("dischargeChute"));
	}
	
	/**
	 * Moves to the indicated row of the building site.
	 * @param number of row to which the robot should move.
	 */
	public void moveToBuildingSite(int row){
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
		moveToCoordinate(coords_places_horizontal.get("source"));
	}
	
	/**
	 * Moves to its out of order place (outOfOrderPlace_1 if id=1, outOfOrderPlace_2 if id=2)
	 */
	public void moveToOOOPlace(){
		moveToCoordinate(coords_places_horizontal.get("outOfOrderPlace_" + id));
	}
	
	/**
	 * Moves to the storage location with the indicated number
	 * @param storageNumb indicates the number of the storage location
	 */
	public void moveToStorageLocation(int storageNumb){
		moveToCoordinate(coords_places_horizontal.get("storageLocation_" + (storageNumb + 1)));
	}
	
	// ----- Moving Methods: Up/Down -----
	
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
	public void moveToBuildingPosition(float row){
		moveUpDown((float)(height_grippingPosition + 0.001 + buildingPosition_height * row));
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
	 * Closes the hand and moves the hand down to the ground to press a brick against the ground.
	 */
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

	/**
	 * Scans the current brick which is located below the robot hand and sets the color as currentBrickColor.
	 */
	public void setCurrentBrickColor(){
		provider_color_brick.fetchSample(sample_color_brick, 0);
		if(0 <= (int)sample_color_brick[0] && (int)sample_color_brick[0] <= 13){
			currentBrickColor = colors[(int)sample_color_brick[0]];
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
	 * Interprets a given JSON String as an action. If randomFailureEnabled is set true, then the robot will fail with a probability of 10% and won't interpret the given JSON String.
	 * @param json JSON String containing an action and optionally a value.
	 */
	public void interpretJsonString(String json){
		if(randomFailureEnabled & (randomGenerator.nextInt() % 10 == 4)){
			currentStatus = STATUS_FAILED;
			stopCommunication();
			moveToOOOPlace();
			Delay.msDelay(delay_restartCommunicationAfterFailure);
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
			if(action.equals( "MOVE_TO_BUILDING_SITE") || action.equals("MOVE_TO_STORAGE_LOCATION") || action.equals("MOVE_TO_BUILDING_POSITION")){
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
			   case "MOVE_TO_BUILDING_POSITION": moveToBuildingPosition(number); break;	
			   case "MOVE_TO_SOURCE": moveToSource(); break;
			   case "MOVE_TO_OOO_PLACE": moveToOOOPlace(); break;
			   default: throw new IllegalStateException("Action not recognized. Given action: " + action);
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
		if(debug_mode)
			Delay.msDelay(3000);
		LCD.clear();
	}
	
	/**
	 * Stops the communication by terminating the sender and receiver threads.
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
	 * Restarts the communication by setting up and starting a sender and receiver thread.
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
		LCD.drawString(" " + coords_places_horizontal.get("deliveryPlace"), 0, 0);
	}
	
	private void write2(){
		LCD.clearDisplay();
		LCD.drawString(" " + coords_places_horizontal.get("dischargeChute"), 0, 0);
	}
}
