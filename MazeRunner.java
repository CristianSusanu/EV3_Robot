import java.util.ArrayList;
import java.util.Arrays;
import java.util.Stack;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.filter.MeanFilter;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class MazeRunner
{
	private static Node[][] mazeMap = new Node[21][21];
	private static String currentOrientation = "N";
	private static int currentX = 2;
	private static int currentY = 2;
	private static int offSetA = 0;
	private static int offSetB = 0;
	private static int offSetC = 0;
	private static int offSetD = 0;
	private static Stack<Integer> turns = new Stack<Integer>();
	private static ArrayList<String> orientations = new ArrayList<String>(Arrays.asList("N", "E", "S", "W"));
	private static ArrayList<Node> exploredNodes = new ArrayList<Node>();
	private static EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S1);
	private static SampleProvider gyroAngle = gyro.getAngleAndRateMode();
	private static float[] sampleGyro = new float[gyroAngle.sampleSize()];
	private static String[][] myArr = new String[20][20];
	private static ServerSocket server = null;
	private static Socket client = null;
	private static OutputStream out = null;
	private static DataOutputStream dOut = null;
	public static final int port = 2554;
	
	/**
	 * Mapping on a connected PC console via bluetooth
	 * @param finished
	 * 				<code>true</code> if the method is being called for the last time
	 * @param firstCall
	 * 				<code>true</code> if the method if being called for the first time
	 * @throws IOException If anything goes wrong
	 */
	public static void 				getBluetooth(Boolean finished, Boolean firstCall) 	throws IOException {
		if (firstCall) {
			server = new ServerSocket(port);
			Lcd.print(6, "Awaiting client...");
			client = server.accept();
			Lcd.clear();
			Lcd.print(6, "CONNECTED");
			Delay.msDelay(500);
			Lcd.clear();
			out = client.getOutputStream();
			dOut = new DataOutputStream(out);
		}
		
		dOut.writeBoolean(true);
		for (int j = 1, k = 0; j < 20; j++, k++) {
			for (int i = 1, n = 18; i < 20; i++, n--) {
				if (j == currentX && i == currentY) {
					myArr[n][k] = "R";
				} else if (mazeMap[j][i].getType() == "No-Go") {
					myArr[n][k] = "@";
				} else if (mazeMap[j][i].getType() == "Finish") {
					myArr[n][k] = "x";
				} else if (mazeMap[j][i].getType() == "Empty" || mazeMap[j][i].getType() == "Gap") {
					myArr[n][k] = " ";
				} else myArr[n][k] = "*";
			}
		}
		
		if (myArr[17][1] != "R") {
			myArr[17][1] = "O";
		}
		
		for (int y = 0; y < 19; y++) {
			for (int x = 0; x < 19; x++) {
				dOut.writeChars((myArr[y][x]));
			}
		}
		
		if (finished) {
			dOut.writeBoolean(false);
			server.close();
			client.close();
			dOut.flush();
		}
		
	}
	
	/**
	 * A method to get the colours from the Light sensor with calibration
	 * @param sensor
	 * 				Light sensor object
	 * @return The name of the colour
	 */
	public static String 			getColour(ColorSensor sensor) 						{
    	Color rgb = sensor.getColor();
    	int r = rgb.getRed();
    	int g = rgb.getGreen();
    	//int b = rgb.getBlue();
    	//Lcd.print(6, "R: " + r + " G: " + g + " B: " + b);
    	if (g > r && g < 40 && r < 40) {
    		return "Green";
    	} else if (r > 30 && g < 30) {
    		return "Red";
    	}
    	return "N/A";
    }
	
	/**
	 * Creates a default map with all but the Start (and the one behind it) nodes having their Types
	 * set to <code>null</code> and Exploration set to <code>false</code>
	 */
	private static void 			mapCreation() 										{
		for (int i = 0; i < 21; i++)
			for (int j = 0; j < 21; j++) {
				mazeMap[i][j] = new Node(i,j);
				if (i % 2 != 0 && j % 2 != 0)
					mazeMap[i][j].setType("Wall");
			}
				
				
		for (int i = 2; i < 19; i++)
			for (int j = 2; j < 19; j++) {
					mazeMap[i][j].addChild(mazeMap[i][j+1]);
					mazeMap[i][j].addChild(mazeMap[i+1][j]);
					mazeMap[i][j].addChild(mazeMap[i][j-1]);
					mazeMap[i][j].addChild(mazeMap[i-1][j]);
			}
		
		for (int i = 0; i < 21; i++) {
			mazeMap[i][0].setType("Wall");
			mazeMap[i][20].setType("Wall");
			mazeMap[i][1].setType("Wall");
			mazeMap[i][19].setType("Wall");
		}
		for (int j = 1; j < 21; j++) {
			mazeMap[0][j].setType("Wall");
			mazeMap[20][j].setType("Wall");
			mazeMap[1][j].setType("Wall");
			mazeMap[19][j].setType("Wall");
		}
		
		mazeMap[2][2].setType("Empty");
		mazeMap[2][2].setPrev(mazeMap[2][0]);
	}
	
	/**
	 * Sets the offsets for mapping neighbouring nodes according to the current
	 * orientation of the EV3 robot
	 */
	private static void 			setOffSets() 										{
		if (currentOrientation == "N") {
	        offSetA = 0;
	        offSetB = 1;
	        offSetC = 1;
	        offSetD = 0;
		} else if (currentOrientation == "E") {
			offSetA = 1;
	        offSetB = 0;
	        offSetC = 0;
	        offSetD = -1;
		} else if (currentOrientation == "S") {
			offSetA = 0;
	        offSetB = -1;
	        offSetC = -1;
	        offSetD = 0;
		} else if (currentOrientation == "W") {
			offSetA = -1;
	        offSetB = 0;
	        offSetC = 0;
	        offSetD = 1;
		}
	}
	
	/**
	 *  Calculates the rotation needed to face the next node to be mapped
	 * @param nextNode
	 * 				The node to which the EV3 robot wants to travel
	 * @return An <code>int</code> value of degrees needed to turn to face the <code>nextNode</code>
	 */
	private static int 				whichDirection(Node nextNode) 						{
		setOffSets();
		Lcd.clear(2);
		Lcd.print(2, "Going to: " + nextNode.toString());
		
		if (nextNode == mazeMap[currentX + 2 * offSetA][currentY + 2 * offSetB]) {
			turns.push(0);
			return(0);
		} else if (nextNode == mazeMap[currentX + 2 * offSetC][currentY + 2 * offSetD]) {
			currentOrientation = orientations.get((orientations.indexOf(currentOrientation) + 1) % 4);
			turns.push(91);
			return(91);
		} else if (nextNode == mazeMap[currentX - 2 * offSetC][currentY - 2 * offSetD]) {
			currentOrientation = orientations.get((((orientations.indexOf(currentOrientation) - 1) % 4) + 4) % 4);
			turns.push(-90);
			return(-90);
		}
		else {
			currentOrientation = orientations.get((orientations.indexOf(currentOrientation) + 2) % 4);
			turns.push(180);
			return(180);
		}
	}
	
	/**
	 * Moves the EV3 robot to the node specified by the <code>nextNode</code> parameter or 
	 * if there is no such node (<code>nextNode</code> is <code>null</code>)
	 * moves to the previous node
	 * @param pilot
	 * 				Chassi pilot in use
	 * @param nextNode
	 * 				The node to which the EV3 robot is going to try and travel
	 */
	private static void 			moveToNode(MovePilot pilot, Node nextNode) 			{
		
		if (nextNode != null) {
			
			if (nextNode.getType() == "Gap" || nextNode == mazeMap[currentX][currentY]) 
				return;
			
			turnTo(pilot, whichDirection(nextNode));
			Lcd.clear(2); Lcd.clear(8);
			Lcd.print(8, "Facing: " + currentOrientation);
			Lcd.print(2, "Going to: " + nextNode.toString());
			pilot.travel(38.3);
			mazeMap[nextNode.getX()][nextNode.getY()].setPrev(mazeMap[currentX][currentY]);
			currentX = nextNode.getX();
			currentY = nextNode.getY();
		} else {
			nextNode = mazeMap[currentX][currentY].getPrev();
			Lcd.clear(2);
			Lcd.print(2, "Going to: " + nextNode.toString());
			pilot.travel(-38);
			turnTo(pilot, -turns.pop());
			currentX = nextNode.getX();
			currentY = nextNode.getY();
			currentOrientation = mazeMap[currentX][currentY].getDiscovery();
		}
	}
	
	/**
	 * A bubble-sort method sorting the <code>nodes</code> in an <code>ArrayList</code>
	 * by their cost
	 * @param consideredNodes
	 * 				The list of nodes to be sorted
	 * @return Sorted <code>ArrayList</code>
	 */
	public static ArrayList<Node> 	bubble(ArrayList<Node> listToSort) 					{
		
		for (int i = 0; i < listToSort.size() - 1; i++) 
			for (int j = i + 1; j < listToSort.size(); j++) 
				if (listToSort.get(i).totalCost > listToSort.get(j).totalCost) {
					Node temp = listToSort.get(i);
					listToSort.set(i, listToSort.get(j));
					listToSort.set(j, temp);
				}
		return listToSort;
	}

	/**
	 * A* algorithm used to find the shortest path from the <code>End</code> node
	 * to the <code>Start</code> node
	 * @param End
	 * 				The end node
	 * @param Start
	 * 				The start node
	 * @return An <code>ArrayList</code> of <code>Nodes</code> - the path to take from the <code>End</code> node
	 * 		   to the <code>Start</code> node
	 * @throws IOException 
	 */
	public static ArrayList<Node> 	aStar(Node End, Node Start) 						{
		
		ArrayList<Node> openList = new ArrayList<Node>();
		ArrayList<Node> closedList = new ArrayList<Node>();
		ArrayList<Node> pathEndToStart = new ArrayList<Node>();
		Node q;
		Boolean search = true;
		Boolean skip;
		
		Start.NodeToEnd = Math.abs(Start.getX() - End.getX()) + Math.abs(Start.getY() - End.getY());
		Start.totalCost = Start.NodeToEnd;
		openList.add(Start);
		
		while (!openList.isEmpty() && search && Button.ESCAPE.isUp()) {
			openList = bubble(openList);
			q = openList.remove(0);
			//Lcd.print(3, q.toString());
			for (Node child : q.getChildren()) {
				//if (child.getType() != "Wall" && child.getType() != "No-Go") {
					skip = false;
					if (q.getParent() != null && !openList.isEmpty()) {
						if (q.getParent().totalCost > child.totalCost)
							child.setParent(q);
					} else child.setParent(q);
					
					//Step 1
					
					if (child == End) {
						search = false;
						q = child;
						break;
					}
					
					if (child.getType() == "Wall" || child.getType() == "No-Go") {
						child.StartToNode = 100;
					} else child.StartToNode = q.StartToNode + 1;
					
					child.NodeToEnd = Math.abs(child.getX() - End.getX()) + Math.abs(child.getY() - End.getY());
					child.totalCost = child.NodeToEnd + child.StartToNode;
					
					//Step 2
					
					if (!openList.isEmpty()) {
						for (Node temp : openList) {
							if (temp.getX() == child.getX() && temp.getY() == child.getY() && temp.totalCost <= child.totalCost) {
								skip = true;
								break;
							}
						}
					}
					
					//Step 3
					
					if (!closedList.isEmpty()) {
						for (Node temp : closedList) {
							if (temp.getX() == child.getX() && temp.getY() == child.getY() && temp.totalCost <= child.totalCost) {
								skip = true;
								break;
							}
						}
					}
					
					if (!skip) openList.add(child);
				//}
			}
			
			closedList.add(q);
		}
		
		q = closedList.remove(closedList.size() - 1);
		//Lcd.clear(3);
		//Lcd.print(3, q.toString());
		while (q != Start && Button.ESCAPE.isUp()) {
			//Delay.msDelay(500);
			q = q.getParent();
			pathEndToStart.add(q);
			//Lcd.clear(3);
			//Lcd.print(3, q.toString());
		}
		
		return pathEndToStart;
	}

	/**
	 * Checks to see if all the nodes have their children explored
	 * @return <code>true</code> if and only if all the explored nodes have their children
	 * 			explored as well
	 */
	private static Boolean 			mazeExplored() 										{
		for (Node cNode : exploredNodes) 
			if (!cNode.getUnvisChildren().isEmpty()) {
				return false;
			}
				
		return true;
	}
	
	/**
	 * Turns the degrees specified by the <code>angle</code> parameter
	 * @param pilot
	 * 				The main chassi pilot
	 * @param angle
	 * 				The degrees to turn
	 */
	private static void 			turnTo(MovePilot pilot, int angle) 					{
		pilot.stop();
		gyro.reset();
        gyroAngle.fetchSample(sampleGyro, 0);
        int i = 0;
        while (sampleGyro[0] != -angle && i < 5) {
        	pilot.rotate(angle + sampleGyro[0]);
        	gyroAngle.fetchSample(sampleGyro, 0);
        	if (sampleGyro[0] < -(angle + 15) || sampleGyro[0] > -(angle - 15)) break;
        	i++;
        }
	}
	
	public static void 				main(String[] args) 								throws IOException {

		/*
		 ********* Robot configuration ************
		 */
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);
		EV3MediumRegulatedMotor irMotor = new EV3MediumRegulatedMotor(MotorPort.C);
		
		Wheel wheel1 = WheeledChassis.modelWheel(LEFT_MOTOR, 5.5).offset(-6);
		Wheel wheel2 = WheeledChassis.modelWheel(RIGHT_MOTOR, 5.5).offset(6);
		
		Chassis  chassis = new WheeledChassis(new Wheel[]{wheel1, wheel2}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot= new MovePilot(chassis);
		
        ColorSensor colour = new ColorSensor(SensorPort.S3);
		
    	EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S2);
    	SampleProvider irDistance = irSensor.getDistanceMode();
    	SampleProvider average = new MeanFilter(irDistance, 5);
    	float[] sampleIR = new float[average.sampleSize()];
		
    	colour.setRGBMode();
        colour.setFloodLight(true);
        pilot.setLinearSpeed(12);
		pilot.setAngularSpeed(40);
		
		Node nextNode;
		Node endNode = null;
    	
		Button.LEDPattern(4);
        Sound.beepSequenceUp();
        /*
         ******************************************
         */
        
        mapCreation();
        Lcd.print(1, "Assignment 2");
        Lcd.print(2, "Press to start");
        Button.waitForAnyPress();
        gyro.setCurrentMode(1);
        Lcd.clear();
        getBluetooth(false, true);
        Button.LEDPattern(0);
        Delay.msDelay(1000);

		while (Button.ESCAPE.isUp()) {
			Lcd.clear(6); Lcd.clear(7); Lcd.clear(8);
			Lcd.print(6, "Previous: " + mazeMap[currentX][currentY].getPrev().toString());
			Lcd.print(7, "Current: " + mazeMap[currentX][currentY].toString());
			Lcd.print(8, "Facing: " + currentOrientation);
			
			//Red node
			
			if (getColour(colour) == "Red") {
				endNode = mazeMap[currentX][currentY];
				endNode.setType("Finish");
				Sound.beep();
			}

			//Explores the node's neighbours
			
			if (!mazeMap[currentX][currentY].getExplored()) {
				
				Node tempChild;
				Node tempChild2;
				setOffSets();
				mazeMap[currentX][currentY].setDiscovery(currentOrientation);

		    	irDistance.fetchSample(sampleIR, 0);
		    	
		    	// Front Node
		    	tempChild = mazeMap[currentX + offSetA][currentY + offSetB];
		    	if (sampleIR[0] < 30) {
		    		tempChild.setType("Wall");
		    		tempChild.setExplored(true);
		    	} else if (!tempChild.getExplored()) {
		    		tempChild2 = mazeMap[currentX + 2 * offSetA][currentY + 2 * offSetB];
		    		tempChild.setType("Gap");
		    		tempChild.setExplored(true);
		    		tempChild2.setType("Empty");
		    		mazeMap[currentX][currentY].addUnvisChild(tempChild2);
		    		tempChild2.addParent(mazeMap[currentX][currentY]);
		    	}
		    	
				// Right Node
		    	irMotor.rotateTo(90);
		    	irDistance.fetchSample(sampleIR, 0);
		    	tempChild = mazeMap[currentX + offSetC][currentY + offSetD];
		    	if (sampleIR[0] < 30) {
		    		tempChild.setType("Wall");
		    		tempChild.setExplored(true);
		    	} else if (!tempChild.getExplored()) {
		    		tempChild2 = mazeMap[currentX + 2 * offSetC][currentY + 2 * offSetD];
		    		tempChild.setType("Gap");
		    		tempChild.setExplored(true);
		    		tempChild2.setType("Empty");
		    		mazeMap[currentX][currentY].addUnvisChild(tempChild2);
		    		tempChild2.addParent(mazeMap[currentX][currentY]);
		    	}
		    	
				// Left Node
		    	irMotor.rotateTo(-90);
		    	irDistance.fetchSample(sampleIR, 0);
		    	tempChild = mazeMap[currentX - offSetC][currentY - offSetD];
		    	if (sampleIR[0] < 30) {
		    		tempChild.setType("Wall");
		    		tempChild.setExplored(true);
		    	} else if (!tempChild.getExplored()) {
		    		tempChild2 = mazeMap[currentX - 2 * offSetC][currentY - 2 * offSetD];
		    		tempChild.setType("Gap");
		    		tempChild.setExplored(true);
		    		tempChild2.setType("Empty");
		    		mazeMap[currentX][currentY].addUnvisChild(tempChild2);
		    		tempChild2.addParent(mazeMap[currentX][currentY]);
		    	}
		    	
		    	irMotor.rotateTo(0);
		    	mazeMap[currentX][currentY].setExplored(true);
		    	
		    	for (Node explored : mazeMap[currentX][currentY].getParents()) {
		    		explored.getUnvisChildren().remove(mazeMap[currentX][currentY]);
		    	}
		    	
		    	exploredNodes.add(mazeMap[currentX][currentY]);
			}
			
			//Green Node
			
			if (getColour(colour) == "Green") {
				mazeMap[currentX][currentY].setType("No-Go");
				mazeMap[currentX][currentY].setExplored(true);
				nextNode = mazeMap[currentX][currentY].getPrev();
				pilot.travel(-38);
				turnTo(pilot, -turns.pop());
				currentX = nextNode.getX();
				currentY = nextNode.getY();
				currentOrientation = mazeMap[currentX][currentY].getDiscovery();
				Lcd.clear(8);
				Lcd.print(8, "Facing: " + currentOrientation);
				getBluetooth(false, false);
			}
			
			//Gets the next node to travel to if possible
			
			do {
				if (mazeMap[currentX][currentY].getUnvisChildren().isEmpty()) {
					nextNode = null;
					break;
				}
				nextNode = mazeMap[currentX][currentY].getUnvisChildren().remove(0);
			} while (nextNode.getExplored());
			
			//Checks if the whole maze has been explored and the Red node found
			
			if (endNode != null && nextNode == null) {
				if (mazeExplored()) {
					Lcd.clear(8);
					Lcd.print(8, "Going to Red");
					//Moves from current node to the Red node
					if (endNode != mazeMap[currentX][currentY]) {
						for (Node next : aStar(mazeMap[currentX][currentY], endNode)) {
							Lcd.clear(2);
							Lcd.print(2, "Going to: " + next.toString());
							getBluetooth(false, false);
							moveToNode(pilot, next);
						}
						
						for (int i = 0; i < 21; i++) 
							for (int j = 0; j < 21; j++)
								mazeMap[i][j].resetNodeCost();
					}
					
					Lcd.clear(8);
					Lcd.print(8, "Going to Start");
					
					//Moves from the Red node to the Start node
					for (Node next : aStar(endNode, mazeMap[2][2])) {
						Lcd.clear(2);
						Lcd.print(2, "Going to: " + next.toString());
						getBluetooth(false, false);
						moveToNode(pilot, next);
					}
					
					break;
				}
			}
			
			//Travels to the next or previous node
			
			moveToNode(pilot, nextNode);
			getBluetooth(false, false);
		}
		
		pilot.stop();
        colour.close();
        irSensor.close();
        irMotor.close();
        gyro.close();
        
        getBluetooth(true, false);

        Sound.beepSequence();
        Button.LEDPattern(4);
        Lcd.clear(1);
        Lcd.print(1, "Press Escape");
        Button.waitForAnyPress();
		
	}
}