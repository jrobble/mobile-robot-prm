/*
 * RobotControl.java
 *
 *  Created on: Apr 2, 2009
 *      Author: jjr0192
 */

import javaclient2.PlayerClient;
import javaclient2.Position2DInterface;
import javaclient2.SonarInterface;
import javaclient2.structures.PlayerConstants;
import javaclient2.structures.PlayerPose;
import javaclient2.structures.sonar.PlayerSonarData;

/*
 * Use sensor data to build a map of the robot's environment. 
 * In particular, build a two-dimensional evidence grid based on sonar
 * data. 
 * 
 * The robot will be able to wander around in the world defined by
 * simple-sonar.cfg (note that there are two robots in this world, 
 * listening on ports 6665 and 6666) and generate a map.
 * 
 * Since we are accumulating evidence, we will need to keep the map
 * data around somehow and use that to compute the new evidence after 
 * each sonar scan. Depending on the complexity and resolution of 
 * the update, we may be able to run the update in the robot control 
 * thread without a problem (making sure to only draw the map once 
 * per cycle!).
 * 
 * simple-sonar.cfg world size: [16 16] meters
 */
public class RobotControl {
	
	///////////////////////////////////////////////////////////////////
	// Variables
	///////////////////////////////////////////////////////////////////
	
	// wall-following variables
	private static final float DESIRED_DIST_TO_WALL = 0.5f; // m [0.5]
	private static final float MAX_TURNRATE = (float) (Math.PI/4); // rads [PI/8]
	private static final int k = 15; // feedback control loop constant [7.5,20]
	
	// global variables
	public static final float PI = 3.14159265358979323846f; 
	public static final float EPSILON = 0.0001f;
	public static final float DEFAULT_FORWARD_SPEED  = 0.50f;  // m/s [0.50]
	public static final float DEFAULT_BACKWARD_SPEED = 0.20f;  // m/s
	public static final float DEFAULT_ANGULAR_SPEED  = (PI/4); // rads/s (10 * 16-18 deg., 10 * .279-.314 rads, is about pioneer max)
	
	public static boolean FLOAT_EQ(float x, float v) {
		return ((v - EPSILON) < x) && (x < (v + EPSILON));
	}
	
	// class variables
	private String server = null;
	private int port = -1;
	
	private float ox, oy, otheta, cx, cy, ctheta;
	private int steps = 0;
	
	private PlayerClient robot = null;
	private Position2DInterface pp = null;
	private SonarInterface sp = null;
	private GridMap map = null;
	
	// sensor geometry - hardcoded since SonarInterface.getGeom() is inaccurate
	// forward of robot is positive x, right of robot is positive y, left of robot is negative y
	private PlayerPose sonarposes[] = new PlayerPose[8];
	{
		for(int i = 0; i < sonarposes.length; i++) { sonarposes[i] = new PlayerPose(); };
		sonarposes[0].setPx(0.075f); sonarposes[0].setPy(0.130f); sonarposes[0].setPa((float) Math.toRadians(90));
		sonarposes[1].setPx(0.115f); sonarposes[1].setPy(0.115f); sonarposes[1].setPa((float) Math.toRadians(50));
		sonarposes[2].setPx(0.150f); sonarposes[2].setPy(0.080f); sonarposes[2].setPa((float) Math.toRadians(30));
		sonarposes[3].setPx(0.170f); sonarposes[3].setPy(0.025f); sonarposes[3].setPa((float) Math.toRadians(10));
		sonarposes[4].setPx(0.170f); sonarposes[4].setPy(-0.025f); sonarposes[4].setPa((float) Math.toRadians(-10));
		sonarposes[5].setPx(0.150f); sonarposes[5].setPy(-0.080f); sonarposes[5].setPa((float) Math.toRadians(-30));
		sonarposes[6].setPx(0.115f); sonarposes[6].setPy(-0.115f); sonarposes[6].setPa((float) Math.toRadians(-50));
		sonarposes[7].setPx(0.075f); sonarposes[7].setPy(-0.130f); sonarposes[7].setPa((float) Math.toRadians(-90));
	}
	private float sonarrangemin = 0.0f;
	private float sonarrangemax = 5.0f;
	private float sonarviewangle = (float) (Math.toRadians(15.0));
	
	// initial robot positions in global world coordinates [x,y,theta]
	public static final float initial_pos0[] = {-15.5f,  12.0f,   0.0f}; // red     (port 6665)
	public static final float initial_pos1[] = {-16.5f,  12.0f, 180.0f}; // green   (port 6666)
	public static final float initial_pos2[] = { -5.0f, -10.5f,   0.0f}; // cyan    (port 6667)
	public static final float initial_pos3[] = {  7.5f,   1.0f,  90.0f}; // magenta (port 6668)
	public static final float initial_pos4[] = {-48.0f,  12.0f,  90.0f}; // yellow  (port 6669)
	public static final float initial_pos5[] = {-48.0f, -10.5f, 270.0f}; // grey    (port 6670)
	public static final float initial_pos6[] = {  7.5f,  -5.0f,  90.0f}; // blue    (port 6671)
	public static final float initial_pos7[] = {  0.0f,  -7.0f, 270.0f}; // white   (port 6672)
	public static final float all_initial_pos[][] = {
		initial_pos0, initial_pos1, initial_pos2, initial_pos3, initial_pos4, initial_pos5, initial_pos6, initial_pos7 };
	
	
	///////////////////////////////////////////////////////////////////
	// Methods
	///////////////////////////////////////////////////////////////////
	
	// constructor
	public RobotControl() {
		this.server = "localhost";
		this.port = 6665;
	}

	public RobotControl(String server, int port) {
		this.server = server;
		this.port = port;
	}
	
	public void printSonarGeometry() {
		/*
		while(!sp.isGeomReady()) {
			System.out.println("waiting for sonar geometry...");
			readPosition();
		}
		// get sonar geometry
		PlayerSonarGeom sonargeom = sp.getGeom();
		PlayerPose poses[] = sonargeom.getPoses();
		System.out.println("geom:");
		for(int i = 0; i < poses.length; i++) {
			System.out.println("[ " + poses[i].getPx() + " " + poses[i].getPy() + " " + Math.toDegrees(poses[i].getPa()) + " ]");
		}
		*/
		System.out.println("geom:");
		for(int i = 0; i < sonarposes.length; i++) {
			System.out.println("[ " + sonarposes[i].getPx() + " " + sonarposes[i].getPy() + " " + Math.toDegrees(sonarposes[i].getPa()) + " ]");
		}
	}
	
	// theta1 - theta2, ctheta - otheta
	float angularDiff(float theta1, float theta2) {
		float t1 = theta1;
		float t2 = theta2;
		if(t1 < 0) { t1 = t1 + 2*PI; }
		if(t2 < 0) { t2 = t2 + 2*PI; }
		System.out.println( 
				" theta1: " + Math.toDegrees(theta1) + 
		        " theta2: " + Math.toDegrees(theta2) +
		        " diff: "   + Math.toDegrees(t1 - t2)); // DEBUG
		float diff = t1 - t2;
		if(diff < 0) {
			diff += 2*PI;
		}
		if(diff > PI) { // cw vs. ccw
			diff = Math.abs(diff-2*PI);
		}
		return diff;
	}

	float distance(float x1, float x2, float y1, float y2) {
	        return (float) Math.sqrt(Math.pow(x2-x1,2)+Math.pow(y2-y1,2));
	}

	void readPosition() {
		robot.readAll();
		// keep old position and heading
		ox = cx;
		oy = cy;
		otheta = ctheta;
		
		// read new position and heading
		// cx = pp.getX();
		// cy = pp.getY();
		// ctheta = pp.getYaw(); // C++ ONLY
		
		PlayerPose pose = pp.getData().getPos();
		cx = pose.getPx();
		cy = pose.getPy();
		ctheta = pose.getPa();

		// System.out.println("ROBOT CTHETA: " + Math.toDegrees(ctheta)); // DEBUG
		
		// convert angles from real robot from 0 to 360 range to -180 to 180 range
		if(ctheta > PI) {
			ctheta = ctheta - 2*PI;
		}
		// ensure 180 instead of -180 for comparison purposes
		if(FLOAT_EQ(ctheta,-PI)) {
			ctheta = PI;
		}
		/*
		System.out.println("curr: cx " + cx +
				                " cy " + cy + 
				                " ctheta " + Math.toDegrees(ctheta) +
				                " steps " + steps); // DEBUG
	    */
	    steps++;
	}

	void run(int mode) {
		// setup client and service proxies
		robot = new PlayerClient(server,port);
		pp = robot.requestInterfacePosition2D(0,PlayerConstants.PLAYER_OPEN_MODE);
		// pp.setMotorPower(1);
		sp = robot.requestInterfaceSonar(0,PlayerConstants.PLAYER_OPEN_MODE);
		// sp.setSonarPower(1);
		
		// create zero-centered evidence map
		// account for possible initial corner starting location and diagonal heading
		// world size [131.2 41], resolution [0.082]
		int mapwidth =  280; // [280]
		int mapheight = 100; // [100]
		double mpp = 0.082; // [0.1] 10 cm/px
		map = new GridMap(mapwidth,mapheight,mpp);
		map.setScaleFactor(0.5); // [0.5]
		map.setVisible(true);
		map.pack();
		 
		//       | +x,+y
		//   ____|____
		//       |
		// -x,-y |
		/*
		map.setVal(0, 0, 0); // center-ish
		map.setVal(-mapwidth/2,      mapheight/2,        0); // left  upper
		map.setVal(mapwidth/2 - mpp, mapheight/2,        0); // right upper
		map.setVal(-mapwidth/2,      -mapheight/2 + mpp, 0); // left  lower
		map.setVal(mapwidth/2 - mpp, -mapheight/2 + mpp, 0); // right lower
		*/

		// get starting position from player interface
		readPosition();
		
		// wait for sonar to warm up
		sp.queryGeometry();
		while(!sp.isDataReady()){
			System.out.println("sonar warming up...");
			readPosition();
		}
		printSonarGeometry(); // DEBUG
		
		boolean error = false;
		while(!error) {
			// read sonar, range[0] is leftmost
			// -90, -50, -30, -10, 10, 30, 50, 90 degrees
			PlayerSonarData sonardata = sp.getData();
			float ranges[] = sonardata.getRanges();
		
			/*
			System.out.print("ranges: [ "); // DEBUG
			for(int i = 0; i < ranges.length; i++) {
				System.out.format("%5.4f ", ranges[i]);
			}
			System.out.println("]");
			*/
			
			// move based on the mode
			switch(mode) {
				case 0:
					simpleMotion(ranges);
					break;
				case 1:
					// wallFollowMotion(ranges);
					break;
			} 
			
			// calculate obstacle probabilities
			float theta;
			float range;
			float offsetdist;
			
			int angsteps = 30; // [6][30] ensure even: left, ..., mid, ..., right
			float deltatheta, parttheta;
			int diststeps = 50; // [10][50] ensure even: 0, ..., dist
			float deltadist, partdist;
			float rtheta, rx, ry; // real-world coordinates local to robot
			
			// sonar sweep
			for(int s = 0; s < sonarposes.length; s++) { // DEBUG  s < sonarposes.length
				theta = sonarposes[s].getPa();
				range = ranges[s];
				// System.out.println("sonar: " + s + " theta: " + Math.toDegrees(theta) + " dist: " + dist); // DEBUG
				
				// offset distance based on sonar geometry
				offsetdist = (float) Math.sqrt(Math.pow(sonarposes[s].getPx(),2)+Math.pow(sonarposes[2].getPy(),2));
				
				// angular sweep
				for(int i = 0; i <= angsteps; i++) {
					deltatheta = (i * sonarviewangle/angsteps) - (sonarviewangle/2);
					parttheta = theta + deltatheta;
					// System.out.println("deltatheta: " + Math.toDegrees(deltatheta) + " parttheta: " + Math.toDegrees(parttheta)); // DEBUG
				
					// distance sweep
					for(int j = 0; j <= diststeps; j++) {
						deltadist = j * (sonarrangemax/diststeps);
						partdist = deltadist + offsetdist;
						// System.out.println("deltadist: " + deltadist + " partdist: " + partdist); // DEBUG
						
						// determine grid cell
						rtheta = ctheta + parttheta;
						rx = (float) Math.cos(rtheta) * partdist + cx;
						ry = (float) Math.sin(rtheta) * partdist + cy;
						// System.out.println("rtheta: " + Math.toDegrees(rtheta) + " rx: " + rx + " ry: " + ry); // DEBUG
						
						// map.setValue(rx, ry, (1-partdist/8)); // DEBUG
						map.calcProbOccupancy(rx,ry,range,deltadist,deltatheta);
					}	
				}
			}
			
			// update map visuals on a regular basis
			if(steps % 4 == 0) {
				map.draw();
			}
			
			// get position from player interface
			readPosition();
		}
	}

	// example obstacle-avoidance motion
	private void simpleMotion(float ranges[]) {
		float turnrate, speed;
		if (ranges[0] + ranges[1] + ranges[2] < ranges[5] + ranges[6] + ranges[7]) {
			turnrate = -30.0f * (float)Math.PI / 180.0f;
		} else {
			turnrate = 30.0f * (float)Math.PI / 180.0f;
		}
		
		if (ranges[3] < 0.5f) {
			speed = 0.0f;
		} else {
			speed = 0.4f;
		}
		
		// send the command
		pp.setSpeed(speed, turnrate);
	}
}
