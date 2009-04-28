/*
 * RobotControl.java
 *
 *  Created on: Apr 17, 2009
 *      Author: jjr0192
 */

import java.util.Stack;

import javaclient2.PlayerClient;
import javaclient2.Position2DInterface;
import javaclient2.SonarInterface;
import javaclient2.structures.PlayerConstants;
import javaclient2.structures.PlayerPose;

/*
 * Program that solves three tasks in a single framework, both in 
 * simulation and on the robot. These tasks are localization, path 
 * planning, and real-time obstacle avoidance. Only the front-facing 
 * eight sonar sensors are available.
 * 
 * The robot (if in simulation) operates in the world defined by 
 * /usr/local/pub/zjb/playerstage/worlds/project.cfg. This world 
 * defines eight identical robots. These robots are differentiated 
 * by each listening for commands on a different port. This program 
 * takes a command line argument of a port number, which is used to 
 * find the correct Player server. 
 * 
 * When the simulation is run, either this code will run on one robot 
 * and a random walk on the other seven, or several teams' programs 
 * will run on different robots, with random walkers on the rest. 
 * This program will not know what the other robots are running, just 
 * that they might be in the way. Of course, in reality, there will be 
 * at most one other robot running, but there will be people to avoid 
 * instead!
 * 
 * The points that must be reached will be given in a file whose name 
 * is the second command-line argument to this program. There will be 
 * between one and five points to visit, and all of them will be 
 * reachable in simulation (i.e. not inside a room). These points 
 * may be visited in any order, and "visit" means that the robot's 
 * center stops with 0.5m of the given coordinates. 
 */
public class RobotControl {
	
	///////////////////////////////////////////////////////////////////
	// Variables
	///////////////////////////////////////////////////////////////////
	
	// global variables
	public static final float PI = 3.14159265358979323846f; 
	public static final float EPSILON = 0.001f;
	public static final float DEFAULT_FORWARD_SPEED  = 0.80f;  // m/s [1.00]
	public static final float DEFAULT_BACKWARD_SPEED = 0.20f;  // m/s
	public static final float DEFAULT_ANGULAR_SPEED  = (PI/4); // rads/s (10 * 16-18 deg., 10 * .279-.314 rads, is about pioneer max)
	public static final float MAX_TURNRATE = (float) (Math.PI/4); // rads [PI/8]
	
	public static boolean FLOAT_EQ(float x, float v) {
		return ((v - EPSILON) < x) && (x < (v + EPSILON));
	}
	
	// class variables
	private String server = null;
	private int port = -1;
	
	private float ox, oy, otheta, cx, cy, ctheta;
	private float speed, turnrate, oturnrate;
	private int steps = 0;
	private int still = 0;
	private boolean first = true; //  ODOMETRY HACK
	
	private PlayerClient robot = null;
	private Position2DInterface pp = null;
	private SonarInterface sp = null;
	private ProbRoadMap prm = null;
	private PositionQueue pq = null;
	
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
	
	public void printSonarRanges() {
		System.out.print("ranges: [ "); // DEBUG
		float ranges[] = pq.getRanges();
		for(int i = 0; i < ranges.length; i++) {
			System.out.format("%5.5f ", ranges[i]);
		}
		System.out.println("]");
	}
	
	// theta1 - theta2, ctheta - otheta
	private float angularDiff(float theta1, float theta2) {
		float t1 = theta1;
		float t2 = theta2;
		if(t1 < 0) { t1 = t1 + 2*PI; }
		if(t2 < 0) { t2 = t2 + 2*PI; }
		/*
		System.out.println( 
				" theta1: " + Math.toDegrees(theta1) + 
		        " theta2: " + Math.toDegrees(theta2) +
		        " diff: "   + Math.toDegrees(t1 - t2)); // DEBUG
		*/
		float diff = t1 - t2;
		if(diff < 0) {
			diff += 2*PI;
		}
		if(diff > PI) { // cw vs. ccw
			diff = Math.abs(diff-2*PI);
		}
		return diff;
	}

	// read positional
	private void readPosition() {
		// keep old position and heading
		ox = cx;
		oy = cy;
		otheta = ctheta;
		
		pq.readData();
		cx = pq.getCx();
		cy = pq.getCy();
		ctheta = pq.getCtheta();
		steps = pq.getSteps();
		
		System.out.printf("curr cx: %5.5f cy: %5.5f ctheta: %5.5f step: %d\n",
							cx,cy,Math.toDegrees(ctheta),steps); // DEBUG
	}
	
	
	///////////////////////////////////////////////////////////////////
	// Movement Methods
	///////////////////////////////////////////////////////////////////

	private void initialize() {
		// setup client and service proxies
		robot = new PlayerClient(server,port);
		robot.setNotThreaded();
		pp = robot.requestInterfacePosition2D(0,PlayerConstants.PLAYER_OPEN_MODE);
		// pp.setMotorPower(1);
		sp = robot.requestInterfaceSonar(0,PlayerConstants.PLAYER_OPEN_MODE);
		// sp.setSonarPower(1);
		pq = new PositionQueue(robot,pp,sp);
		pq.start();
	}
	
	// activate and control the robot
	public void run(double realdestpts[][]) {
		initialize();
		
		// initialize the probabilistic road map
		prm = new ProbRoadMap(500,realdestpts); // [1000] [500]
		// prm.setScaleFactor(2.0);
		// prm.setVisible(true);
		// prm.pack();
		
		boolean success = false;
		int startindex = 0; // red robot
		while(!success) {
			// DEBUG - plan next path
			System.out.println(">> PLAN NEXT PATH");
			Stack<Node> nodepath = prm.createPath(prm.planPath(startindex,8)); // robot 0 -> dest 0
			prm.reset();
			prm.drawPath(nodepath);
			prm.drawAllPoints();
			Retriever.pause(); // see path
			
			success = followPath(nodepath);
			
			if(!success) {
				startindex = prm.addPoint(cx,cy);
				prm.genAllEdges(); // account for new obstacles
				prm.reset();
				// prm.drawAllEdges();
				// prm.drawAllPoints();
				// pause();
			}
		}
		
		Retriever.pause();
		
		System.out.println("Terminating program.");
		robot.close();
		prm.dispose();
	}
	
	// instruct the robot to move along a path
	private boolean followPath(Stack<Node> nodepath) {
		Stack<Node> tmpnodepath = new Stack<Node>(); // don't modify the original
		tmpnodepath.addAll(nodepath);
		
		System.out.println(">> FOLLOWPATH length: " + tmpnodepath.size()); // DEBUG
		
		// TODO - HACK - set robot's current odometry (should localize instead)
		Node currnode = tmpnodepath.pop();
		if(first) {
			pq.setOdometry(currnode.realx,currnode.realy,0.0f);
			first = false;
		}
		
		boolean success = true;
		while(!tmpnodepath.isEmpty() && success) {
			currnode = tmpnodepath.pop();
			// goTo(currnode.realx, currnode.realy);
			success = potentialFieldMotion(currnode.realx,currnode.realy);
		}
		
		return success;
	}
	
	// determine the closet sonar to the given angle
	/*
	private int getClosestSonarIndex(float angle) {
		float tmpdiff;
		float mindiff = Float.POSITIVE_INFINITY;
		int minindex = -1;
		for(int i = 0; i < sonarposes.length; i++) {
			tmpdiff = Math.abs(sonarposes[i].getPa() - angle);
			if(tmpdiff < mindiff) {
				mindiff = tmpdiff;
				minindex = i;
			}
		}
		return minindex;
	}
	*/
	
	private boolean isSafe(float fattx, float fatty, float ranges[]) {
		return FLOAT_EQ(go(fattx,fatty,ranges,false),0.0f) ||
			   (ranges[3] > 0.2f && ranges[4] > 0.2f &&
		        ranges[2] > 0.1f && ranges[5] > 0.1f);
	}
	
	// move robot according to artificial potential field forces 
	// from current location to destination
	// based on: Autonomous Mobile Robots by Siegwart and Nourbaksh pages 267-270
	private boolean potentialFieldMotion(float nx, float ny) {

		float fx, fy, fattx, fatty; 
		float tmpfrepx, tmpfrepy, frep, frepx, frepy;
		float sonarangle, range, dirx, diry;
		boolean success = false;
		boolean cont = true;
		
		// constants
		float katt = 20.0f; // [0.20][10.20]
		float cap = 20.0f; // [50]
		// float p0 = 2.5f; // [1.5] [2.5] object distance of influence
		// float krep = 0.20f; // [0.20]
		
		// distance of influence and repulsive force of each front sonar
		float krep = 10.0f; // [8.20f]
		float p0 = 2.5f; // [2.5f]
		float p0s[]    = { p0, p0, p0, 5.0f, 5.0f, p0, p0, p0 };
		float kreps[]  = { 1.5f, krep, krep, krep, krep, krep, krep, 1.5f };
		
		// rotate in place towards initial waypoint
		readPosition();
		float totalangle = calcTotalAngle(nx-cx,ny-cy);
		rotate((float) Math.toDegrees(totalangle));
		
		while(cont && !success) {	
			readPosition();
			
			// read sonar, range[0] is leftmost
			// 90, 50, 30, 10, -10, -30, -50, -90 degrees
			printSonarRanges(); // DEBUG
			
			// calculate attractive force
			// F_att(q) = - k_att (q - q_goal), where q is a point (x,y) 
			fattx = -katt * (cx - nx);
			fatty = -katt * (cy - ny);
			
			// cap attractive force but maintain ratio
			float attratio = fattx/fatty;
			if(fattx > fatty) {
				if(Math.abs(fattx) > cap) { 
					fattx = cap * Math.signum(fattx); 
					fatty = Math.signum(fatty) * Math.abs(fattx * 1/attratio);  
				}
			} else {
				if(Math.abs(fatty) > cap) { 
					fatty = cap * Math.signum(fatty); 
					fattx = Math.signum(fattx) * Math.abs(fatty * attratio);  
				}
			}
		
			// calculate repulsive forces for each sonar reading
			frepx = 0.0f;
			frepy = 0.0f;
				
			float ranges[] = pq.getRanges();
			for(int i = 0; i < ranges.length; i++) { // 0...ranges.length
				range = ranges[i];
			
				if(range < p0s[i]) {
					tmpfrepx = 0.0f;
					tmpfrepy = 0.0f;
					
					// account for sonar geometry
					range += (float) Math.sqrt( Math.pow(sonarposes[i].getPx(),2) 
						    	                 + Math.pow(sonarposes[i].getPy(),2) );
					sonarangle = sonarposes[i].getPa();
					// System.out.printf("sonarangle: %5.5f\n",Math.toDegrees(sonarangle)); // DEBUG
					
					// F_rep(q) = k_rep ( 1/p(q) - 1/p0 ) 1/p^2(q)  if p(q) < p0
					// 0  if p(q) >= p0
					frep = (float) (kreps[i] * ( (1/range) - (1/p0s[i]) ) * ( 1/Math.pow(range,2) ) * 1);
					
					tmpfrepx = (float) (frep * Math.abs(Math.cos(sonarangle)));
					tmpfrepy = (float) (frep * Math.abs(Math.sin(sonarangle)));
					
					dirx = (float) Math.cos(ctheta + sonarangle); // left or right
					diry = (float) Math.sin(ctheta + sonarangle); // up or down
					
					tmpfrepx = -Math.signum(dirx) * tmpfrepx;
					tmpfrepy = -Math.signum(diry) * tmpfrepy;
					
					frepx += tmpfrepx;
					frepy += tmpfrepy;
					
					// System.out.printf("ranges[%d] frep: %5.5f tmpfrep: [%5.5f, %5.5f] frepxy: [%5.5f, %5.5f]\n",
					//	i,frep,tmpfrepx,tmpfrepy,frepx,frepy); // DEBUG
				}
			}
			
			// F(q) = F_att(q) + F_rep(q) 
			fx = fattx + frepx; 
			fy = fatty + frepy;
			
			System.out.printf("fatt: [%5.5f, %5.5f] f: [%5.5f, %5.5f]\n",fattx,fatty,fx,fy); // DEBUG


			// what % of the force is opposing the robot's current heading?
			float oppx = (float) (frepx * Math.cos(ctheta));
			float oppy = (float) (frepy * Math.sin(ctheta));
			float oppratio = Math.abs(oppx/oppy);
			System.out.printf(">> oppx: %f5.5 oppy: %f5.5 oppratio: %f5.5\n",oppx,oppy,oppratio); // DEBUG
			
			// TODO oppx + oppy ???
			float opph = (float) Math.sqrt( Math.pow(oppx,2) + Math.pow(oppy,2) );
			System.out.printf(">> >> >> >> frepx: %f5.5 frepy: %f5.5\n",frepx,frepy); // DEBUG
			System.out.printf(">> >> >> >> opph: %f5.5 still: %d\n",opph,still); // DEBUG
			
			
			// if(oppratio > 50.0f || Math.abs(oppx) > 30.0f) {
			if(still > 30 || !isSafe(fattx,fatty,ranges)) { // still > 30 opph > 15.0f
				
				// if we can keep moving forward towards the destination
				float buff = 0.25f;
				if(go(fattx,fatty,ranges,false) > 0.0f &&
				   ((ranges[3] > 0.3f && ranges[4] > 0.1f) || (ranges[4] > 0.3f && ranges[3] > 0.1f)) && // 1.0f
				   ranges[0] > 0.1f && ranges[1] > 0.1f && ranges[2] > buff && 
				   ranges[5] > buff && ranges[6] > 0.1f && ranges[7] > 0.1f) {
					System.out.println(">> CONTINUE FORWARD"); // DEBUG
					
					// TODO - go towards most "free" forward sonar....
					
					go(fattx,fatty,ranges,true);
				} else {
					// replan
					System.out.println("\n\n>> REPLAN\n"); // DEBUG
					// System.exit(0); // DEBUG
					stop(); 
					foundObstacle();
					
					// rotate towards goal
					// float totalangle = calcTotalAngle(fattx,fatty);
					// rotate(Math.signum(totalangle) * 45);
					
					cont = false;
					still = 0;
				}

			} else {
				// normal potential field motion
				go(fx,fy,ranges,true);
			}
			
			// destination reached?
			float mindist = (float) Math.sqrt( Math.pow(nx-cx, 2) + Math.pow(ny-cy, 2) );
			System.out.println(">> DEST [" + nx + "," + ny + "] mindist: " + mindist); // DEBUG
			// success = mindist < 0.50;
			success = mindist < 0.50;
			if(success) { stop(); }
 			
		}
		
		return success;
	}
	
	
	///////////////////////////////////////////////////////////////////
	// Actuation Methods
	///////////////////////////////////////////////////////////////////
	
	// rotate the robot a certain number of degrees
	private void rotate(float degrees) {
		float angle = 0;
		float totalangle = (float) Math.toRadians(degrees);
		boolean success = false;
		speed = 0.0f;
		
		System.out.println(">> ROTATE " + degrees); // DEBUG
		while (!success) {
			// get position from player interface
			readPosition();
			
			if(FLOAT_EQ(angle,totalangle)) {
				success = true;
			} else {
				// TODO - sim only?
				turnrate = totalangle - angle;
				if (Math.abs(turnrate) > MAX_TURNRATE) {
					turnrate = Math.signum(turnrate) * MAX_TURNRATE;
				}
				// angle += turnrate/10; // this is not guaranteed in Java
				angle += Math.signum(turnrate) * angularDiff(ctheta,otheta);
				// System.out.printf("angle: %5.5f | %5.5f\n",Math.toDegrees(angle),Math.toDegrees(totalangle)); // DEBUG
			}

			// command the motors
			System.out.printf("SetSpeed(%5.5f, %5.5f)\n",speed,Math.toDegrees(turnrate));
			pp.setSpeed(speed, turnrate);
		}
	}
	
	// stop the robot
	private void stop() {
		// get position from player interface
		readPosition();
		speed = 0.0f; turnrate = 0.0f;
		pp.setSpeed(speed, turnrate);
	}
	
	
	// example obstacle-avoidance motion
	private void simpleMotion(float ranges[]) {
		// get position from player interface
		readPosition();
		
		float turnrate, speed;
		if (ranges[0] + ranges[1] + ranges[2] < ranges[5] + ranges[6] + ranges[7]) {
			turnrate = -30.0f * (float)Math.PI / 180.0f;
		} else {
			turnrate = 30.0f * (float)Math.PI / 180.0f;
		}
		
		//if (ranges[3] < 0.5f) {
		//	speed = 0.0f;
		//} else {
			speed = 0.1f;
		//}
		
		// send the command
		pp.setSpeed(speed, turnrate);
	}
	
	// calculate the total angle between the robot and the vector that points to the goal
	private float calcTotalAngle(float dx, float dy) {
		float dtheta, totalangle;
		
		// determine next angle
		dtheta = (float) Math.atan2(dy,dx);
		
		
		// convert angles from real robot from 0 to 360 range to -180 to 180 range
		if(dtheta > PI) {
			dtheta = dtheta - 2*PI;
		}
		// ensure 180 instead of -180 for comparison purposes
		if(FLOAT_EQ(dtheta,-PI)) {
			dtheta = PI;
		}
		
		
		totalangle = dtheta-ctheta;
		
		// prevent values > 360
		if(totalangle > 2*PI) {
			totalangle -= 2*PI;
		}
		if(totalangle < -2*PI) {
			totalangle += 2*PI;
		}
		// determine if we should go ccw instead of cw
		if(totalangle > PI) {
			totalangle -= 2*PI;
		}
		if(totalangle < -PI) {
			totalangle += 2*PI;
		}
		
		System.out.println("DTHETA: " + Math.toDegrees(dtheta) + " TOTALANGLE: " + Math.toDegrees(totalangle)); // DEBUG
		return totalangle;
	}
	
	// dx, dy - change specified in world offset coordinates
	private float go(float dx, float dy, float ranges[], boolean move) {
	
		float totaldist, totalangle;
		float tmpturnrate;

		// get position from player interface
		// readPosition();
		
		totalangle = calcTotalAngle(dx,dy);
		
		// System.out.printf("totalangle: %5.5f\n", turnrate); // DEBUG
		if(move) { oturnrate = turnrate; };
		tmpturnrate = totalangle;
		
		// determine next distance
		totaldist = (float) Math.sqrt(Math.pow(dx,2)+Math.pow(dy,2));
		speed = totaldist;
		
		// cap turnrate and speed
		if(Math.abs(tmpturnrate) > MAX_TURNRATE) {
			System.out.println(">> CAP TURN"); // DEBUG
			tmpturnrate = Math.signum(tmpturnrate) * MAX_TURNRATE;
			if(move) { still += 1; };
		} else {
			if(move) { still = 0; };
		}
		
		if(move) {
			if(FLOAT_EQ(Math.abs(oturnrate),MAX_TURNRATE) &&
			   FLOAT_EQ(Math.abs(tmpturnrate) ,MAX_TURNRATE) &&
			   Math.signum(oturnrate) != Math.signum(tmpturnrate)) {
				still += 10; // HACK [50]
			}
		}
		
		if(Math.abs(speed) > DEFAULT_FORWARD_SPEED) {
			speed = Math.signum(speed) * DEFAULT_FORWARD_SPEED;
		}
		
		// the more we need to turn, the slower we should go
		speed = speed * (1 - Math.abs((tmpturnrate/MAX_TURNRATE)));
		
		// if we're boxed in, go slower
		/*
		float minrange = Float.POSITIVE_INFINITY;
		for(int i = 2; i <= 5; i++) {
			if(ranges[i] < minrange) {
				minrange = ranges[i];
			}
		}
		if(minrange < 0.5f) {
			speed = 0.3f;
		}
		*/
		
		
		// go there
		if (pq.isStalled()) {
			System.out.printf(">> GO [%5.5f,%5.5f]\n",dx,dy); // DEBUG
			System.out.println(">> STALLED - TERMINATE PROGRAM"); // DEBUG
			System.exit(1);
			
		} 
			
		// System.out.printf("totalangle: %5.5f\n",Math.toDegrees(totalangle)); // DEBUG
		// System.out.printf("totaldist:  %5.5f\n",totaldist); // DEBUG
		
		// command the motors
		if(move) {
			turnrate = tmpturnrate;
			System.out.printf("SetSpeed(%5.5f, %5.5f)\n",speed,Math.toDegrees(tmpturnrate));
			pp.setSpeed(speed, tmpturnrate);
		}
		return speed;
	}
	
	/*
	// travel from current location to destination
	// nx, ny - destination specified in world offset coordinates
	private void goTo(float nx, float ny) {
		System.out.printf(">> GOTO [%5.5f,%5.5f]\n",nx,ny); // DEBUG
		
		float dx, dy, dtheta, ntheta;
		float dist = 0.0f, totaldist = 0.0f; 
		float angle, totalangle;
		float speed, turnrate;
		boolean success, turning, movingforward;

		// get starting position from player interface
		readPosition();

		// determine next angle
		dx = nx - cx;
		dy = ny - cy;
		dtheta = (float) Math.atan2(dy,dx);
		
		totalangle = dtheta-ctheta;
		angle = 0.0f;

		// prevent values > 360
		if(totalangle > 2*PI) {
			totalangle -= 2*PI;
		}
		// determine if we should go ccw instead of cw
		if(totalangle > PI) {
			totalangle -= 2*PI;
		}

		// set the turnrate
		if(totalangle > 0) {
			turnrate = DEFAULT_ANGULAR_SPEED;
		} else {
			turnrate = -DEFAULT_ANGULAR_SPEED;
		}

		ntheta = dtheta;
		// cout << "NTHETA:         " << radiansToDegrees(ntheta) << endl; // DEBUG
		// determine if we should go ccw instead of cw
		if(ntheta > PI) {
			ntheta -= 2*PI;
		}
		// ensure 180 instead of -180 for comparison purposes
		if(FLOAT_EQ(ntheta,-PI)) {
			ntheta = PI;
		}
		
		speed = 0.0f; // turn in place first
		totaldist = (float) Math.sqrt(Math.pow(dx,2)+Math.pow(dy,2));

		// DEBUG
		System.out.printf("DTHETA     %5.5f\n",Math.toDegrees(dtheta));
		System.out.printf("TOTALANGLE %5.5f\n",Math.toDegrees(totalangle));
		System.out.printf("TOTALDIST  %5.5f\n",Math.toDegrees(totaldist));
		
		// go there
		success = false;
		turning = true;
		movingforward = false;
		steps = 0;
		
		while (!success) {
			// System.out.println(">> LOOP"); // DEBUG
			
			if (pp.getData().getStall() > 0) {
				System.out.println(">> STALLED - TERMINATE PROGRAM"); // DEBUG
				System.exit(1);
			
			// } else if (FLOAT_EQ(cx,nx) && FLOAT_EQ(cy,ny)) { // TODO - better?
			} else if(FLOAT_EQ(dist,totaldist)) {	
				System.out.println(">> DESTINATION REACHED\n"); // DEBUG
				speed = 0.0f;
				success = true;
				movingforward = false;
			} else {
				steps++;
				// are we heading in the right direction?
				if(turning) { // if turning
					if(FLOAT_EQ(angle,totalangle)) {
						System.out.println(">> ORIENTATION REACHED\n"); // DEBUG
						turnrate = 0.0f;
						dist = 0.0f;
						speed = DEFAULT_FORWARD_SPEED;
						turning = false;
						movingforward = true;
					} else {

						// TODO - sim only?
						turnrate = totalangle - angle;
						if (Math.abs(turnrate) > MAX_TURNRATE) {
							turnrate = Math.signum(turnrate) * MAX_TURNRATE;
						}
						// angle += turnrate/10; // this is not guaranteed in Java
						angle += Math.signum(turnrate) * angularDiff(ctheta,otheta);
						System.out.printf("angle: %5.5f | %5.5f\n",Math.toDegrees(angle),Math.toDegrees(totalangle)); // DEBUG
					}
				}

				// change speed to get better position if necessary
				if(movingforward) { // if moving forward

					// TODO - sim only?
					speed = totaldist - dist;
					if (Math.abs(speed) > DEFAULT_FORWARD_SPEED) {
						speed = Math.signum(speed) * DEFAULT_FORWARD_SPEED;
					}
					// dist += speed/10;  // this is not guaranteed in Java
					dist += distance(cx,ox,cy,oy);
					System.out.printf("dist: %5.5f | %5.5f\n",dist,totaldist); // DEBUG
				}

				// command the motors
				System.out.printf("SetSpeed(%5.5f, %5.5f)\n",speed,Math.toDegrees(turnrate));
				pp.setSpeed(speed, turnrate);

				// get position from player interface
				readPosition();
			}
		}
	}
	*/
	
	///////////////////////////////////////////////////////////////////
	// Map Update Methods
	///////////////////////////////////////////////////////////////////
	
	// form a cusp representing the obstacle boundary and update obstacle map
	private void foundObstacle() {
		System.out.println(">> FOUND OBSTACLE"); // DEBUG
		
		float range, theta, dist, offsetdist;
		float rtheta, rx, ry; // real-world coordinates local to robot
		
		float ranges[] = pq.getRanges();
		for(int i = 0; i < ranges.length; i++) {
			range = ranges[i];
			
			if(range < 1.0f) { // [2.5]
				// offset distance based on sonar geometry
				offsetdist = (float) Math.sqrt( Math.pow(sonarposes[i].getPx(),2) 
						                      + Math.pow(sonarposes[i].getPy(),2));
				range += offsetdist;
		
				// angular sweep
				float d = -7.5f;
				while(d < 7.5) {
					theta = (float) Math.toRadians(d) + sonarposes[i].getPa();
					
					// distance sweep
					for(int p = 0; p < 3; p++) {
						dist = (float) (range + (p * ProbRoadMap.MPP));
		
						// determine grid cell
						rtheta = ctheta + theta;
						rx = (float) Math.cos(rtheta) * dist + cx;
						ry = (float) Math.sin(rtheta) * dist + cy;
						
						prm.setVal(rx, ry);
					}
					
					d += 1.0;
				}
			}
		}
		
		prm.draw(); // DEBUG
		System.out.println(">> UPDATED PRM"); 
	}
	
}
