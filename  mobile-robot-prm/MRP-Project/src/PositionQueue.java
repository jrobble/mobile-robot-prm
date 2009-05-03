import javaclient2.PlayerClient;
import javaclient2.Position2DInterface;
import javaclient2.SonarInterface;
import javaclient2.structures.PlayerConstants;
import javaclient2.structures.PlayerPose;
import javaclient2.structures.sonar.PlayerSonarData;


public class PositionQueue extends Thread {

	public static final float PI = 3.14159265358979323846f; 
	public static final float EPSILON = 0.001f;
	
	public static boolean FLOAT_EQ(float x, float v) {
		return ((v - EPSILON) < x) && (x < (v + EPSILON));
	}
	
	private float ox, oy, otheta;
	private float cx = 0.0f, cy = 0.0f, ctheta;
	float totaldist = 0.0f;
	private int steps = 0;
	private float[] ranges;
	
	private long starttime;
	
	private PlayerClient robot = null;
	private Position2DInterface pp = null;
	private SonarInterface sp = null;
	
	public PositionQueue(PlayerClient robot, Position2DInterface pp, SonarInterface sp) {
		this.robot = robot;
		this.pp = pp;
		this.sp = sp;
		warmup();
	}
	
	public void tic() {
		starttime = System.currentTimeMillis();
	}
	
	public float toc() {
		// Get elapsed time in seconds
		long etimemills = System.currentTimeMillis()-starttime;
		printHMS(etimemills/1000);
		return etimemills;
	}
	
	// taken from: http://www.javaworld.com/javaworld/jw-03-2001/jw-0330-time.html
	public void printHMS(long timeInSeconds) {
		int hours, minutes, seconds;
	    hours =   (int) (timeInSeconds / 3600);
	    timeInSeconds = timeInSeconds - (hours * 3600);
	    minutes = (int) (timeInSeconds / 60);
	    timeInSeconds = timeInSeconds - (minutes * 60);
	    seconds = (int) timeInSeconds;
	    System.out.println(hours + " hour(s) " + minutes + " minute(s) " + seconds + " second(s)");
	}
	
	public float getCx() { return cx; }
	
	public float getCy() { return cy; }
	
	public float getCtheta() { return ctheta; }
	
	public int getSteps() { return steps; }
	
	public float getTotalDist() { return totaldist; }
	
	public synchronized boolean isStalled() {
		return pp.getData().getStall() > 0;
	}
	
	public float[] getRanges() { 
		updateSonar(); // on-demand
		return ranges; 
	}
	
	private synchronized void updatePosition() {
		// keep old position and heading
		ox = cx;
		oy = cy;
		otheta = ctheta;
		
		// read new position and heading		
		PlayerPose pose = pp.getData().getPos();
		cx = pose.getPx();
		cy = pose.getPy();
		ctheta = pose.getPa();
		
		totaldist += Math.sqrt( Math.pow(cx-ox, 2) + Math.pow(cy-oy, 2) );

		// System.out.println("ROBOT CTHETA: " + Math.toDegrees(ctheta)); // DEBUG
		
		// convert angles from real robot from 0 to 360 range to -180 to 180 range
		if(ctheta > PI) {
			ctheta = ctheta - 2*PI;
		}
		// ensure 180 instead of -180 for comparison purposes
		if(FLOAT_EQ(ctheta,-PI)) {
			ctheta = PI;
		}
		
		// System.out.printf("curr cx: %5.5f cy: %5.5f ctheta: %5.5f step: %d\n",
		//					cx,cy,Math.toDegrees(ctheta),steps); // DEBUG
	}
	
	private synchronized void updateSonar() {
	    PlayerSonarData sonardata = sp.getData();
		ranges = sonardata.getRanges();
	}
	
	// must read positional and sonar data
	public synchronized void readData() {
		robot.readAll();
		updatePosition();
		
		// doing this here causes a buffer underflow exception
		// so read sonar data on-demand
		// updateSonar();
	    
		steps++;
	}
	
	// may not be necessary
	private synchronized void warmup() {
		while(!sp.isDataReady()) {
			robot.readAll();
			updatePosition();
		}
		// System.out.println("PositionQueue warmed up..."); // DEBUG
	}
	
	public void run() {
		while(true) {
			readData();
		}
	}
	
	// set the robot's odometry
	public synchronized void setOdometry(float x, float y, float theta) {
		System.out.printf(">> SET ODOMETRY [%5.5f,%5.5f,%5.5f] ...\n",x,y,Math.toDegrees(theta)); // DEBUG
		PlayerPose pose = new PlayerPose();
		pose.setPx(x); pose.setPy(y); pose.setPa(theta);
		boolean valid = false;
		do {
			pp.setOdometry(pose); // [m,m,rad]
			readData();
			valid = FLOAT_EQ(cx,pose.getPx()) && FLOAT_EQ(cy,pose.getPy()) && FLOAT_EQ(ctheta,pose.getPa());
		} while(!valid);
		totaldist = 0.0f;
		System.out.printf(">> ODOMETRY SET [%5.5f,%5.5f,%5.5f]\n",x,y,Math.toDegrees(theta)); // DEBUG
	}
	
	
	// TEST
	public static void main(String args[]) {
		PlayerClient robot = new PlayerClient("localhost", 6665);
		Position2DInterface pp = robot.requestInterfacePosition2D(0,PlayerConstants.PLAYER_OPEN_MODE);
		SonarInterface sp = robot.requestInterfaceSonar(0,PlayerConstants.PLAYER_OPEN_MODE);
		
		PlayerPose pose;
		PlayerSonarData sonardata;
		float ranges[] = {};
		
		for(int i = 0; i < 100; i ++) {
			System.out.println("WARMING UP " + i); // DEBUG
			robot.readAll();
			pose = pp.getData().getPos();
			sonardata = sp.getData();
		}

		
		for(int i = 0; i < 200; i++) {
		// while(true) {
			System.out.printf(">> %d\n",i);
			// System.out.printf(">> %d GOT cx: %5.5f cy: %5.5f ctheta: %5.5f\n",i,pq.getCx(),pq.getCy(),Math.toDegrees(pq.getCtheta()));
			//try {
			//	sleep(1000);
			//} catch (InterruptedException e) {
			//	e.printStackTrace();
			//}
			
			robot.readAll();
			pose = pp.getData().getPos();
			sonardata = sp.getData();
			pp.setSpeed(0.0f, PI/8);
		}
		
		PositionQueue pq = new PositionQueue(robot,pp,sp);
		pq.start();
		
		
		System.out.println(">> PROB???");
		while(true) {
			pq.readData();
			System.out.printf(">> GOT cx: %5.5f cy: %5.5f ctheta: %5.5f\n",pq.getCx(),pq.getCy(),Math.toDegrees(pq.getCtheta()));
			pp.setSpeed(0.0f, PI/8); // causes buffer issue
		}
		
		
	}
	
}
