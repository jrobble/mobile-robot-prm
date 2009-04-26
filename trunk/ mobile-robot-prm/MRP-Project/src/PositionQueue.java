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
	
	private float ox, oy, otheta, cx, cy, ctheta;
	private int steps = 0;
	
	private PlayerClient robot = null;
	private Position2DInterface pp = null;
	private SonarInterface sp = null;
	
	public PositionQueue(PlayerClient robot, Position2DInterface pp, SonarInterface sp) {
		this.robot = robot;
		this.pp = pp;
		this.sp = sp;
	}
	
	public float getCx() { return cx; }
	
	public float getCy() { return cy; }
	
	public float getCtheta() { return ctheta; }
	
	// must read positional and sonar data
	public void readPosition() {
		robot.readAll();
		// keep old position and heading
		ox = cx;
		oy = cy;
		otheta = ctheta;
		
		// read new position and heading		
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
		
		System.out.printf("curr cx: %5.5f cy: %5.5f ctheta: %5.5f step: %d\n",
							cx,cy,Math.toDegrees(ctheta),steps); // DEBUG
	    
	    steps++;
	    
	    PlayerSonarData sonardata = sp.getData();
	}
	
	public void run() {
		while(true) {
			readPosition();
		}
	}
	
	
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
			// ranges = sonardata.getRanges();
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
			pq.readPosition();
			System.out.printf(">> GOT cx: %5.5f cy: %5.5f ctheta: %5.5f\n",pq.getCx(),pq.getCy(),Math.toDegrees(pq.getCtheta()));
			pp.setSpeed(0.0f, PI/8); // causes buffer issue
		}
		
		
	}
	
}
