import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Scanner;

/*
 * Retriever.java
 *
 *  Created on: Apr 13, 2009
 *      Author: jjr0192
 */

// /System/Library/Frameworks/JavaVM.framework/Versions/1.5.0/Home/bin/java 
// -Dfile.encoding=MacRoman 
// -classpath /Users/fejjro/Desktop/eclipseGJava/workspace/MRP-Project/bin:
//            /Users/fejjro/Desktop/Javaclient2-2.0.1/jars/javaclient.jar Retriever

public class Retriever {
	
	// process the data points file
	private static double[][] readPts(String filename) throws FileNotFoundException {
		// process the data points file
		Scanner scan = new Scanner(new FileReader(filename));
		double pts[][] = new double[20][2];
		int count = 0;
		while(scan.hasNext()) {
			pts[count][0] = scan.nextDouble();
			pts[count][1] = scan.nextDouble();
			count += 1;
			// expand point array if necessary
			if(count+1 > pts.length && scan.hasNext()) {
				System.out.println("Expand point array."); // DEBUG
				double tmppts[][] = pts;
				pts = new double[pts.length*2][2];
				for(int i = 0; i < tmppts.length; i++) {
					pts[i][0] = tmppts[i][0];
					pts[i][1] = tmppts[i][1];
				}
			}
		}
		// resize point array
		double tmppts[][] = pts;
		pts = new double[count][2];
		for(int i = 0; i < count; i++) {
			pts[i][0] = tmppts[i][0];
			pts[i][1] = tmppts[i][1];
		}
		return pts;
	}
	
	// print points
	public static void printPts(double[][] pts) {
		for(int i = 0; i < pts.length; i++) {
			System.out.println("point " + i + ": [" + pts[i][0] + "," + pts[i][1] + "]");
		}
	}
	public static void printPts(int[][] pts) {
		for(int i = 0; i < pts.length; i++) {
			System.out.println("point " + i + ": [" + pts[i][0] + "," + pts[i][1] + "]");
		}
	}

	// main method
	public static void main(String args[]) {
		try {
			// if(args.length == 1 || args.length == 3)
			
			// process the data points file
			double pts[][] = readPts(args[0]);
			
			System.out.println("World destination points:");
			printPts(pts); // DEBUG
			
			// initialize the probabilistic road map
			ProbRoadMap prm = new ProbRoadMap(500,pts); // [1000] [500]
			prm.setScaleFactor(2.0);
			prm.setVisible(true);
			prm.pack();
			
			
			
			// for ...
			/*
	        ProbRoadMap.Node lastnode = prm.planPath(8,9); // DEBUG - mapdestpt 0 -> 1
	        if(lastnode != null) {
	        	prm.drawPath(lastnode);
	        } else {
	        	System.out.println("Couldn't plan path!");
	        }
	        */
	        

			
			while(true) {
				// DEBUG
			}
			
			
			/*
			try {
				System.out.println("Waiting ...");
				Thread.sleep(99999999);
			} catch (InterruptedException e) {
				e.printStackTrace();
			} // DEBUG
			*/
			/*
			RobotControl rc = null;
			int mode = 0;
			
			// configure robot
			if(args.length < 2) {
				rc = new RobotControl();
			}else if(args.length < 4){
				String server = args[0];
				int port = Integer.parseInt(args[1]);
				rc = new RobotControl(server,port);
			}else{
				System.out.println("Usage: java Retriever [mode]");
				System.out.println("Usage: java Retriever host port [mode]");
			}
			
			// determine movement mode and run
			if(args.length == 1) {
				mode = Integer.parseInt(args[0]);
			} else if(args.length == 3) {
				mode = Integer.parseInt(args[2]);
			}
			rc.run(mode);
			*/
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}
}