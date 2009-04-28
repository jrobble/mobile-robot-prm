import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Stack;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

/*
 * ProbRoadMap.java
 * Based partly on Zack Butler's GridMap.java code.
 *
 *  Created on: Apr 16, 2009
 *      Author: jjr0192
 */

public class ProbRoadMap extends JFrame {

	///////////////////////////////////////////////////////////////////
	// Variables
	///////////////////////////////////////////////////////////////////
	
	private static final long serialVersionUID = 8130028453731235660L;
	
	// The robot is [0.50,0.37] m, so use a circular structuring element with a radius of 0.50 m
	// If MPP = 0.082, then structuring element should have a radius of 6.089 px => 7 px
	// If the c-space were calculated, there would be < 4 px of space in the robotics lab doorway.
	// It is very unlikely that two PRM random pts would be generated to provide a suitable path in.
	// Thus, consider using potential field motion for obstacle avoidance.
	
	// raw world map is a 1600*500 file containing byte values of either 0 or 255
	private static final String filename = "../3large.raw";
	public static final double WORLD_WIDTH = 131.2;
	public static final double WORLD_HEIGHT = 41;
	public static final int MAP_WIDTH  = 1600; // 131.2m
	public static final int MAP_HEIGHT = 500;  // 41m
	public static final double MPP = 0.082; // meters per map pixel
	public static final int PATH_CHECK_INTERVAL = 1; // must be less than minimum obstacle width
	public static final int POINT_BUFFER_ZONE = 5;
	public static final int PATH_BUFFER_ZONE = 4;
	
	private BufferedImage img;
	private int scaledimwidth, scaledimheight;
	
	private double realdestpts[][]; // destination points specified in meter offsets from robot starting location
	private int obstaclemap[][]; // contains values of 1 (no obstacle) or 0 (obstacle)
	private int mappts[][]; // [row,col] form, contains random points in PRM
	private int adjmatrix[][]; // represents paths between PRM points
	private int mapdestpts[][]; // destination points in map coordinates
	private int mapstartpts[][]; // initial robot starting points in map coordinates
	
	
	///////////////////////////////////////////////////////////////////
	// Methods
	///////////////////////////////////////////////////////////////////
	
	// constructor
	public ProbRoadMap(int numpts, double realdestpts[][]) {
		this.realdestpts = realdestpts;
		
		// System.out.println("Constructor..."); // DEBUG

		// convert real destinations to map destination points
		mapdestpts = new int[realdestpts.length][2];
		for(int i = 0; i < realdestpts.length; i++) {
			mapdestpts[i][0] = realDistToMapDist(realdestpts[i][0] + WORLD_WIDTH/2);
			mapdestpts[i][1] = realDistToMapDist(WORLD_HEIGHT/2 - realdestpts[i][1]);
		}
		
		/*
		// DEBUG - hard-code map destination points
		mapdestpts = new int[3][2];
		mapdestpts[0][0] = 712; // upper hallway, right of red robot
		mapdestpts[0][1] = 98;
		mapdestpts[1][0] = 842; // robotics lab, right of white robot
		mapdestpts[1][1] = 314;
		mapdestpts[2][0] = 800; // to right of mapdestpt 0
		mapdestpts[2][1] = 98;
		*/
		
		System.out.println("Map destination points:");
		Retriever.printPts(mapdestpts);
		
		try {
			// create a 2D binary array using the world map
			FileInputStream input = new FileInputStream(filename);
			byte buff[] = new byte[MAP_HEIGHT * MAP_WIDTH];
			input.read(buff);
			
			byte val = 0;
			int count = 0;
			obstaclemap = new int[MAP_WIDTH][MAP_HEIGHT];
			for(int y = 0; y < MAP_HEIGHT; y++) {
				for(int x = 0; x < MAP_WIDTH; x++) {
					val = buff[(y * MAP_WIDTH) + x];
					// invert intensities
					if(val == 0) val = 1;
					if(val < 0) val = 0;
					if(val > 1) val = 0;
					obstaclemap[x][y] = val;
					count += 1;
					// System.out.println("x: " + x + " y: " + y + " => " + val); // DEBUG
				}
			}
			// System.out.println("Create count: " + count); // DEBUG
			
			// create image
			img = new BufferedImage(MAP_WIDTH,MAP_HEIGHT,BufferedImage.TYPE_INT_ARGB);
			
	        // scrollpane
	        MapPanel mp = new MapPanel();
	        JScrollPane scrollpane = new JScrollPane(mp);
	        add(scrollpane);
	        
	        // generate points and edges	        
			boolean valid = false;
	        while(!valid) {
	        	System.out.println("Computing probabilistic road map ..."); // DEBUG
	        	genAllPoints(numpts);
	            genAllEdges();
	            valid = checkPaths();
	       }
	        
            draw();
			drawAllEdges();
            System.out.println(">> ALL EDGES"); // DEBUG
    		setScaleFactor(2.0);
    		setVisible(true);
    		pack();
            Retriever.pause(); // interactive
            reset();
	        
	       drawAllPoints();
		   drawDestPoints();
		   drawStartPoints();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	// convert from real meter distances to map pixel distances
	private int realDistToMapDist(double realdist) {
		return (int) (realdist / MPP); 
	}
	// convert from map pixel distances to real meter distances
	private float mapDistToRealDist(int mapdist) {
		return (float) (mapdist * MPP); 
	}
	
	
	///////////////////////////////////////////////////////////////////
	// Display Methods
	///////////////////////////////////////////////////////////////////
	
	// scale
    public void setScaleFactor(double scaleFactor) {
        scaledimwidth = (int)(MAP_WIDTH * scaleFactor);
        scaledimheight = (int)(MAP_HEIGHT * scaleFactor);
    }
	
    // clear points and paths on map
    public void reset() {
    	draw();
    }
    
	// display the map
	public void draw() {
		int val, rgbval;
		for(int y = 0; y < MAP_HEIGHT; y++) {
			for(int x = 0; x < MAP_WIDTH; x++) {
				val = obstaclemap[x][y]*255;
				rgbval = (0xff << 24) | (val << 16) | (val << 8) | val;
				img.setRGB(x,y,rgbval);
			}
		}
		repaint();
	}
	
	// draw all edges in adjmatrix
	public void drawAllEdges() {
		int cval = Color.BLUE.getRGB();
		for(int i = 0; i < mappts.length; i++) {
			for(int j = i; j < mappts.length; j++) {
				if(adjmatrix[i][j] > 0) {
					drawLine(mappts[i],mappts[j],cval);
				}
			}
		}
		repaint();
	}
	
	// draw all vertices in adjmatrix
	public void drawAllPoints() {
		int cval = Color.RED.getRGB();
		int x, y;
		for(int i = 0; i < mappts.length; i++) {
			x = mappts[i][0];
			y = mappts[i][1];
			img.setRGB(x,y,cval);
		}
		repaint();
	}
	
	// draw initial robot starting points
	public void drawStartPoints() {
		int cval = Color.YELLOW.getRGB();
		int x, y;
		for(int i = 0; i < mapstartpts.length; i++) {
			x = mapstartpts[i][0];
			y = mapstartpts[i][1];
			img.setRGB(x,y,cval);
		}
		repaint();
	}
	
	// draw destination points
	public void drawDestPoints() {
		int cval = Color.GREEN.getRGB();
		int x, y;
		for(int i = 0; i < mapdestpts.length; i++) {
			x = mapdestpts[i][0];
			y = mapdestpts[i][1];
			img.setRGB(x,y,cval);
		}
		repaint();
	}
	
	
	// draw a line
	private void drawLine(int startpt[], int endpt[], int cval) {
		int x, y;
		double oldy, oldx, curry, currx, endy, endx;
		double theta, totaldist, dist, deltay, deltax;
		
		currx = startpt[0]; curry = startpt[1];
		endx  = endpt[0]; endy  = endpt[1];
		totaldist = Math.sqrt( Math.pow(endx-currx, 2) + Math.pow(endy-curry, 2) );
		theta = Math.atan2(endy-curry, endx-currx);
		dist = 0.0;
		
		while(dist < totaldist) {
			// step along path
			oldy = curry; oldx = currx;
			deltay = Math.sin(theta);
			deltax = Math.cos(theta);  
			curry = curry + deltay;
			currx = currx + deltax;
			dist += Math.sqrt( Math.pow(currx-oldx, 2) + Math.pow(curry-oldy, 2) );
			y = (int)Math.round(curry); x = (int)Math.round(currx);
			
			if(dist < totaldist) {
				img.setRGB(x,y,cval);
			}
		}
	}
	
	
	// draw a path
	public void drawPath(Stack<Node> nodepath) {
		Stack<Node> tmpnodepath = new Stack<Node>(); // don't modify the original
		tmpnodepath.addAll(nodepath);
    	int cval = Color.GREEN.getRGB();
    	Node n = null;
		// System.out.println("Path:"); // DEBUG
    	while(!tmpnodepath.isEmpty()) {
    		n = tmpnodepath.pop();
    		// System.out.print(n + " (gscore: " + n.gscore + ")  ->  "); // DEBUG
    		if(!tmpnodepath.isEmpty()) {
    			drawLine(mappts[n.index],mappts[tmpnodepath.peek().index],cval);
    		}
    	}
    	// System.out.println(); // DEBUG
    	repaint();
	}
	
	
	///////////////////////////////////////////////////////////////////
	// Map Generation Methods
	///////////////////////////////////////////////////////////////////
	
	// add a new map point - don't do this often
	public int addPoint(float realx, float realy) {
		int x = realDistToMapDist(realx + WORLD_WIDTH/2);
		int y = realDistToMapDist(WORLD_HEIGHT/2 - realy);
		int numpts = mappts.length;
		int tmppts[][] = new int[numpts+1][2];
		
		for(int i = 0; i < numpts; i++) {
			tmppts[i][0] = mappts[i][0];
			tmppts[i][1] = mappts[i][1];
		}
		tmppts[numpts][0] = x;
		tmppts[numpts][1] = y;
		mappts = tmppts; // reset
		
		// make room in the adjmatrix
		int tmpmatrix[][] = new int[numpts+1][numpts+1];
		for(int i = 0; i < numpts; i++) {
			for(int j = 0; j < numpts; j++) {
				tmpmatrix[i][j] = adjmatrix[i][j];
			}
		}
		adjmatrix = tmpmatrix;
		
		// generate new edges
		/*
		for(int i = 0; i < numpts-1; i++) {
			genEdge(numpts,i);
		}
		*/
		
		return numpts; // index of new point
	}
	
	// update the obstacle map with a new obstacle
	public void setVal(float realx, float realy) {
		int x = realDistToMapDist(realx + WORLD_WIDTH/2);
		int y = realDistToMapDist(WORLD_HEIGHT/2 - realy);
		obstaclemap[x][y] = 0;
		// System.out.println(">> PRM SETVAL [" + x + ", " + y + "]"); // DEBUG 
	}
	
	// determine if at least one path exists between all initial robot starting locations
	// and all destination points
	private boolean checkPaths() {
		System.out.println("Checking necessary paths ..."); // DEBUG
		boolean valid = true;
		int startindex, endindex;
		int numchecks = mapstartpts.length + mapdestpts.length;
		Node lastnode = null;
		
		// we know that all initial robot starting locations
		// and all destination points appear first in mappts
		for(int i = 1; i < numchecks && valid; i++) {
			startindex = i-1; endindex = i;
			System.out.print("Checking path " + startindex + " => " + endindex + " ... "); // DEBUG
			
			lastnode = planPath(i-1,i);
			valid = (lastnode != null);
			
			// DEBUG
			if(valid) {
				System.out.println("valid");
				drawPath(createPath(lastnode));
			} else {
				System.out.println("invalid");
			}
		}
		return valid;
	}
	
	// generate random points
	// numpts - number of random points to generate
	private void genAllPoints(int numpts) {
		int numstartpts = RobotControl.all_initial_pos.length;
		int numdestpts = mapdestpts.length;
		mapstartpts = new int[numstartpts][2];
		mappts = new int[numstartpts + numdestpts + numpts][2];
		Random rand = new Random();
		int x,y,count = 0;
		double realx, realy;

		// add initial robot starting points
		for(int i = 0; i < numstartpts; i++) {
			// convert real world offsets from world origin to map coordinates
			realx = RobotControl.all_initial_pos[i][0];
			realy = RobotControl.all_initial_pos[i][1];
			x = realDistToMapDist(realx + WORLD_WIDTH/2);
			y = realDistToMapDist(WORLD_HEIGHT/2 - realy);

			System.out.println("startingpt: [" + realx + "," + realy + "] => [" + x + "," + y + "]"); // DEBUG
			
			mapstartpts[count][0] = x;
			mapstartpts[count][1] = y;
			mappts[count][0] = x;
			mappts[count][1] = y;
			count += 1;
		}
		// add destination points
		for(int i = 0; i < numdestpts; i++) {
			mappts[count][0] = mapdestpts[i][0];
			mappts[count][1] = mapdestpts[i][1];
			count += 1;
		}
		// generate random points
		while(count < numdestpts + numpts) {
			y = rand.nextInt(MAP_HEIGHT);
			x = rand.nextInt(MAP_WIDTH);
			
			// determine if point is valid (no obstacle in buffer zone)
			boolean valid = true;
			int minx = Math.max(x-POINT_BUFFER_ZONE, 0);
			int maxx = Math.min(x+POINT_BUFFER_ZONE, MAP_WIDTH-1);
			int miny = Math.max(y-POINT_BUFFER_ZONE, 0);
			int maxy = Math.min(y+POINT_BUFFER_ZONE, MAP_HEIGHT-1);
			// System.out.println(">> minx: " + minx + " maxx: " + maxx + " miny: " + miny + " maxy: " + maxy); // DEBUG
			
			for(int i = minx; i <= maxx && valid; i++) {
				for(int j = miny; j <= maxy && valid; j++) {
					valid = (obstaclemap[i][j] != 0);
				}
			}
			
			if(valid) {
				mappts[count][0] = x;
				mappts[count][1] = y;
				count += 1;
			}
		}
	}
	
	// generate an edge
	private void genEdge(int startindex, int endindex) {
		int x, y;
		double oldy, oldx, curry, currx, endy, endx;
		double theta, totaldist, dist, deltay, deltax;
		boolean cont;
		
		// determine if there are obstacles along the path
		currx = mappts[startindex][0]; curry = mappts[startindex][1];
		endx  = mappts[endindex][0];   endy  = mappts[endindex][1];
		totaldist = Math.sqrt( Math.pow(endx-currx, 2) + Math.pow(endy-curry, 2) );
		theta = Math.atan2(endy-curry, endx-currx);
		dist = 0.0;
		
		cont = true;
		while(dist < totaldist && cont) {
			// step along path
			oldy = curry; oldx = currx;
			deltay = Math.sin(theta) * PATH_CHECK_INTERVAL;
			deltax = Math.cos(theta) * PATH_CHECK_INTERVAL;  
			curry = curry + deltay;
			currx = currx + deltax;
			dist += Math.sqrt( Math.pow(currx-oldx, 2) + Math.pow(curry-oldy, 2) );
			y = (int)Math.round(curry); x = (int)Math.round(currx);
			
			/*
			// determine if point is valid (no obstacle in buffer zone)
			int minx = Math.max(x-PATH_BUFFER_ZONE, 0);
			int maxx = Math.min(x+PATH_BUFFER_ZONE, MAP_WIDTH-1);
			int miny = Math.max(y-PATH_BUFFER_ZONE, 0);
			int maxy = Math.min(y+PATH_BUFFER_ZONE, MAP_HEIGHT-1);
			// System.out.println(">> minx: " + minx + " maxx: " + maxx + " miny: " + miny + " maxy: " + maxy); // DEBUG
			
			for(int k = minx; k <= maxx && cont; k++) {
				for(int l = miny; l <= maxy && cont; l++) {
					cont = (obstaclemap[k][l] > 0);
				}
			}
			*/
			
			// left
			float ldeltay, ldeltax, ltheta = (float) (theta + Math.toRadians(90));
			int ly, lx;
			for(int l = 0; l < PATH_BUFFER_ZONE && cont; l++) {
				ldeltay = (float) Math.sin(ltheta) * l;  ldeltax = (float) Math.cos(ltheta) * l;
				ly = (int)Math.round(ldeltay+y); 		 lx = (int)Math.round(ldeltax+x);
				// System.out.println("theta: " + Math.toDegrees(theta) + " ltheta: " + Math.toDegrees(ltheta) + 
				//		           " ldeltay: " + ldeltay + " ldeltax: " + ldeltax + " y: " + y + " x: " + x + " ly:" + ly + " lx: " + lx); // DEBUG
				cont = (obstaclemap[lx][ly] > 0);
			}
			
			// right
			float rdeltay, rdeltax, rtheta = (float) (theta - Math.toRadians(90));
			int ry, rx;
			for(int r = 1; r < PATH_BUFFER_ZONE && cont; r++) {
				rdeltay = (float) Math.sin(rtheta) * r;  rdeltax = (float) Math.cos(rtheta) * r;
				ry = (int)Math.round(rdeltay+y); 		 rx = (int)Math.round(rdeltax+x);
				cont = (obstaclemap[rx][ry] > 0);
			}
			
			
		}
		// add valid points to adjacency matrix
		if(cont) {
			adjmatrix[startindex][endindex] = 1;
			adjmatrix[endindex][startindex] = 1;
		}
	}
	
	// generate edges between points
	public void genAllEdges() {
		// determine if there is a path between every pair of points
		int numpts = mappts.length;
		adjmatrix = new int[numpts][numpts];
		for(int i = 0; i < numpts; i++) {
			for(int j = i; j < numpts; j++) {
				if(i != j) { // if not same point
					genEdge(i,j);
				}
			}
		}
	}
	
	/*
	// generate edges between points
	private void genAllEdges() {
		// determine if there is a path between every pair of points
		int x, y;
		double oldy, oldx, curry, currx, endy, endx;
		double theta, totaldist, dist, deltay, deltax;
		boolean cont;
		int numpts = mappts.length;
		adjmatrix = new int[numpts][numpts];
		for(int i = 0; i < numpts; i++) {
			for(int j = i; j < numpts; j++) {
				if(i != j) { // if not same point
					// determine if there are obstacles along the path
					currx = mappts[i][0]; curry = mappts[i][1];
					endx  = mappts[j][0]; endy  = mappts[j][1];
					totaldist = Math.sqrt( Math.pow(endx-currx, 2) + Math.pow(endy-curry, 2) );
					theta = Math.atan2(endy-curry, endx-currx);
					dist = 0.0;
					
					cont = true;
					while(dist < totaldist && cont) {
						// step along path
						oldy = curry; oldx = currx;
						deltay = Math.sin(theta) * PATH_CHECK_INTERVAL;
						deltax = Math.cos(theta) * PATH_CHECK_INTERVAL;  
						curry = curry + deltay;
						currx = currx + deltax;
						dist += Math.sqrt( Math.pow(currx-oldx, 2) + Math.pow(curry-oldy, 2) );
						y = (int)Math.round(curry); x = (int)Math.round(currx);
						cont = obstaclemap[x][y] > 0;
						
					}
					// add valid points to adjacency matrix
					if(cont) {
						adjmatrix[i][j] = 1;
						adjmatrix[j][i] = 1;
					}
				}
			}
		}
	}
	*/
	
	
	///////////////////////////////////////////////////////////////////
	// Path Planning Methods
	///////////////////////////////////////////////////////////////////
	
	// plan path using A* search and return the last node in the path if a path exists
	// indexes specify points in mappts
	// algorithm based on: http://en.wikipedia.org/wiki/A*_search_algorithm
	public Node planPath(int startindex, int destindex) {
		int endx, endy;
		int numpts = mappts.length;
		double fscore, tmpgscore, hscore;
		boolean tmpbetter;
		Node xnode, ynode;
		
		endx = mappts[destindex][0]; endy = mappts[destindex][1];
		NodeComparator comparator = new NodeComparator();
		
		// keep sets in sorted order
		List<Node> openset = new ArrayList<Node>();
		List<Node> closedset = new ArrayList<Node>();
		
		fscore = Math.sqrt( Math.pow(endx-mappts[startindex][0], 2) 
				          + Math.pow(endy-mappts[startindex][1], 2) );
		xnode = new Node(startindex,fscore);
		openset.add(xnode);
		
		while(!openset.isEmpty()) {
			// determine node in openset with lowest fscore (straight-line distance to dest)
			xnode = openset.get(0);
			// System.out.println(">> xnode: " + xnode.index); // DEBUG
				
			if(xnode.index == destindex) {
				return xnode;
			}
			
			// System.out.println("Try to remove from openset: " + xnode + " (" + xnode.fscore + ") from: " + openset); // DEBUG
			if(!openset.remove(xnode)) {
				System.err.println("Could not remove: " + xnode);
				System.exit(1);
			}
			// System.out.println("Try to add to closedset: " + xnode + " to: " + closedset); // DEBUG
			closedset.add(xnode);
			
			// iterate over xnode neighbors
			for(int y = 0; y < numpts; y++) {
				ynode = new Node(y,0.0);
				// System.out.println(">> >> ynode: " + ynode.index + ", " + 
				//		(adjmatrix[xnode.index][ynode.index] > 0) + " && " + !closedset.contains(ynode)); // DEBUG
				if(adjmatrix[xnode.index][ynode.index] > 0 && !closedset.contains(ynode)) { // if unchecked neighbor
					
					// System.out.println(">> >> checking"); // DEBUG
					
					// attempt to calculate distance from start along optimal path
					tmpgscore = xnode.gscore + 
					         Math.sqrt( Math.pow(mappts[ynode.index][0]-mappts[xnode.index][0], 2) 
									  + Math.pow(mappts[ynode.index][1]-mappts[xnode.index][1], 2) );
					hscore = Math.sqrt( Math.pow(endx-mappts[ynode.index][0], 2) 
					                  + Math.pow(endy-mappts[ynode.index][1], 2) );
					/*
					System.out.println(">> >> tmpgscore: " + tmpgscore); // DEBUG
					System.out.println(xnode.index); // DEBUG
					System.out.println(pts[xnode.index][0]); // DEBUG
					System.out.println(pts[xnode.index][1]); // DEBUG
					System.out.println(ynode.index); // DEBUG
					System.out.println(pts[ynode.index][0]); // DEBUG
					System.out.println(pts[ynode.index][1]); // DEBUG
					*/
					
					tmpbetter = false;
					if(!openset.contains(ynode)) {
						// System.out.println("Try to add to openset: " + ynode + " to: " + openset); // DEBUG
						openset.add(ynode);
						tmpbetter = true;
					} else if(tmpgscore < ynode.gscore) {
						tmpbetter = true;
					}
					
					// if best found so far
					if(tmpbetter) {
						// since we modified the node, we need to resort the openset
						ynode.prev = xnode;
						ynode.gscore = tmpgscore;
						ynode.fscore = ynode.gscore + hscore;
						Collections.sort(openset,comparator);
					}
				}
			}	
		}
		// if we made it this far without returning, a path cannot be formed
		return null;
	}
	
	// create a path in stack form by recursing from the last path node to the first
	public Stack<Node> createPath(Node lastnode) {
    	Stack<Node> nodepath = new Stack<Node>();
    	Node prevnode = null;
    	
    	// construct the stack
    	nodepath.add(lastnode);
    	if(lastnode == null) { System.out.println("!! LASTNODENULL"); } // DEBUG
    	while((prevnode = lastnode.prev) != null) { // TODO - NullPointerException
    		nodepath.add(prevnode);
    		lastnode = prevnode;
    	}

    	// determine the real-world coordinate offset values for each node
    	Node node = null;
    	Iterator<Node> iter = nodepath.iterator();
    	while(iter.hasNext()) {
    		node = iter.next();
    		node.mapx = mappts[node.index][0];
    		node.mapy = mappts[node.index][1];
    		node.realx = mapDistToRealDist(node.mapx - MAP_WIDTH/2);
    		node.realy = mapDistToRealDist(MAP_HEIGHT/2 - node.mapy);
    	}
		return nodepath;
	}
	
	
	///////////////////////////////////////////////////////////////////
	// GUI Classes
	///////////////////////////////////////////////////////////////////
	
	// GUI component for showing the map
	class MapPanel extends JPanel {
		private static final long serialVersionUID = 3524455568145773574L;
		
		// image-scaling idea taken from: 
    	// http://today.java.net/pub/a/today/2007/04/03/perils-of-image-getscaledinstance.html
        protected void paintComponent(Graphics g) {
            // g.drawImage(theMap,0,0,null); // old way
            Graphics2D g2 = (Graphics2D)g;
            g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                                RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR);
            g2.drawImage(img, 0, 0, scaledimwidth, scaledimheight, null);
        }
        public Dimension getPreferredSize() {
        	// System.out.println("scaledimwidth: " + scaledimwidth + " scaledimheight: " + scaledimheight); // DEBUG
            return new Dimension(scaledimwidth ,scaledimheight);
        }
    }
}



