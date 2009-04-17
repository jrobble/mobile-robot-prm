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
import java.util.Comparator;
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

	private static final long serialVersionUID = 8130028453731235660L;
	
	// The robot is [0.50,0.37] m, so use a circular structuring element with a radius of 0.50 m
	// If MPP = 0.082, then structuring element should have a radius of 6.089 px => 7 px
	// If the c-space were calculated, there would be < 4 px of space in the robotics lab doorway.
	// It is very unlikely that two PRM random pts would be generated to provide a suitable path in.
	// Thus, consider using potential field motion for obstacle avoidance.
	
	// raw world map is a 1600*500 file containing byte values of either 0 or 255
	private static final String filename = "../3large.raw";
	private static final int MAP_WIDTH  = 1600; // 131.2m
	private static final int MAP_HEIGHT = 500;  // 41m
	private static final double MPP = 0.082; // meters per map pixel
	private static final int PATH_CHECK_INTERVAL = 2; // must be less than minimum obstacle width
	
	private BufferedImage img;
	private int scaledimwidth, scaledimheight;
	private int map[][]; // contains values of 0 or 1
	private int pts[][]; // [row,col]
	private int adjmatrix[][]; // represents paths between points
	private double realdestpts[][]; // destination points specified in meter offsets from robot starting location
	private int mapdestpts[][]; // destination points in map coordinates
	
	// constructor
	public ProbRoadMap(int numpts, double destpts[][]) {
		// System.out.println("Constructor..."); // DEBUG
		
		// convert robot real initial locations into map points
		// TODO
		
		// convert real destinations to map destination points
		/*
		realdestpts = destpts;
		mapdestpts = new int[realdestpts.length][2];
		for(int i = 0; i < realdestpts.length; i++) {
			// TODO - account for robot's initial starting location
			mapdestpts[i][0] = realDistToMapDist(realdestpts[i][0]);
			mapdestpts[i][1] = realDistToMapDist(realdestpts[i][1]);
		}
		*/
		
		// DEBUG - hard-code map destination points
		mapdestpts = new int[3][2];
		mapdestpts[0][0] = 712; // upper hallway, right of red robot
		mapdestpts[0][1] = 98;
		mapdestpts[1][0] = 842; // robotics lab, right of white robot
		mapdestpts[1][1] = 314;
		mapdestpts[2][0] = 800; // to right of mapdestpt 0
		mapdestpts[2][1] = 98;
		System.out.println("Map destination points:");
		Retriever.printPts(mapdestpts);
		
		try {
			// create a 2D binary array using the world map
			FileInputStream input = new FileInputStream(filename);
			byte buff[] = new byte[MAP_HEIGHT * MAP_WIDTH];
			input.read(buff);
			
			byte val = 0;
			int count = 0;
			map = new int[MAP_WIDTH][MAP_HEIGHT];
			for(int y = 0; y < MAP_HEIGHT; y++) {
				for(int x = 0; x < MAP_WIDTH; x++) {
					val = buff[(y * MAP_WIDTH) + x];
					// invert intensities
					if(val == 0) val = 1;
					if(val < 0) val = 0;
					if(val > 1) val = 0;
					map[x][y] = val;
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
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	// convert from real meter distances to map pixel distances
	private int realDistToMapDist(double realdist) {
		return (int) (realdist / MPP); 
	}
	
	// scale
    public void setScaleFactor(double scaleFactor) {
        scaledimwidth = (int)(MAP_WIDTH * scaleFactor);
        scaledimheight = (int)(MAP_HEIGHT * scaleFactor);
    }
	
	// display the map
	public void draw() {
		int val, rgbval;
		for(int y = 0; y < MAP_HEIGHT; y++) {
			for(int x = 0; x < MAP_WIDTH; x++) {
				val = map[x][y]*255;
				rgbval = (0xff << 24) | (val << 16) | (val << 8) | val;
				img.setRGB(x,y,rgbval);
			}
		}
		repaint();
	}
	
	// draw all edges in adjmatrix
	public void drawAllEdges() {
		int cval = Color.BLUE.getRGB();
		for(int i = 0; i < pts.length; i++) {
			for(int j = i; j < pts.length; j++) {
				if(adjmatrix[i][j] > 0) {
					drawLine(pts[i],pts[j],cval);
				}
			}
		}
		repaint();
	}
	
	// draw all vertices in adjmatrix
	public void drawAllPoints() {
		int cval = Color.RED.getRGB();
		int x, y;
		for(int i = 0; i < pts.length; i++) {
			x = pts[i][0];
			y = pts[i][1];
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
			img.setRGB(x,y,cval);
		}
	}
	
	
	// draw a path
	public void drawPath(Node lastnode) {
		System.out.println("Path:");
    	Stack<Node> nodepath = new Stack<Node>();
    	nodepath.add(lastnode);
    	Node prevnode = null;
    	while((prevnode = lastnode.prev) != null) {
    		nodepath.add(prevnode);
    		lastnode = prevnode;
    	}
    	int cval = Color.GREEN.getRGB();
    	Node n = null;
    	while(!nodepath.isEmpty()) {
    		n = nodepath.pop();
    		System.out.print(n + " (gscore: " + n.gscore + ")  ->  ");
    		if(!nodepath.isEmpty()) {
    			drawLine(pts[n.index],pts[nodepath.peek().index],cval);
    		}
    	}
    	System.out.println();
    	repaint();
	}
	
	// determine if a path exists between all starting locations
	// and all destination points
	private boolean checkPaths() {
		boolean valid = true;
		
		int startx, starty, endx, endy;
		Node lastnode = null;
		startx = realDistToMapDist(RobotControl.all_initial_pos[0][0]);
		starty = realDistToMapDist(RobotControl.all_initial_pos[0][1]);
		
		for(int i = 1; i < RobotControl.all_initial_pos.length; i++) {
			endx = realDistToMapDist(RobotControl.all_initial_pos[i][0]);
			endy = realDistToMapDist(RobotControl.all_initial_pos[i][1]);
			// lastnode = planPath();
			
		}
		
		return valid;
	}
	
	// generate random points
	// numpts - number of random points to generate
	private void genAllPoints(int numpts) {
		int numdestpts = mapdestpts.length;
		pts = new int[numdestpts + numpts][2];
		Random rand = new Random();
		int x,y,count = 0;

		// add initial robot location points
		
		
		// add destination points
		for(int i = 0; i < numdestpts; i++) {
			pts[i][0] = mapdestpts[i][0];
			pts[i][1] = mapdestpts[i][1];
		}
		count = numdestpts;
		// generate random points
		while(count < numdestpts + numpts) {
			y = rand.nextInt(MAP_HEIGHT);
			x = rand.nextInt(MAP_WIDTH);
			// determine if point is valid (not inside an obstacle)
			if(map[x][y] != 0) {
				pts[count][0] = x;
				pts[count][1] = y;
				count += 1;
			}
		}
	}
	
	// generate edges between points
	private void genAllEdges() {
		// determine if there is a path between every pair of points
		int x, y;
		double oldy, oldx, curry, currx, endy, endx;
		double theta, totaldist, dist, deltay, deltax;
		boolean cont;
		int numpts = pts.length;
		adjmatrix = new int[numpts][numpts];
		for(int i = 0; i < numpts; i++) {
			for(int j = i; j < numpts; j++) {
				if(i != j) { // if not same point
					// determine if there are obstacles along the path
					currx = pts[i][0]; curry = pts[i][1];
					endx  = pts[j][0]; endy  = pts[j][1];
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
						cont = map[x][y] > 0;
						
						/*
						if(cont) {
							// DEBUG
							int cval = Color.BLUE.getRGB();
							img.setRGB(x,y,cval);
							repaint();
						}
						*/
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
	
	// plan path using A* search
	// return the last node in the path if a path exists
	// algorithm based on: http://en.wikipedia.org/wiki/A*_search_algorithm
	public Node planPath(int startindex, int destindex) {
		int endx, endy;
		int numpts = pts.length;
		double fscore, tmpgscore, hscore;
		boolean tmpbetter;
		Node xnode, ynode;
		
		endx = pts[destindex][0]; endy = pts[destindex][1];
		NodeComparator comparator = new NodeComparator();
		
		// keep sets in sorted order
		List<Node> openset = new ArrayList<Node>();
		List<Node> closedset = new ArrayList<Node>();
		
		fscore = Math.sqrt( Math.pow(endx-pts[startindex][0], 2) 
				          + Math.pow(endy-pts[startindex][1], 2) );
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
					         Math.sqrt( Math.pow(pts[ynode.index][0]-pts[xnode.index][0], 2) 
									  + Math.pow(pts[ynode.index][1]-pts[xnode.index][1], 2) );
					hscore = Math.sqrt( Math.pow(endx-pts[ynode.index][0], 2) 
					                  + Math.pow(endy-pts[ynode.index][1], 2) );
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
	
	
	// simple node used for A* search
	class Node implements Comparable<Node> {
		public int index;
		public Node prev = null;
		public double fscore;
		public double gscore;
		
		public Node(int index, double fscore) {
			this.index = index;
			this.fscore = fscore;
			this.gscore = 0.0;
		}
		
		// this never seems to be called
		public boolean equals(Object other) {
			boolean retval = false;
			if(other != null) {
				if(other instanceof Node) {
					retval = (index == ((Node)other).index);
				}
			}
			return retval;
		}
		
		public int compareTo(Node other) {
			int retval = 0;
			if(index != other.index) {
				if(index < other.index) {
					retval = -1;
				} else if(index > other.index) {
					retval = 1;
				}
			}
			return retval;
		}
		
		public String toString() {
			return index + "";
		}
		
	}
	
	// simple comparator used to keep A* sets sorted
	// Returns a negative integer, zero, or a positive integer as 
	// the first argument is less than, equal to, or greater than the second.
	class NodeComparator implements Comparator<Node> {
		public int compare(Node node1, Node node2) {
			int retval = 0;
			if(node1.fscore != node2.fscore) {
				if(node1.fscore < node2.fscore) {
					retval = -1;
				} else if(node1.fscore > node2.fscore) {
					retval = 1;
				}
			}
			return retval;
		}
	}
	
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



