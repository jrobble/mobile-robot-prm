import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;

/**
 * Simple poke-able grayscale image for displaying grid maps.
 *
 * @author zjb 3/09
 * @author jjr0192 4/09
 */
public class GridMap extends JFrame {
	private static final long serialVersionUID = 4962252855017064533L;
		
	// occupancy probability constants
	private static final float d1 = (float) (5.5/14.0 * 0.5);
	private static final float d2 = (float) (8.0/14.0 * 0.5);
	private static final float d3 = (float) (11.0/14.0 * 0.5);
	
	// gaussian constants
	private static final float sd = (float) Math.sqrt(0.05);
	
	// linear function constants
	private static final float rangethresh = 3.0f; // [5.0]
	private static final float m = (float) (-0.04/rangethresh); // slope
	private static final float b = 0.05f; // intercept
	
	private BufferedImage img;
    private int imwidth, imheight;
	private int scaledimwidth, scaledimheight;
    private double mpp;
    private float grid[][] = null;
    private boolean update[][] = null;
    
    /**
     * Construct a map image of given size and resolution.
     * Map's <b>center</b> will be at (0,0) and coordinates right-handed.
     * @param width map width in meters
     * @param height map height in meters
     * @param mpp Resolution in meters per pixel
     */
    public GridMap(int width, int height, double mpp) {
    	this.mpp = mpp;
        imwidth = (int)(width/mpp);
        imheight = (int)(height/mpp);
        System.out.println("imwidth: " + imwidth + " imheight: " + imheight + " mpp: " + mpp); // DEBUG
        
        // setup image
        img = new BufferedImage(imwidth,imheight, BufferedImage.TYPE_INT_ARGB);
        int midgray = (0xff << 24) | (128 << 16) | (128 << 8) | (128);
        for (int x = 0; x < imwidth; x++) {
            for (int y = 0; y < imheight; y++) {
                img.setRGB(x,y,midgray);
            }
        }
        
        // scrollpane
        MapPanel mp = new MapPanel();
        JScrollPane scrollpane = new JScrollPane(mp);
        add(scrollpane);
        
        // setup grid
        grid = new float[imwidth][imheight];
        update = new boolean[imwidth][imheight];
        for(int x = 0; x < imwidth; x++) {
        	for(int y = 0; y < imheight; y++) {
        		grid[x][y] = 0.5f;
        		update[x][y] = false;
        	}
        }
    }
    
    public float getValue(float dx, float dy) {
    	// make coords relative to the origin and discretize in cells
        int x = (int) Math.round(dx/mpp + grid.length/2);
        int y = (int) Math.round(grid[0].length/2 - dy/mpp); // flip
        return grid[x][y];
    }

    // update the grid
    // pass in actual read-world coords centered relative to the robot
    public void setValue(float dx, float dy, float value) {
    	// System.out.println("GridMap.setVal dx: " + dx + " dy: " + dy + " value: " + value); // DEBUG
        if (value < 0.0 || value > 1.0) {
        	System.err.println(">> Error: value: " + value + " invalid.");
            return;
        }
     
        // make coords relative to the origin and discretize in cells
        int x = (int) Math.round(dx/mpp + grid.length/2);
        int y = (int) Math.round(grid[0].length/2 - dy/mpp); // flip
        
        if(x > grid.length) {
        	System.err.println(">> Error: x: " + x + " > " + grid.length);
        	return;
        }
        if(y > grid[0].length) {
        	System.err.println(">> Error: y: " + y + " > " + grid[0].length);
        	return;
        }
        // System.out.println("imwidth/2: " + (imwidth/2) + " x/scale: " + (x/scale) +
        //                   "; imheight/2: " + (imheight/2) + " y/scale: " + (y/scale)); // DEBUG
        grid[x][y] = value;
        update[x][y] = true;
    }
    
    public void setScaleFactor(double scaleFactor) {
        scaledimwidth = (int)(imwidth * scaleFactor);
        scaledimheight = (int)(imheight * scaleFactor);
    }
    
    public void draw() {
		// set all pixels
    	int value = -1;
    	for(int x = 0; x <  grid.length; x++) {
    		for(int y = 0; y < grid[0].length; y++) {
    			if(update[x][y]) { // only update altered cells
	    			// convert to image coords
	    			value = (int) Math.round((1-grid[x][y]) * 255);
	    	        // System.out.println("imx: " + imx + " imy: " + imy); // DEBUG
					int rgbval = (0xff << 24) | (value << 16) | (value << 8) | value;
					if (x >= 0 && x < imwidth && y >= 0 && y < imheight) {
					    img.setRGB(x,y,rgbval);
					} else {
						System.err.println(">> Error: x: " + x + " or y: " + y + " invalid.");
					}
					update[x][y] = false;
    			}
    		}
    	}
    	repaint();
    }

    // P(m_{d,theta}(x(k))) | y(k),x(k)) = ...
    // calculate the obstacle probability for a cell at (rx,ry) and update the grid
    public float calcProbOccupancy(float rx, float ry, float range, float dist, float theta) {
    	float prob, deviation;
    	if(dist < range - d1) {
    		// System.out.println("case 1: dist < range - d1"); // DEBUG
    		prob = -calcProbDeviation(range,theta);
    	} else if(dist < range + d1) {
    		// System.out.println("case 2: dist < range + d1"); // DEBUG
    		deviation = calcProbDeviation(range,theta);
    		prob = -deviation + (deviation/d1) * (dist-range+d1);
    	} else if(dist < range + d2) {
    		// System.out.println("case 3: dist < range + d2"); // DEBUG
    		prob = calcProbDeviation(range,theta);
    	} else if(dist < range + d3) {
    		// System.out.println("case 4: dist < range + d3"); // DEBUG
    		deviation = calcProbDeviation(range,theta);
    		prob = deviation - (deviation/(d3-d2)) * (dist-range-d2);
    	} else {
    		// System.out.println("case 5: 0"); // DEBUG
    		prob = 0;
    	}
    	
    	prob += getValue(rx,ry);
    	if(prob < 0.0) {
    		prob = 0.0f;
    	}
    	if(prob > 1.0) {
    		prob = 1.0f;
    	}
    	
    	// update the grid
    	// System.out.println("probOccupancy: " + prob); // DEBUG
    	setValue(rx,ry,prob);
    	return prob;
    }
    
    // s(y(k),theta) = g(y(k)) * N(0,sd_theta)
    private float calcProbDeviation(float range, float theta) {
    	float linearpart = m * range + b;
    	float min = m * rangethresh + b;
    	if(linearpart < min) {
    		linearpart = min;
    	}
    	
    	float gaussconst = (float) (1/(sd * Math.sqrt(2*Math.PI))); 
    	float gausspow = (float) -(Math.pow(theta - 0,2)/(2 * Math.pow(sd,2)));
    	float gausspart = (float) (gaussconst * Math.exp(gausspow));
    	
    	float prob = linearpart * gausspart;
    	// System.out.println("probDeviation: " + prob); // DEBUG
    	return prob;
    }
    
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