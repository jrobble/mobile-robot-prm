/*
 * Node.java
 *
 *  Created on: Apr 16, 2009
 *      Author: jjr0192
 */

// simple node used for A* search
class Node implements Comparable<Node> {
	public int index; // corresponds to mappts
	public Node prev = null;
	
	public double fscore;
	public double gscore;
	
	public int mapx;
	public int mapy;
	
	public float realx;
	public float realy;
	
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