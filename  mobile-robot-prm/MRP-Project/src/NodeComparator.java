import java.util.Comparator;

/*
 * NodeComparator.java
 *
 *  Created on: Apr 16, 2009
 *      Author: jjr0192
 */

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
