To compile and run this program, you must have the "javaclient.jar"
file in your path. This program was developed using Javaclient2-2.0.1. 

Run the Retriever program as follows:

java Retriever [-i] pts_file
java Retriever [-i] host port pts_file

The interactive "-i" flag is optional. Using it requires the user to
press enter at certain times during program execution. This allows the
user to view planned paths and other information before the robot 
continues execution.

Lines that begin with "# " in the pts_file will be ignored.