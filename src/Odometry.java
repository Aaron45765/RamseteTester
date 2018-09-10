
/** Wrapper class for robot pose estimation, giving the robot's x-y position and heading on the field
* The positive X-axis is defined as the robot going straight forward towards the center of the field.
* The positive Y-axis is defined as the robot moving left from its starting point against the back wall.
* Theta is within the range [-2pi, 2pi] and is defined to be counter-clockwise positive, following the Unit Circle.
 */
public class Odometry {
    private double x, y, theta;

    public Odometry(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta % (Math.PI * 2.0);
    }

    public String toString(){
        return "X Position: " + x + " Y Position: " + y + " Heading: " + theta;
    }

}
