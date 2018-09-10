import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class Basic {
    
    public static void main(String[] args) {
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        Waypoint[] points = new Waypoint[] {
                new Waypoint(-4, -1, Pathfinder.d2r(-45)),
                new Waypoint(-2, -2, 0),
                new Waypoint(0, 0, 0)
        };

        Trajectory trajectory = Pathfinder.generate(points, config);
        
        // Do something with the new Trajectory...
    }
    
}
