import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.SwerveModifier;

public class Swerve {

    public static void main(String[] args) {
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
        Waypoint[] points = new Waypoint[] {
                new Waypoint(-4, -1, Pathfinder.d2r(-45)),
                new Waypoint(-2, -2, 0),
                new Waypoint(0, 0, 0)
        };

        Trajectory trajectory = Pathfinder.generate(points, config);

        // Wheelbase Width = 0.5m, Wheelbase Depth = 0.6m, Swerve Mode = Default
        SwerveModifier modifier = new SwerveModifier(trajectory).modify(0.5, 0.6, SwerveModifier.Mode.SWERVE_DEFAULT);

        // Do something with the new Trajectories...
        Trajectory fl = modifier.getFrontLeftTrajectory();
        Trajectory fr = modifier.getFrontRightTrajectory();
        Trajectory bl = modifier.getBackLeftTrajectory();
        Trajectory br = modifier.getBackRightTrajectory();
    }

}
