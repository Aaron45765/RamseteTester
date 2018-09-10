import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;

public class Main {

    static RamseteFollower follower;
    static ArrayList<Odometry> robotPos = new ArrayList<>();
    static Waypoint[] waypoints = new Waypoint[]{
            new Waypoint(0 ,0,0),
            new Waypoint(4, 0, 0),
            new Waypoint(8, 4, Math.toDegrees(90.0))
    };
    public static void main(String[] args){
        Trajectory traj = Pathfinder.generate(waypoints, new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, .02, 4, 10, 60));
        follower = new RamseteFollower(2.166666, traj);
        robotPos.add(follower.getInitOdometry());
        int odometryIdx = 0;
        DriveSignal driveSignal;
        while(!follower.isFinished()){
            Odometry current = robotPos.get(odometryIdx);
            follower.setOdometry(current);
            driveSignal = follower.getNextDriveSignal();
            double w = (-driveSignal.getLeft() + driveSignal.getRight()) / 2.166666;
            double v = (driveSignal.getLeft() + driveSignal.getRight()) / 2;
            double heading = w * .02;
            double pos = v * .02;
            double x = pos * Math.cos(current.getTheta() + heading);
            double y = pos * Math.sin(current.getTheta() + heading);

            double newX = current.getX() + x;
            double newY = current.getY() + y;
            double newTheta = current.getTheta() + heading;

            System.out.println("New X: " + newX + " New Y: " + newY + " New Heading: " + Math.toDegrees(newTheta));
            robotPos.add(new Odometry(newX, newY, newTheta));
            odometryIdx++;
        }
//
//        System.out.println(System.getProperty("user.dir"));
//        File trajLoc = new File(System.getProperty("user.dir") + "\\out\\path.csv");
//        File robotLoc = new File(System.getProperty("user.dir") + "\\out\\robot.csv");
//        try {
//            PrintWriter pw = new PrintWriter(robotLoc);
//            StringBuilder writer = new StringBuilder();
//            writer.append("x,y,heading");
//            writer.append("\n");
//            for(Odometry odo: robotPos){
//                writer.append(odo.getX() + "," + odo.getY() + "," + odo.getTheta() + "\n");
//            }
//            pw.write(writer.toString());
//            pw.close();
//        }catch(Exception e){
//            System.out.println("ya done goofed");
//        }
//        Pathfinder.writeToCSV(trajLoc, traj);


    }
}
