package frc.robot.utility;

import java.util.ArrayList;

public class AprilTag {
    private double id;
    private double height;
    private double headingNeeded;
    private double distanceNeeded;

    public AprilTag(double id, double height, double headingNeeded, double distanceNeeded) {
        this.id = id;
        this.height = height;
        this.headingNeeded = headingNeeded;
        this.distanceNeeded = distanceNeeded;
    }

    public double getHeight() {
        return height;
    }

    public double getExpectedHeading() {
        return headingNeeded;
    }

    public double getDistance() {
        return distanceNeeded;
    }

    public static ArrayList<AprilTag> LoadAprilTagList() {
        ArrayList<AprilTag> list = new ArrayList<AprilTag>();
        list.add(new AprilTag(1, 0, 0, 0)); // 
        list.add(new AprilTag(2, 0, 0, 0)); // 
        list.add(new AprilTag(3, 0, 0, 0)); // 
        list.add(new AprilTag(4, 0, 0, 0)); // 
        list.add(new AprilTag(5, 0, 0, 0)); // 
        list.add(new AprilTag(6, 0, 0, 0)); // 
        list.add(new AprilTag(7, 0, 0, 0)); // 
        list.add(new AprilTag(8, 0, 0, 0)); // 
        list.add(new AprilTag(9, 0, 0, 0)); // 
        list.add(new AprilTag(10, 50.5, 15, 70)); // Red
        list.add(new AprilTag(11, 50.5, 135, 70)); // Red
        list.add(new AprilTag(12, 50.5, -105, 70)); // Red
        list.add(new AprilTag(13, 0, 0, 0)); // 
        list.add(new AprilTag(14, 51.5, 15, 70)); // Blue
        list.add(new AprilTag(15, 51.5, 135, 70)); // Blue
        list.add(new AprilTag(16, 51.5, -105, 70)); // Blue
        return list;
    }
}
