package frc.robot.hijackablerotation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLock implements RotationSource {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static PIDController rotationPID = createPIDController();

    private static PIDController createPIDController() {
        PIDController pid = new PIDController(.01, .01, .001); // original: .01, .02, .001
        pid.setTolerance(1); // allowable angle error
        pid.enableContinuousInput(0.5, 360); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
        pid.setSetpoint(0); // 0 = apriltag angle
        return pid;
    }

    @Override
    public double getR() {
        return rotationPID.calculate(table.getEntry("tx").getDouble(0));
    }
}