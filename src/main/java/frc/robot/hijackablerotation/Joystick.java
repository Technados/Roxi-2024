package frc.robot.hijackablerotation;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Joystick implements RotationSource {
    @Override
    public double getR() {
        return -MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(),
                Constants.OIConstants.kDriveDeadband);
    }
}