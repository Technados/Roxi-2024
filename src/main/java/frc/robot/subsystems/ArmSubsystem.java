package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import pabeles.concurrency.IntOperatorTask.Max;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.Timer;

// import frc.lib.PIDGains;

public class ArmSubsystem extends SubsystemBase {

    /** Creates a new Arm motor system. */

    private CANSparkMax leftArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmCanId, MotorType.kBrushless);
    private CANSparkMax rightArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmCanId, MotorType.kBrushless);
    // private RelativeEncoder m_rightArmEncoder;
    // private RelativeEncoder m_leftArmEncoder;
    private SparkPIDController m_rightController;
    private SparkPIDController m_leftController;
    private double m_setpoint;

    private TrapezoidProfile m_profile;
    private Timer m_timer;
    private TrapezoidProfile.State m_startState;
    private TrapezoidProfile.State m_endState;

    private TrapezoidProfile.State m_targetState;
    private double m_feedforward;
    private double m_manualValue;

    private static ArmSubsystem instance;
    // private CANSparkMax intakeMotor = new
    // CANSparkMax(Constants.IntakeConstants.kIntakeMotorCanId,
    // MotorType.kBrushless);
    // Encoders
    private RelativeEncoder leftArmEncoder = leftArmMotor.getEncoder();
    private RelativeEncoder rightArmEncoder = rightArmMotor.getEncoder();

    private float speed = 0.25f;
    private double value;
    // private double m_armPositionAverage;
    // private double m_armVelocityAverage;

    public ArmSubsystem() {
        // create a new SPARK MAX for each arm motor and configure them
        leftArmMotor.restoreFactoryDefaults();
        rightArmMotor.restoreFactoryDefaults();

        // Resetting the encoder postion on robot startup
        leftArmEncoder.setPosition(0);
        rightArmEncoder.setPosition(0);

        rightArmMotor.setInverted(ArmConstants.kRightArmMotorInverted);
        leftArmMotor.setInverted(ArmConstants.kLeftArmMotorInverted);
        rightArmMotor.setInverted(ArmConstants.kRightArmMotorInverted);
        leftArmMotor.setInverted(ArmConstants.kLeftArmMotorInverted);
        // rightArmMotor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
        // leftArmMotor.setSmartCurrentLimit(Constants.ArmConstants.kArmMotorCurrentLimit);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        leftArmMotor.setIdleMode(IdleMode.kBrake);

        // rightArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // rightArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // leftArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // leftArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // rightArmMotor.setSoftLimit(SoftLimitDirection.kForward, (float)
        // Constants.ArmConstants.kSoftLimitForward);
        // leftArmMotor.setSoftLimit(SoftLimitDirection.kForward, (float)
        // Constants.ArmConstants.kSoftLimitForward);
        // rightArmMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)
        // Constants.ArmConstants.kSoftLimitReverse);
        // leftArmMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)
        // Constants.ArmConstants.kSoftLimitReverse);

        // set up the motor encoder including conversion factors to convert to radians
        // and radians per
        // second for position and velocity
        // m_rightArmEncoder =
        // rightArmMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        // m_leftArmEncoder =
        // leftArmMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        // m_rightArmEncoder.setPositionConversionFactor(Constants.ArmConstants.kArmPositionFactor);
        // m_leftArmEncoder.setPositionConversionFactor(Constants.ArmConstants.kArmPositionFactor);
        // m_rightArmEncoder.setVelocityConversionFactor(Constants.ArmConstants.kVelocityFactor);
        // m_leftArmEncoder.setVelocityConversionFactor(Constants.ArmConstants.kVelocityFactor);
        // rightArmEncoder.setPosition(0.0);
        // leftArmEncoder.setPosition(0.0);

        // m_armPositionAverage = ((m_rightArmEncoder.getPosition() +
        // m_leftArmEncoder.getPosition()) / 2);
        // m_armVelocityAverage = ((m_rightArmEncoder.getVelocity() +
        // m_leftArmEncoder.getVelocity()) / 2);

        // m_rightController = rightArmMotor.getPIDController();
        // m_leftController = leftArmMotor.getPIDController();
        // PIDGains.setSparkMaxGains(m_rightController,
        // Constants.ArmConstants.kArmPositionGains);
        // PIDGains.setSparkMaxGains(m_leftController,
        // Constants.ArmConstants.kArmPositionGains);

        rightArmMotor.burnFlash();
        leftArmMotor.burnFlash();

        // m_setpoint = Constants.ArmConstants.kHomePosition;

        // m_timer = new Timer();
        // m_timer.start();

        // updateMotionProfile();

        /**
         * Sets the target position and updates the motion profile if the target
         * position changed.
         *
         * @param _setpoint The new target position in radians.
         */
        // public void setTargetPosition(double _setpoint) {
        // if (_setpoint != m_setpoint) {
        // m_setpoint = _setpoint;
        // updateMotionProfile();
        // }
        // }

        /**
         * Update the motion profile variables based on the current setpoint and the
         * pre-configured motion
         * constraints.
         */
        // private void updateMotionProfile() {
        // m_startState = new TrapezoidProfile.State(m_armPositionAverage,
        // m_armVelocityAverage);
        // m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
        // m_profile = new
        // TrapezoidProfile(Constants.ArmConstants.kArmMotionConstraint);
        // m_timer.reset();
        // }

        /**
         * Drives the arm to a position using a trapezoidal motion profile. This
         * function is usually
         * wrapped in a {@code RunCommand} which runs it repeatedly while the command is
         * active.
         *
         * <p>
         * This function updates the motor position control loop using a setpoint from
         * the trapezoidal
         * motion profile. The target position is the last set position with
         * {@code setTargetPosition}.
         */
        // public void runAutomatic() {
        // double elapsedTime = m_timer.get();
        // if (m_profile.isFinished(elapsedTime)) {
        // m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
        // } else {
        // m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
        // }

        // m_feedforward = Constants.ArmConstants.kArmFeedforward.calculate(
        // m_armPositionAverage + Constants.ArmConstants.kArmZeroCosineOffset,
        // m_targetState.velocity);

        // m_rightController.setReference(
        // m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
        // m_leftController.setReference(
        // m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
        // }

        /**
         * Drives the arm using the provided power value (usually from a joystick). This
         * also adds in the
         * feedforward value which can help counteract gravity.
         *
         * @param _power The motor power to apply.
         */
        // public void runManual(double _power) {
        // // reset and zero out a bunch of automatic mode stuff so exiting manual mode
        // // happens cleanly and
        // // passively
        // m_setpoint = m_armPositionAverage;
        // updateMotionProfile();
        // // update the feedforward variable with the newly zero target velocity
        // m_feedforward = Constants.ArmConstants.kArmFeedforward.calculate(
        // m_armPositionAverage + Constants.ArmConstants.kArmZeroCosineOffset,
        // m_targetState.velocity);
        // // set the power of the motor
        // rightArmMotor.set(_power + (m_feedforward / 1.0));
        // leftArmMotor.set(_power + (m_feedforward / 1.0));
        // m_manualValue = _power; // this variable is only used for logging or
        // debugging if needed

        // }

        // @Override
        // public void periodic() { // This method will be called once per scheduler run
        // }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        int num = (int) (value * 100);
        String percent = String.valueOf(num);
        SmartDashboard.putString("Arm Motors Speed", percent + "%");
        SmartDashboard.putNumber("Left Arm Encoder Position", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Arm Encoder Position", getRightEncoderPosition());
        SmartDashboard.putNumber("Average Arm Encoder Postion", getAverageArmEncoderDistance());
    }

    // Returns an instance of this subsystem
    public static ArmSubsystem getInstance() {
        if (instance == null) {
            instance = new ArmSubsystem();
        }
        return instance;
    }

    // Resets the encoders position to 0.0
    public void resetEncoders() {
        leftArmEncoder.setPosition(0.0);
        rightArmEncoder.setPosition(0.0);
    }

    // Returns the left arm encoder position
    public double getLeftEncoderPosition() {
        return leftArmEncoder.getPosition();
    }

    // Returns the right arm encoder position
    public double getRightEncoderPosition() {
        return rightArmEncoder.getPosition();
    }

    // Returns the average encoder distance of left and right encoders
    public double getAverageArmEncoderDistance() {
        return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
    }

    // Returns true when the encoder right arm is at it's limit
    public boolean getRightTriggerLimit() {
        return ((leftArmEncoder.getPosition() > 65) ||
                (rightArmEncoder.getPosition() > 65));
    }

    // Returns true when the encoder left arm is at it's limit
    public boolean getLeftTriggerLimit() {
        return ((leftArmEncoder.getPosition() < 0) ||
                (rightArmEncoder.getPosition() < 0));
    }

    // Controls arm movement based on trigger inputs
    public void moveArm(double leftTrigger, double rightTrigger) {
        this.value = value;

        if (getLeftTriggerLimit()) {
            leftTrigger = 0;
        } else if (getRightTriggerLimit()) {
            rightTrigger = 0;
        }

        if (leftTrigger != 0) {
            value = leftTrigger;
        } else if (rightTrigger != 0) {
            value = rightTrigger;
        } else {
            value = 0;
        }

        rightArmMotor.set(value);
        leftArmMotor.set(value);
    }

    // Spins the arm motors forwards
    public void armForward() {
        leftArmMotor.set(-speed);
        rightArmMotor.set(-speed);
    }

    // Spins the arm motors in reverse
    public void armReverse() {
        leftArmMotor.set(speed);
        rightArmMotor.set(speed);
    }

    // Stops the arm motors
    public void stopArm() {
        leftArmMotor.set(0.0);
        rightArmMotor.set(0.0);
    }

    // Auto method to extend the arm out
    public boolean autoArmOut() {
        resetEncoders();
        while ((leftArmEncoder.getPosition() <= 65) || (rightArmEncoder.getPosition() <= 65)) {
            armForward();
        }
        stopArm();
        return true;
    }

    // Auto method to extend the arm in
    public boolean autoArmIn() {
        resetEncoders();
        while ((leftArmEncoder.getPosition() >= 0) || (rightArmEncoder.getPosition() >= 0)) {
            armReverse();
        }
        stopArm();
        return true;
    }

    // public boolean getArmLimitStatus() {
    // if ((leftArmEncoder.getPosition() > -1.5) || (rightArmEncoder.getPosition() >
    // -1.5)) {
    // return true;
    // } else if ((leftArmEncoder.getPosition() < 100) ||
    // (rightArmEncoder.getPosition() < 100)) {
    // return true;
    // } else {
    // return false;
    // }
    // }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
