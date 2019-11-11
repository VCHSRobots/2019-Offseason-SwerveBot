package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import org.frcteam2910.c2019.RobotMap;
//import org.frcteam2910.c2019.commands.HolonomicDriveCommand;

import frc.robot.RobotMap;
import frc.robot.drivers.SwerveModule4415Mk1;
import frc.robot.subsystems.Superstructure;
import frc.robot.Robot;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import java.util.Optional;

public class DrivetrainSubsystem extends SwerveDrivetrain {
    /* TODO: update dimensions */
    private static final double TRACKWIDTH = 19.5;
    private static final double WHEELBASE = 23.5;

    private static final double MAX_VELOCITY = 12.0 * 12.0;

    /* TODO: update constraints */
    public static final ITrajectoryConstraint[] CONSTRAINTS = {
            new MaxVelocityConstraint(MAX_VELOCITY),
            new MaxAccelerationConstraint(13.0 * 12.0),
            new CentripetalAccelerationConstraint(25.0 * 12.0)
    };

    private static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-154.3);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-329.0);
    private static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-218.10);
    private static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-268.9);

    
    private static final double FRONT_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-170.2152486947372);
    private static final double FRONT_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-43.55619048306742);
    private static final double BACK_LEFT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-237.47063008637048);
    private static final double BACK_RIGHT_ANGLE_OFFSET_PRACTICE = Math.toRadians(-336.70093128378477);
    /*
    private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)
    );

    private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    */
    /**TODO: update PID constants */
    private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
    private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
    private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
            new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0)
    );

    private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);
    /* */

    private static final DrivetrainSubsystem instance = new DrivetrainSubsystem();

    private SwerveModule4415Mk1[] swerveModules;

    private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            FOLLOWER_TRANSLATION_CONSTANTS,
            FOLLOWER_ROTATION_CONSTANTS,
            FOLLOWER_FEEDFORWARD_CONSTANTS
    );

    private PidController snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
    private double snapRotation = Double.NaN;

    private double lastTimestamp = 0;

    private final Object lock = new Object();
    private HolonomicDriveSignal signal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);
    private Trajectory.Segment segment = null;

    private DrivetrainSubsystem() {
        double frontLeftAngleOffset = FRONT_LEFT_ANGLE_OFFSET_COMPETITION;
        double frontRightAngleOffset = FRONT_RIGHT_ANGLE_OFFSET_COMPETITION;
        double backLeftAngleOffset = BACK_LEFT_ANGLE_OFFSET_COMPETITION;
        double backRightAngleOffset = BACK_RIGHT_ANGLE_OFFSET_COMPETITION;
       // if (Superstructure.getInstance().isPracticeBot()) {
       //     frontLeftAngleOffset = FRONT_LEFT_ANGLE_OFFSET_PRACTICE;
       //     frontRightAngleOffset = FRONT_RIGHT_ANGLE_OFFSET_PRACTICE;
       //     backLeftAngleOffset = BACK_LEFT_ANGLE_OFFSET_PRACTICE;
       //     backRightAngleOffset = BACK_RIGHT_ANGLE_OFFSET_PRACTICE;
       // }

       // SwerveModule4415Mk1(Vector2 modulePosition, double angleOffset,
         //                  TalonSRX angleMotor, CANSparkMax driveMotor, AnalogInput angleEncoder);

        //SwerveModule4415Mk1 frontLeftModule1 = new SwerveModule4415Mk1(new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), frontLeftAngleOffset, new TalonSRX(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR), driveMotor, angleEncoder);
        
        SwerveModule4415Mk1 frontLeftModule = new SwerveModule4415Mk1(
                new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontLeftAngleOffset,
                //new Spark(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new TalonSRX(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        frontLeftModule.setName("Front Left");

        SwerveModule4415Mk1 frontRightModule = new SwerveModule4415Mk1(
                new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                frontRightAngleOffset,
                new TalonSRX(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        frontRightModule.setName("Front Right");

        SwerveModule4415Mk1 backLeftModule = new SwerveModule4415Mk1(
                new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backLeftAngleOffset,
                new TalonSRX(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        backLeftModule.setName("Back Left");

        SwerveModule4415Mk1 backRightModule = new SwerveModule4415Mk1(
                new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                backRightAngleOffset,
                new TalonSRX(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)
        );
        backRightModule.setName("Back Right");

        swerveModules = new SwerveModule4415Mk1[]{
                frontLeftModule,
                frontRightModule,
                backLeftModule,
                backRightModule,
        };

        snapRotationController.setInputRange(0.0, 2.0 * Math.PI);
        snapRotationController.setContinuous(true);
        snapRotationController.setOutputRange(-0.5, 0.5);
    }

    public void setSnapRotation(double snapRotation) {
        synchronized (lock) {
            this.snapRotation = snapRotation;
        }
    }

    public void stopSnap() {
        synchronized (lock) {
            this.snapRotation = Double.NaN;
        }
    }

    @Override
    public void holonomicDrive(Vector2 translation, double rotation, boolean fieldOriented) {
        synchronized (lock) {
            this.signal = new HolonomicDriveSignal(translation, rotation, fieldOriented);
        }
    }

    @Override
    public synchronized void updateKinematics(double timestamp) {
        super.updateKinematics(timestamp);

        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;

        double localSnapRotation;
        synchronized (lock) {
            localSnapRotation = snapRotation;
        }
        RigidTransform2 currentPose = new RigidTransform2(
                getKinematicPosition(),
                getGyroscope().getAngle()
        );

        Optional<HolonomicDriveSignal> optSignal = follower.update(currentPose, getKinematicVelocity(),
                getGyroscope().getRate(), timestamp, dt);
        HolonomicDriveSignal localSignal;

        if (optSignal.isPresent()) {
            localSignal = optSignal.get();

            synchronized (lock) {
                segment = follower.getLastSegment();
            }
        } else {
            synchronized (lock) {
                localSignal = this.signal;
            }
        }

        if (Math.abs(localSignal.getRotation()) < 0.1 && Double.isFinite(localSnapRotation)) {
            snapRotationController.setSetpoint(localSnapRotation);

            localSignal = new HolonomicDriveSignal(localSignal.getTranslation(),
                    snapRotationController.calculate(getGyroscope().getAngle().toRadians(), dt),
                    localSignal.isFieldOriented());
        } else {
            synchronized (lock) {
                snapRotation = Double.NaN;
            }
        }

        super.holonomicDrive(localSignal.getTranslation(), localSignal.getRotation(), localSignal.isFieldOriented());
    }

    @Override
    public void outputToSmartDashboard() {
        super.outputToSmartDashboard();

        HolonomicDriveSignal localSignal;
        Trajectory.Segment localSegment;
        synchronized (lock) {
            localSignal = signal;
            localSegment = segment;
        }

        SmartDashboard.putNumber("Drivetrain Follower Forwards", localSignal.getTranslation().x);
        SmartDashboard.putNumber("Drivetrain Follower Strafe", localSignal.getTranslation().y);
        SmartDashboard.putNumber("Drivetrain Follower Rotation", localSignal.getRotation());
        SmartDashboard.putBoolean("Drivetrain Follower Field Oriented", localSignal.isFieldOriented());

        if (follower.getCurrentTrajectory().isPresent() && localSegment != null) {
            SmartDashboard.putNumber("Drivetrain Follower Target Angle", localSegment.rotation.toDegrees());

            Vector2 position = getKinematicPosition();

            SmartDashboard.putNumber("Drivetrain Follower X Error", localSegment.translation.x - position.x);
            SmartDashboard.putNumber("Drivetrain Follower Y Error", localSegment.translation.y - position.y);
            SmartDashboard.putNumber("Drivetrain Follower Angle Error", localSegment.rotation.toDegrees() - getGyroscope().getAngle().toDegrees());
        }

        for (SwerveModule4415Mk1 module : swerveModules) {
            SmartDashboard.putNumber(String.format("%s Module Drive Current Draw", module.getName()), module.getDriveCurrent());
        }
    }

    public static DrivetrainSubsystem getInstance() {
        return instance;
    }

    @Override
    public SwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    @Override
    public Gyroscope getGyroscope() {
        return Superstructure.getInstance().getGyroscope();
    }

    @Override
    public double getMaximumVelocity() {
        return 0;
    }

    @Override
    public double getMaximumAcceleration() {
        return 0;
    }

    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new HolonomicDriveCommand());
    }

    @Override
    public void stop() {
        super.stop();
        synchronized (lock) {
            snapRotation = Double.NaN;
        }
    }

    public HolonomicMotionProfiledTrajectoryFollower getFollower() {
        return follower;
    }

     public void DrivetrainTeleopPeriodic() {
        boolean ignoreScalars = Robot.getOi().primaryController.getLeftBumperButton().get();

        double forward = Robot.getOi().primaryController.getLeftYAxis().get(true);
        double strafe = Robot.getOi().primaryController.getLeftXAxis().get(true);
        double rotation = Robot.getOi().primaryController.getRightXAxis().get(true, ignoreScalars);

        boolean robotOriented = Robot.getOi().primaryController.getXButton().get();
        boolean reverseRobotOriented = Robot.getOi().primaryController.getYButton().get();

        Vector2 translation = new Vector2(forward, strafe);

        if (reverseRobotOriented) {
            robotOriented = true;
            translation = translation.rotateBy(Rotation2.fromDegrees(180.0));
        }

        DrivetrainSubsystem.getInstance().holonomicDrive(translation, rotation, !robotOriented);
     }
}