/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.frcteam2910.common.robot.drivers.Mk2SwerveModule;
//import org.frcteam2910.common.robot.subsystems.Subsystem;
import org.frcteam2910.common.robot.subsystems.SubsystemManager; 
import org.frcteam2910.common.robot.Utilities;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final double UPDATE_DT = 5e-3; // 5 ms

  private final SubsystemManager subsystemManager = new SubsystemManager(
          DrivetrainSubsystem.getInstance()
  );

  private static final OI oi = new OI(); 

  public Robot() {
  }

  public static OI getOi() {
    return oi;
  }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    subsystemManager.enableKinematicLoop(UPDATE_DT);
    Superstructure.getInstance().getGyroscope().getAngle();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    subsystemManager.outputToSmartDashboard();

    SmartDashboard.putNumber("Robot Angle", Superstructure.getInstance().getGyroscope().getAngle().toDegrees());
  }

  @Override
  public void disabledInit() {
    subsystemManager.stop();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
   
  }

  @Override
  public void teleopInit() {
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    DrivetrainSubsystem.getInstance().drivetrainTeleopPeriodic();
  }

  private TalonSRX test_motor;

  @Override
  public void testInit() {
    super.testInit();
    test_motor = new TalonSRX(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    super.testPeriodic();
    
    test_motor.set(
        ControlMode.PercentOutput, 
        Utilities.deadband(getOi().primaryController.getX(Hand.kRight)));

  }
}
