/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {
// drive motor controler can id's
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 5;
  private static final int kRearRightChannel = 6;

  // shooter motoro controller can id's
  private static final int kLeftShooterID = 7;
  private static final int kRightShooterID = 8;

  // lift motor controler can id
  private static final int kLiftMotorID = 9;


  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private XboxController m_stick;

  private WPI_VictorSPX m_LeftShooter;
  private WPI_VictorSPX m_RightShooter;
  private WPI_VictorSPX m_liftMotor;

  @Override
  public void robotInit() {

    // create the talonSRX motor controller objects
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    // create the victorSPX motor controller objects
    WPI_VictorSPX m_LeftShooter = new WPI_VictorSPX(kLeftShooterID);
    WPI_VictorSPX m_RightShooter = new WPI_VictorSPX(kRightShooterID);
    WPI_VictorSPX m_liftMotor = new WPI_VictorSPX(kLiftMotorID);
 

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    //frontLeft.setInverted(true);
    //rearLeft.setInverted(true);
    m_LeftShooter.setInverted(true);

    //m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new XboxController(kJoystickChannel);

    m_LeftShooter.set(0);
    m_RightShooter.set(0);
    m_liftMotor.set(0);
  }

  @Override
  public void teleopPeriodic() {

    m_robotDrive.driveCartesian(m_stick.getX(Hand.kLeft),
                                m_stick.getY(Hand.kLeft) , 
                                m_stick.getX(Hand.kRight));
  }
  @Override
  public void testInit(){
    // Called at the begining of test mode

    // need to invert one of the shooter motors
    
    
  }

  @Override
  public void testPeriodic(){
    // Called every loop while in drivers station set to test mode
    m_LeftShooter.set(m_stick.getTriggerAxis(Hand.kLeft));
    m_RightShooter.set(m_stick.getTriggerAxis(Hand.kLeft));
    m_liftMotor.set(m_stick.getTriggerAxis(Hand.kLeft));
    
  }
}
