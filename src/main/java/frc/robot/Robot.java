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
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 5;
  private static final int kRearRightChannel = 6;

  

  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private XboxController m_stick;

  @Override
  public void robotInit() {

    
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

  
 

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    //frontLeft.setInverted(true);
    //rearLeft.setInverted(true);

    //m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_stick = new XboxController(kJoystickChannel);
  }

  @Override
  public void teleopPeriodic() {

    m_robotDrive.driveCartesian(m_stick.getX(Hand.kLeft),
                                m_stick.getY(Hand.kLeft) , 
                                m_stick.getX(Hand.kRight));
  }
}
