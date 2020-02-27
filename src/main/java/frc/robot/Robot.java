/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {
// drive motor controller can id's
  private static final int kFrontLeftChannel = 5;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 6;
  private static final int kRearRightChannel = 2;

  // shooter motor controller can id's
  private static final int kLeftShooterID = 5;
  private static final int kRightShooterID = 7;

  // lift motor controller can id
  private static final int kLiftMotorID = 1;

  private static final int kPCMCanID = 15;
  private static final int kCollectorForwardPort = 0;
  private static final int kCollectorBackwardPort = 1;

  private static final int kDriverXBoxPort = 0;
  private static final int kOperXBoxPort = 1;

  private MecanumDrive m_robotDrive;

  private XboxController m_Driver;
  private XboxController m_Operator;

  private VictorSPX m_LeftShooter = new VictorSPX(kLeftShooterID);
  private VictorSPX m_RightShooter = new VictorSPX(kRightShooterID);
  private VictorSPX m_liftMotor = new VictorSPX(kLiftMotorID);

  private double motorSpeed;

  
  final Compressor m_compressor = new Compressor();
  final DoubleSolenoid m_CollectorArm  = new DoubleSolenoid(kPCMCanID,kCollectorForwardPort, kCollectorBackwardPort);
  
  
  @Override
  public void robotInit() {

    // create the talonSRX motor controller objects

    final WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    final WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    final WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    final WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    //create the victorSPX motor controller objects
    final VictorSPX m_LeftShooter = new VictorSPX(kLeftShooterID);
    final VictorSPX m_RightShooter = new VictorSPX(kRightShooterID);
    final VictorSPX m_liftMotor = new VictorSPX(kLiftMotorID);
 

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    //frontLeft.setInverted(true);
    //rearLeft.setInverted(true);
    m_LeftShooter.setInverted(true);

    
    m_compressor.enabled();


    //m_RightShooter.follow(m_LeftShooter);

    //m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_Driver = new XboxController(kDriverXBoxPort);
    m_Operator = new XboxController(kOperXBoxPort);

    m_LeftShooter.set(ControlMode.PercentOutput,0);
    m_RightShooter.set(ControlMode.PercentOutput,0);
    m_liftMotor.set(ControlMode.PercentOutput,0);

    System.out.println("Set motors to zero");
  }

  @Override
  public void teleopPeriodic() {

    m_robotDrive.driveCartesian(m_Driver.getX(Hand.kLeft),
                                m_Driver.getY(Hand.kLeft) , 
                                m_Driver.getX(Hand.kRight));

    motorSpeed = m_Operator.getTriggerAxis(Hand.kLeft) * -1;

    m_LeftShooter.set(ControlMode.PercentOutput,motorSpeed);
    //m_RightShooter.set(ControlMode.PercentOutput,motorSpeed);
    m_liftMotor.set(ControlMode.PercentOutput,motorSpeed);
                                
  }
  @Override
  public void testInit(){
    // Called at the begining of test mode

    // need to invert one of the shooter motors
    
    
  }

  @Override
  public void testPeriodic(){
    /* motorSpeed = m_Operator.getTriggerAxis(Hand.kLeft);
    System.out.println("Trigger =" + m_Operator);

    m_LeftShooter.set(ControlMode.PercentOutput,motorSpeed); // <-- Right should follow what the left does.
    m_RightShooter.set(ControlMode.PercentOutput, motorSpeed);
    m_liftMotor.set(ControlMode.PercentOutput,motorSpeed);
    */

    if(m_Operator.getYButtonPressed() == true){
      System.out.println("Y Button Pressed");
      m_CollectorArm.set(Value.kForward);
     
    }
    
    if(m_Operator.getAButtonPressed() == true){
      System.out.println("X Button Pressed");
      m_CollectorArm.set(Value.kReverse);
    }


    } // end testPeriodic()

  // create medthoids here to do stuff.  
}
