/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.limelight;


/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {
  
  private MecanumDrive m_robotDrive;

  private XboxController m_Driver;
  private XboxController m_Operator;

  private VictorSPX m_LeftShooter = new VictorSPX(Const.kLeftShooterID);
  private VictorSPX m_RightShooter = new VictorSPX(Const.kRightShooterID);

  private PWMVictorSPX m_liftMotor = new PWMVictorSPX(Const.kLiftMotorID);
  private PWMVictorSPX m_collectorMotor = new PWMVictorSPX(Const.kCollectorPWMPort);

  private double motorSpeed;

  
  final Compressor m_compressor = new Compressor();
  final DoubleSolenoid m_CollectorArm  = new DoubleSolenoid(Const.kPCMCanID,Const.kCollectorForwardPort, Const.kCollectorBackwardPort);
  
  final limelight m_limelight = new limelight();

  private AHRS ahrs = new AHRS();

  
  @Override
  public void robotInit() {

    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.

       ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP);
    } 
    catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    // create the talonSRX motor controller objects

    final WPI_TalonSRX frontLeft = new WPI_TalonSRX(Const.kFrontLeftChannel);
    final WPI_TalonSRX rearLeft = new WPI_TalonSRX(Const.kRearLeftChannel);
    final WPI_TalonSRX frontRight = new WPI_TalonSRX(Const.kFrontRightChannel);
    final WPI_TalonSRX rearRight = new WPI_TalonSRX(Const.kRearRightChannel);

    //create the victorSPX motor controller objects
    final VictorSPX m_LeftShooter = new VictorSPX(Const.kLeftShooterID);
    final VictorSPX m_RightShooter = new VictorSPX(Const.kRightShooterID);
    final VictorSPX m_liftMotor = new VictorSPX(Const.kLiftMotorID);
 
    frontLeft.setNeutralMode(NeutralMode.Brake);
    rearLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    //frontLeft.setInverted(true);
    //rearLeft.setInverted(true);
    m_LeftShooter.setInverted(true);
    m_RightShooter.follow(m_LeftShooter);

    
    m_compressor.enabled();

    m_limelight.setCAMMode(m_limelight.kCamModeVisionProc);
    m_limelight.setStreamMode(m_limelight.kStreamModePIPMain);
    m_limelight.setLEDMode(m_limelight.kLedModePipeLineSetting);

    //m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    m_Driver = new XboxController(Const.kDriverXBoxPort);
    m_Operator = new XboxController(Const.kOperXBoxPort);

    m_LeftShooter.set(ControlMode.PercentOutput,0);
    m_RightShooter.set(ControlMode.PercentOutput,0);
    m_liftMotor.set(ControlMode.PercentOutput,0);

    System.out.println("Set motors to zero");


  } // *********************** robotInit() *************************
  
  
  @Override
  public void teleopInit() {
    // Set the right shooter to do what the left shooter does, powerwise.
    m_LeftShooter.set(ControlMode.PercentOutput,motorSpeed);
    m_RightShooter.follow(m_LeftShooter);

  } // ********************** teleopInit() ************************
  
  
  
  
  @Override
  public void teleopPeriodic() {
    
    // Invert the driver controls and set the limelight to not vision processing
    if (m_Driver.getBumper(Hand.kLeft) == true){
      //System.out.println("Inverted y speed: ",;
      System.out.println("Left bumper pressed - Drive backwards$");
      m_robotDrive.driveCartesian(m_Driver.getX(Hand.kRight) ,
                                  m_Driver.getY(Hand.kLeft) , 
                                  m_Driver.getX(Hand.kLeft));

      m_limelight.setStreamMode(m_limelight.kStreamModePIP2nd);
      m_limelight.setCAMMode(m_limelight.kCamModeDriver);
      m_limelight.setLEDMode(m_limelight.kLedModeOff);
    } // end if bumper held
    else 
    { // Drive it like you stole it  With vision processing
      m_robotDrive.driveCartesian(m_Driver.getX(Hand.kRight)* -1,
                                   m_Driver.getY(Hand.kLeft)* -1,
                                   m_Driver.getX(Hand.kLeft));

      m_limelight.setStreamMode(m_limelight.kStreamModePIPMain);
      m_limelight.setCAMMode(m_limelight.kCamModeVisionProc);
      m_limelight.setLEDMode(m_limelight.kLedModePipeLineSetting);
    }

    // Shooter - Hooked to Operator left and right trigger
    if (m_Operator.getTriggerAxis(Hand.kLeft) != 0){
    motorSpeed = m_Operator.getTriggerAxis(Hand.kLeft) * -1;
    }
    else if (m_Operator.getTriggerAxis(Hand.kRight) != 0){
    motorSpeed = m_Operator.getTriggerAxis(Hand.kRight);
    }
    else{
      motorSpeed = 0.0;
    }
    
    m_LeftShooter.set(ControlMode.PercentOutput,motorSpeed);

    

    if(m_Operator.getBumper(Hand.kLeft) == true){
      m_liftMotor.set(-0.35);
    } else {
      m_liftMotor.stopMotor();
    }

    // Lift - Hooked to Operator Right and Left Bumper 
    if(m_Operator.getBumper(Hand.kRight) == true){
      m_liftMotor.set(0.35);
    } 
    else if (m_Operator.getBumper(Hand.kLeft) == true) {
      m_liftMotor.set(-0.35); 
    }
    else{
      m_liftMotor.stopMotor();
    }

    

    // Collector - Hooked to X and B
    // If X collect in
    if (m_Operator.getXButton() == true){
      System.out.println("X collector should run");
      m_collectorMotor.set(-0.25);
    }
    // If 
    else if (m_Operator.getBButton() == true) {
      m_collectorMotor.set(0.50); // spit ball away
    }
    else{
      m_collectorMotor.stopMotor();
    }

    // Collector cgo down
    if(m_Operator.getYButtonPressed() == true){
      System.out.println("Y Button Pressed");
      m_CollectorArm.set(Value.kForward);
     
    }
    // Collector go up
    if(m_Operator.getAButtonPressed() == true){
      System.out.println("X Button Pressed");
      m_CollectorArm.set(Value.kReverse);
    }
  
                                
  }// ********************* End teleopPeriodic ***********************


  @Override
  public void testInit(){
    // Called at the begining of test mode

    // need to invert one of the shooter motors
    
    m_RightShooter.follow(m_LeftShooter);
  }

  @Override
  public void testPeriodic(){
/** 
    if(m_Operator.getYButtonPressed() == true){
      System.out.println("Y Button Pressed");
      m_CollectorArm.set(Value.kForward);
     
    }
    
    if(m_Operator.getAButtonPressed() == true){
      System.out.println("X Button Pressed");
      m_CollectorArm.set(Value.kReverse);
    }

    if(m_Operator.getBumper(Hand.kLeft) == true){
      m_liftMotor.set(-0.35);
    } else {
      m_liftMotor.stopMotor();
    }

    if(m_Operator.getBumper(Hand.kRight) == true){
      m_liftMotor.set(0.35);
    } else {
      m_liftMotor.stopMotor();
    }

    motorSpeed = m_Operator.getTriggerAxis(Hand.kRight) * -1;

    m_LeftShooter.set(ControlMode.PercentOutput,motorSpeed);
    //m_RightShooter.set(ControlMode.PercentOutput,motorSpeed);

    if (m_Operator.getXButton() == true){
      System.out.println("X collector should run");
      m_collectorMotor.set(-0.25);

    }
    else{
      m_collectorMotor.stopMotor();
    }
    */

    // Testing driving on a gyro heading

      if (m_Driver.getBumper(Hand.kRight)) {
        ahrs.reset();
      }
      try {
        /* Use the joystick X axis for lateral movement, */
        /* Y axis for forward movement, and Z axis for rotation. */
        /* Use navX MXP yaw angle to define Field-centric transform */
        SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
        SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
        SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());
    
        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
    
        SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

        
        m_robotDrive.driveCartesian(m_Driver.getX(Hand.kRight)* -1,
                                    m_Driver.getY(Hand.kLeft)* -1,
                                    m_Driver.getX(Hand.kLeft),
                                    ahrs.getAngle());
      } catch (RuntimeException ex) {
        DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
      }


    } // ********************** end testPeriodic() *********************

  // create medthoids here to do stuff.  
}
