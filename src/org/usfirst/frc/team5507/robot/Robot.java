/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/**
 * 
 * @author Daphne Nong, Jennessa Ma, Julia Ma (No, Jennessa and Julia are NOT SISTERS!!!) 
 * 		   Special shoutout to Greg our mentor a.k.a. the mastermind of this code.
 * 	
 *
 */

public class Robot extends IterativeRobot {
	private DifferentialDrive m_robotDrive
			= new DifferentialDrive(new Spark(0), new Spark(1));
	private Joystick m_stick = new Joystick(0);
	private Timer m_timer = new Timer();
	AHRS ahrs;
	
	int current_state;
	final int STATE_DRIVE_FORWARD = 1;
	final int STATE_TURN_LEFT = 2;
	final int STATE_DRIVE_FORWARD_AGAIN = 3;
	final int STATE_DONE = 4;
	final int STATE_TURN_RIGHT = 5;
	final int STATE_DRIVE_FORWARD_AGAIN2 = 6;
	final int STATE_TURN_LEFT_AGAIN = 7;
	final int STATE_DRIVE_FORWARD_AGAIN3 = 8;
	final int STATE_TURN_LEFT_AGAIN2 = 9;
	final int STATE_DRIVE_FORWARD_AGAIN4 = 10;
	final int STATE_TURN_LEFT_AGAIN3 = 11;
	final int STATE_DRIVE_FORWARD_AGAIN5 = 12;
	final int STATE_TURN_RIGHT_AGAIN = 13;
	final int STATE_DRIVE_FORWARD_AGAIN6 = 14;
	final int STATE_TURN_RIGHT_AGAIN2 = 15;
	final int STATE_DRIVE_FORWARD_AGAIN7 = 16;
	final int STATE_TURN_LEFT_AGAIN4 = 17;
	
	final double SPEED = -0.5;
	int current_auto_mode;
	final int AUTO_DRIVE_STRAIGHT_TURN_LEFT_DROP = 1;
	final int AUTO_DRIVE_STRAIGHT_TURN_RIGHT_DROP = 2;
	final int AUTO_DRIVE_STRAIGHT = 3;
	final int AUTO_4 = 4;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		 try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            //ahrs = new AHRS(SerialPort.Port.kUSB1);
            //ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)200);
            //ahrs = new AHRS(SPI.Port.kMXP);
            ahrs = new AHRS(I2C.Port.kMXP);
        	ahrs.enableLogging(true);
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		current_auto_mode = AUTO_4;
		m_timer.reset();
		m_timer.start();
		ahrs.reset();
		switch (current_auto_mode) {
			case AUTO_DRIVE_STRAIGHT_TURN_LEFT_DROP:
				current_state = STATE_DRIVE_FORWARD;
				break;
			
			case AUTO_DRIVE_STRAIGHT_TURN_RIGHT_DROP:
				current_state = STATE_DRIVE_FORWARD;
				break;
			
			case AUTO_DRIVE_STRAIGHT: 
				current_state = STATE_DRIVE_FORWARD;
				break;
			
			case AUTO_4:
				current_state = STATE_DRIVE_FORWARD;
				break;
				
		}			
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (current_auto_mode) {
		case AUTO_DRIVE_STRAIGHT_TURN_LEFT_DROP:
			this.autonomousDriveStraightTurnLeftDrop();
			break;
			
		case AUTO_DRIVE_STRAIGHT_TURN_RIGHT_DROP:
			this.autonomousDriveStraightTurnRightDrop();
			break;
			
		case AUTO_DRIVE_STRAIGHT:
			this.autonomousDriveStraight();
			break;
			
		case AUTO_4:
			this.auto4();
			break;
			
		}
		SmartDashboard.putNumber("IMU_Yaw",ahrs.getYaw());
		SmartDashboard.putNumber("current_state", current_state);
		SmartDashboard.putNumber("Time", m_timer.get());
		
	}
	
	public void autonomousDriveStraightTurnLeftDrop() {
		switch (current_state) {
		case STATE_DRIVE_FORWARD: 
			m_robotDrive.arcadeDrive(-0.4, 0.0);
			if (m_timer.get() > 4.0)
			{
				current_state = STATE_TURN_LEFT;
			}
			break;
			
		case STATE_TURN_LEFT:
			if(ahrs.getYaw() > -88)
        	{
        		m_robotDrive.arcadeDrive(0, -0.5);
        	}
        	else if((ahrs.getYaw() > -92) && (ahrs.getYaw() < -88))
        	{
        		m_robotDrive.arcadeDrive(0, 0);
        		current_state = STATE_DRIVE_FORWARD_AGAIN;
        		m_timer.reset();
        	}
        	else if(ahrs.getYaw() < -92)
        	{
        		m_robotDrive.arcadeDrive(0, -0.5);
        	}
			break;
			
		case STATE_DRIVE_FORWARD_AGAIN:
			m_robotDrive.arcadeDrive(-0.6, 0.0);
			if (m_timer.get() > 4.0)
			{
				current_state = STATE_DONE;
			}
			break;
			
		case STATE_DONE:
			m_robotDrive.arcadeDrive(0.0, 0.0);
			break;
			
		}
	}
	
	public void autonomousDriveStraightTurnRightDrop() {
		switch (current_state) {
		case STATE_DRIVE_FORWARD: 
			m_robotDrive.arcadeDrive(-0.5, 0.0);
			if (m_timer.get() > 6.0)
			{
				current_state = STATE_TURN_LEFT;
			}
			break;
			
		case STATE_TURN_RIGHT:
			if(ahrs.getYaw() < 88)
        	{
        		m_robotDrive.arcadeDrive(0, 0.5);
        	}
        	else if((ahrs.getYaw() < 92) && (ahrs.getYaw() > 88))
        	{
        		m_robotDrive.arcadeDrive(0, 0);
        		current_state = STATE_DRIVE_FORWARD_AGAIN;
        		m_timer.reset();
        	}
        	else if(ahrs.getYaw() > 92)
        	{
        		m_robotDrive.arcadeDrive(0, 0.5);
        	}
			break;
			
		case STATE_DRIVE_FORWARD_AGAIN:
			m_robotDrive.arcadeDrive(-0.5, 0.0);
			if (m_timer.get() > 3.0)
			{
				current_state = STATE_DONE;
			}
			break;
			
		case STATE_DONE:
			m_robotDrive.arcadeDrive(0.0, 0.0);
			break;
			
		}
	}
	
	public void autonomousDriveStraight()
	{
		switch(current_state) {
			case STATE_DRIVE_FORWARD:
				if(m_timer.get() < 6)
				{
					m_robotDrive.arcadeDrive(-0.5, 0.0);
				}
				break;
		
			case STATE_DONE:
				m_robotDrive.arcadeDrive(0.0, 0.0);
			break;
			
		}
	}
	
	public void auto4()
	{
		switch(current_state)
		{
			case STATE_DRIVE_FORWARD:
				m_robotDrive.arcadeDrive(SPEED, 0.0);
				if(m_timer.get() > 2.0)
				{		
					current_state = STATE_TURN_LEFT;
				}
				break;
		
			case STATE_TURN_LEFT:
				if(ahrs.getYaw() > -88)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
	        	else if((ahrs.getYaw() > -92) && (ahrs.getYaw() < -88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DRIVE_FORWARD_AGAIN;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() < -92)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
				break;
				
			//drop box
			case STATE_DRIVE_FORWARD_AGAIN:
				m_robotDrive.arcadeDrive(SPEED ,0.0);
				if(m_timer.get() > 2.0)
				{					
					current_state = STATE_TURN_RIGHT;
				}
				break;
				
			case STATE_TURN_RIGHT:			
				if(ahrs.getYaw() < 88)
	        	{
	        		m_robotDrive.arcadeDrive(0, 0.5);
	        	}
	        	else if((ahrs.getYaw() < 92) && (ahrs.getYaw() > 88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DRIVE_FORWARD_AGAIN2;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() > 92)
	        	{
	        		m_robotDrive.arcadeDrive(0, 0.5);
	        	}
				break;
			
			case STATE_DRIVE_FORWARD_AGAIN2:
				m_robotDrive.arcadeDrive(SPEED , 0.0);
				if(m_timer.get() > 2.0)
				{					
					current_state = STATE_TURN_LEFT_AGAIN;
				}
				break;
				
			case STATE_TURN_LEFT_AGAIN:
				if(ahrs.getYaw() > -88)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
	        	else if((ahrs.getYaw() > -92) && (ahrs.getYaw() < -88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DRIVE_FORWARD_AGAIN3;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() < -92)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
				break;
				
			case STATE_DRIVE_FORWARD_AGAIN3:
				m_robotDrive.arcadeDrive(SPEED , 0.0);
				if(m_timer.get() > 2.0)
				{
					current_state = STATE_TURN_LEFT_AGAIN2;
				}
				break;
				
			case STATE_TURN_LEFT_AGAIN2:
				if(ahrs.getYaw() > -88)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
	        	else if((ahrs.getYaw() > -92) && (ahrs.getYaw() < -88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DRIVE_FORWARD_AGAIN4;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() < -92)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
				break;
				
			case STATE_DRIVE_FORWARD_AGAIN4:
				m_robotDrive.arcadeDrive(SPEED , 0.0);
				if(m_timer.get() > 2.0)
				{
					current_state = STATE_TURN_LEFT_AGAIN3;
				}
				break;
				
			case STATE_TURN_LEFT_AGAIN3:
				if(ahrs.getYaw() > -88)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
	        	else if((ahrs.getYaw() > -92) && (ahrs.getYaw() < -88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DRIVE_FORWARD_AGAIN5;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() < -92)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
				break;
				
			case STATE_DRIVE_FORWARD_AGAIN5:
				m_robotDrive.arcadeDrive(SPEED , 0.0);
				if(m_timer.get() > 2.0)
				{
					current_state = STATE_TURN_RIGHT_AGAIN;
				}
				break;
				
			case STATE_TURN_RIGHT_AGAIN:
				if(ahrs.getYaw() < 88)
	        	{
	        		m_robotDrive.arcadeDrive(0, 0.5);
	        	}
	        	else if((ahrs.getYaw() < 92) && (ahrs.getYaw() > 88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DRIVE_FORWARD_AGAIN6;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() > 92)
	        	{
	        		m_robotDrive.arcadeDrive(0, 0.5);
	        	}
				break;
				
			case STATE_DRIVE_FORWARD_AGAIN6:
				m_robotDrive.arcadeDrive(SPEED , 0.0);
				if(m_timer.get() > 2.0)
				{
					current_state = STATE_TURN_RIGHT_AGAIN2;
				}
				break;
				
			case STATE_TURN_RIGHT_AGAIN2:
				if(ahrs.getYaw() < 88)
	        	{
	        		m_robotDrive.arcadeDrive(0, 0.5);
	        	}
	        	else if((ahrs.getYaw() < 92) && (ahrs.getYaw() > 88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DRIVE_FORWARD_AGAIN7;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() > 92)
	        	{
	        		m_robotDrive.arcadeDrive(0, 0.5);
	        	}
				break;
				
			case STATE_DRIVE_FORWARD_AGAIN7:
				m_robotDrive.arcadeDrive(SPEED , 0.0);
				if(m_timer.get() > 2.0)
				{
					current_state = STATE_TURN_LEFT_AGAIN4;
				}
				break;
				
			case STATE_TURN_LEFT_AGAIN4:
				if(ahrs.getYaw() > -88)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
	        	else if((ahrs.getYaw() > -92) && (ahrs.getYaw() < -88))
	        	{
	        		m_robotDrive.arcadeDrive(0, 0);
	        		current_state = STATE_DONE;
	        		m_timer.reset();
	        	}
	        	else if(ahrs.getYaw() < -92)
	        	{
	        		m_robotDrive.arcadeDrive(0, -0.5);
	        	}
				break;
				
			case STATE_DONE:
				m_robotDrive.arcadeDrive(0.0, 0.0);
				break;
				
		}
	}
	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("testMessage", "Hello");
		SmartDashboard.putNumber("JoystickY", m_stick.getY());
		SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw()); //yaw range -180 to 180
        
        if(m_stick.getRawButton(1))
        {
        	SmartDashboard.putBoolean("rotateMode", true);
        	if(ahrs.getYaw() < 88)
        	{
        		m_robotDrive.arcadeDrive(0, 0.75);
        	}
        	else if((ahrs.getYaw() < 92) && (ahrs.getYaw() > 88))
        	{
        		m_robotDrive.arcadeDrive(0, 0);
        	}
        	else if(ahrs.getYaw() > 92)
        	{
        		m_robotDrive.arcadeDrive(0, -0.75);
        	}
        }
        else 
        {
        	m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
        	SmartDashboard.putBoolean("rotateMode", false);
        }
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
