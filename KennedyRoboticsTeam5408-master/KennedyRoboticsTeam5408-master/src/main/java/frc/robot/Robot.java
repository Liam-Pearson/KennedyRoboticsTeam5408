// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //init of motors ([MotorName] = new Talon([Port]);)
  // left side motors.
  private Talon Motor0FrontLeft = new Talon(0); //left drive motor
  private Talon Motor1BackLeft = new Talon(1); //right drive motor
  // right side motors.
  private Talon Motor2FrontRight = new Talon(2); //left drive motor (back left)
  private Talon Motor3BackRight = new Talon(3); //right drive motor
  // pulley system motor.
  private Talon Motor4Pulley = new Talon(4); // pulley system drive motor.
  // telescoping arm motor.
  private Talon Motor5Telescope = new Talon(5); // telescoping arm motor.

  // motor control groups for left and side sides of the robot.
  MotorControllerGroup leftMotorCG = new MotorControllerGroup(Motor0FrontLeft, Motor1BackLeft);
  MotorControllerGroup rightMotorCG = new MotorControllerGroup(Motor2FrontRight, Motor3BackRight);

  // master differental drive group for the robot.
  DifferentialDrive driveCG = new DifferentialDrive(leftMotorCG, rightMotorCG);

  // variables for drive motors (non-differental, hard coded solution version).
  double leftMotors;
  double rightMotors;

  // start time for autonmous.
  long autoStartTime;

  // solenoid object decleration. PCMConeGrabber is what grabs the game cones.
  DoubleSolenoid PCMConeGrabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  //init of controllers/joysticks ([Name] = new [Type]([USBPort]))
  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

  // all pretty self explainitory.
  double joystickXAxisPos;
  double joystickYAxisPos;
  double joystickZAxisPos;
  // speedAxis is controlled by the scroller on the joystick.
  double joystickSpeedAxisPos;

  // drive motor control.
  double deadzoneLimit = 0.4; // sets the value that the joystick position must exceed for movement of the robot to occur.
  double speed = 0;
  boolean pulleyMotorToggle = false;

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    joystickXAxisPos = joystick.getRawAxis(0); //x axis on joystick
    joystickYAxisPos = joystick.getRawAxis(1); //y axis on joystick
    joystickZAxisPos = joystick.getRawAxis(2); //z axis on joystick
    joystickSpeedAxisPos = joystick.getRawAxis(3); //y axis on joystick

    // prints the current position of the joystick.
    System.out.println("x-pos: " + joystickXAxisPos);
    System.out.println("y-pos: " + joystickYAxisPos);

    // deadzones.
    if (Math.abs(joystickXAxisPos) < deadzoneLimit || Math.abs(joystickXAxisPos) < deadzoneLimit*-1 ) {
      System.out.println(joystickXAxisPos);
      joystickXAxisPos = 0; // sets value of joystickXAxisPos to 0 if joystick value is under the deadzoneLimit.
    } // end if.

    if (Math.abs(joystickYAxisPos) < deadzoneLimit || Math.abs(joystickYAxisPos) < deadzoneLimit*-1) {
      System.out.println(joystickYAxisPos);
      joystickYAxisPos = 0; // sets value of joystickYAxisPos to 0 if joystick value is under the deadzoneLimit.
    } // end if.

    speed = ((Math.abs(joystickSpeedAxisPos)-1)*4.9); // sets the speed of the robot by dividing the motor speed by the speed value.

    // prints robot speed information.
    System.out.println("Speed input: " + joystickSpeedAxisPos + "\n Re-calculated speed input: " + speed);

    // LEGACY: setting speed of left and right motors.
    leftMotors = (joystickYAxisPos+joystickXAxisPos+joystickZAxisPos+joystickSpeedAxisPos)/speed; // left motors are postive values. The value is then divided to set speed.
    rightMotors = (joystickYAxisPos-joystickXAxisPos-joystickZAxisPos-joystickSpeedAxisPos)/speed; // right motors are negative values. The value is then divided to set speed.

    // LEGACY:motors are inverted depending on the side. To move forward, left motors must be positive values, while right side motors must be set to negative values, or vice-versa.
    Motor1BackLeft.set(leftMotors); // setting speed of back left drive motor.
    Motor3BackRight.set(rightMotors*-1); // setting speed of back right motor.
    Motor0FrontLeft.set(leftMotors); // setting speed of front left drive motor.
    Motor2FrontRight.set(rightMotors*-1); // setting speed of front right motor.

    // arcade drive. Not sure how this works.
    // driveCG.arcadeDrive(joystickYAxisPos, joystickXAxisPos);

    // controls the speed of the pulley system when button 1 (front trigger) is pressed.
    if (joystick.getRawButtonPressed(1)) {
      Motor4Pulley.set(joystickSpeedAxisPos);
    } // end if.

    // Initialize the DoubleSolenoid so it knows where to start.  Not required for single solenoids.
    PCMConeGrabber.set(DoubleSolenoid.Value.kReverse);

    // if button two is pressed, toggle the piston.
    if (joystick.getRawButtonPressed(2)) {
      PCMConeGrabber.toggle();
    } // end if.
  } // end robotPeriodic.

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoStartTime = System.currentTimeMillis();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } // end if.
  } // end autonomousInit.

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    long autoTimer = System.currentTimeMillis();

    leftMotors = 0.2; // left motors autonmous speed.
    rightMotors = 0.2; // right motors autonomous speed.

    // hello.

    // if less than 3 seconds has passed, drive the robot forward. This is extremely basic (and relatively useless) autonomous code.
    if (autoTimer - autoStartTime > 3000) {
      Motor1BackLeft.set(leftMotors); // setting speed of back left drive motor.
      Motor3BackRight.set(rightMotors*-1); // setting speed of back right motor.
      Motor0FrontLeft.set(leftMotors); // setting speed of front left drive motor.
      Motor2FrontRight.set(rightMotors*-1); // setting speed of front right motor.

      // arcade drive. Not sure how this works.
      // driveCG.arcadeDrive(0.2, 0.2);
    } // end if.
  } // end autonomousPeriodic.

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } // end if.
  } // end teleopInit.

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  } // end testInit.

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
