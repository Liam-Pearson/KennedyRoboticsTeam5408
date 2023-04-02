// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  // init of motors ([MotorName] = new Talon([Port]);)
  // left side motors.
  private Talon Motor0FrontLeft = new Talon(0); // left drive motor
  private Talon Motor1BackLeft = new Talon(1); // right drive motor
  // right side motors.
  private Talon Motor2FrontRight = new Talon(2); // left drive motor (back left)
  private Talon Motor3BackRight = new Talon(3); // right drive motor
  // telescoping arm motors.
  private Talon Motor5Telescope = new Talon(5); // left telescoping arm motor.

  private static SPI.Port gyroPort = SPI.Port.kOnboardCS0;

  // solenoid object decleration. PCMConeGrabber is what grabs the game cones.
  DoubleSolenoid PCMConeGrabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  // built in RoboRio Accelerometer.
  Accelerometer accelerometer = new BuiltInAccelerometer();

  // gryoscope.
  ADXRS450_Gyro gyro = new ADXRS450_Gyro(gyroPort);

  // master start time for robot.
  long masterStartTime = System.currentTimeMillis();

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture(0); // front camera.
    CameraServer.startAutomaticCapture(1); // top camera.

    PCMConeGrabber.set(DoubleSolenoid.Value.kReverse); // sets the solenoid to forward. Opens the claw.

    gyro.calibrate();

  } // end robotInit.

  // emergency stop condition.
  boolean emergencyStop = false;

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  } // end robotPeriodic.

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
  public void teleopPeriodic() {
    if(emergencyStop == false){
      controllerDrive(0.2, 8.9, false); // allows the robot to drive.
      armLifter(0.7, 0.5, 0.1, false); // allows the arm of the robot to lift.
      armExtender(0.6, 0.15, 0.1, false); // alows the arm to extend.
      clawOperations(false); // allows the claw to open and shut.
    } // end if.

    if(joystick.getRawButton(11)){
      emergencyStop = true;
      // System.out.println("EMERGENCY STOP!");
    } // end if.

    if(joystick.getRawButton(12)){
      emergencyStop = false;
      // System.out.println("EMERGENCY STOP DISABLED.");
    } // end if.

    // Initialize the DoubleSolenoid so it knows where to start. Not required for
    // single solenoids.
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  } // end testInit.

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  // all pretty self explainitory.
double joystickXAxisPos;
double joystickYAxisPos;
double joystickZAxisPos; 

// variables for drive motors (non-differental, hard coded solution version).
double leftMotors;
double rightMotors;

// init of controllers/joysticks ([Name] = new [Type]([USBPort]))
private Joystick joystick = new Joystick(0);

private XboxController xboxControllerFRC = new XboxController(0); // 0 is the USB Port to be used as indicated on the Driver Station

// handles all operations regarding controller-operated driving.
public void controllerDrive(double deadzone, double speedCoefficent, boolean controllerDriveOn) {
  
  if(controllerDriveOn == false){
  joystickXAxisPos = joystick.getRawAxis(0)/2; // x axis on joystick
  joystickYAxisPos = joystick.getRawAxis(1); // y axis on joystick
  joystickZAxisPos = joystick.getRawAxis(2);
  } // end if.
  else{
  joystickXAxisPos = xboxControllerFRC.getLeftX(); // x axis on joystick
  joystickYAxisPos = (xboxControllerFRC.getRightY())/2; // y axis on joystick
  System.out.println("Controller running.");
  System.out.println("X-axis: " + xboxControllerFRC.getLeftX() + " Y-axis: " + xboxControllerFRC.getRightY() + ".");
  } // end else

  // deadzones for joystick and controller.
  // ==========================================================================================================================
  if (Math.abs(joystickXAxisPos) < deadzone) {
    System.out.println(joystickXAxisPos);
    joystickXAxisPos = 0; // sets value of joystickXAxisPos to 0 if joystick value is under the deadzoneLimit.
  } // end if.

  if (Math.abs(joystickYAxisPos) < deadzone) {
    System.out.println(joystickYAxisPos);
    joystickYAxisPos = 0; // sets value of joystickYAxisPos to 0 if joystick value is under the deadzoneLimit.
  } // end if.
  // ==========================================================================================================================

  // setting speed of left and right motors.
  leftMotors = (joystickYAxisPos - joystickXAxisPos - joystickZAxisPos); // left motors are postive values. The value is then divided to set speed.
  rightMotors = (joystickYAxisPos + joystickXAxisPos + joystickZAxisPos); // right motors are negative values. The value is then divided to set speed.

  // motors are inverted depending on the side. To move forward, left motors must
  // be positive values, while right side motors must be set to negative values,
  // or vice-versa.
  // motors are totally weird and wrong due to wiring and positioning but they
  // work.
  Motor0FrontLeft.set(leftMotors); // setting speed of front left drive motor.
  Motor1BackLeft.set(leftMotors*-1); // setting speed of back left drive motor.
  Motor2FrontRight.set(rightMotors); // setting speed of front right motor.
  Motor3BackRight.set(rightMotors); // setting speed of back right motor.

  // robot motor diagram.
  // Front [N]
  // 0----2
  // | |
  // | |
  // 1----3
} // end controllerDrive.

// pulley system motor.
private Talon Motor4Pulley = new Talon(7); // pulley system drive motor.

// handles all operations regarding lifting of the arm.
public void armLifter(double armLiftUpSpeed, double armLiftDownSpeed, double armConstantBreakingSpeed, boolean controllerDriveOn) {

  if(controllerDriveOn == false){
  // controls the speed of the arm pulley system
    if (joystick.getRawButton(1)) {
      System.out.println("Moving up at " + armLiftUpSpeed);
      Motor4Pulley.set(armLiftUpSpeed*-1);
    } // end if.
    else if (joystick.getRawButton(2)) {
      Motor4Pulley.set(armLiftDownSpeed); // retract arm.
    } // end else if.
    else {
      Motor4Pulley.set(armConstantBreakingSpeed*-1);
      System.out.println("Arm breakinbg.");
    } // end else.
  } // end if.
  else{
    System.out.println("Controller being used.");
    Motor4Pulley.set(xboxControllerFRC.getRightTriggerAxis() - (xboxControllerFRC.getLeftTriggerAxis()/2) + armConstantBreakingSpeed);
    xboxControllerFRC.setRumble(RumbleType.kBothRumble, 0.5);
  } // end else.
} // end armExtender.

// handles all operations regarding extending of the arm.
public void armExtender(double telescopeOutSpeed, double telescopeInSpeed, double telescopeConstantBreakSpeed, boolean controllerDriveOn) {
  // controls the speed of the telescoping arm pulley system.
  if(controllerDriveOn == false){
    if (joystick.getRawButton(5)) {
      Motor5Telescope.set(telescopeOutSpeed);
    } // end if.
    else if (joystick.getRawButton(6)) {
      Motor5Telescope.set(telescopeInSpeed*-1); // retract telescoping arm.
    } // end else if.
    else {
      Motor5Telescope.set(telescopeConstantBreakSpeed);
    } // end else.
  } // end if.
  else{
    if(xboxControllerFRC.getRightBumper()){
      Motor5Telescope.set(telescopeOutSpeed);
      xboxControllerFRC.setRumble(RumbleType.kBothRumble, 0.5);
    } // end if.
    else if(xboxControllerFRC.getLeftBumper()){
      Motor5Telescope.set(telescopeInSpeed*-1); // retract telescoping arm.
      xboxControllerFRC.setRumble(RumbleType.kBothRumble, 0.5);
    } // end else if.
    else{
      Motor5Telescope.set(telescopeConstantBreakSpeed);
    } // end else.
  } // end else.
} // end armExtender.

boolean clawToggle = false; // claw on or off.

// handles all operations regarding opening and closing of the claw.
public void clawOperations(boolean controllerDriveOn) {
  // if button three is pressed, clawToggle will be set to true, thus closing the
  // claw inward.
  if(controllerDriveOn == false){
    if (joystick.getRawButton(3)) {
      clawToggle = true;
      PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward.
    } // end if.
    // if button 4 is pressed, clawToggle will be set to false.
    if (joystick.getRawButton(4)) {
      clawToggle = false;
      // System.out.println("CLAW OPEN.");
      PCMConeGrabber.set(DoubleSolenoid.Value.kReverse); // sets the solenoid to reverse.
    } // end if.
    // checks the condition of clawToggle. If clawToggle is true, close the claw.
    if (clawToggle == true) {
      PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward.
    } // end if.
  } // end if.
  else{
    if(xboxControllerFRC.getBButton()){
      clawToggle = true;
      PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward. Opens claw.
      xboxControllerFRC.setRumble(RumbleType.kLeftRumble, 0.5);
    } // end if.
    // if button 4 is pressed, clawToggle will be set to false.
    if (xboxControllerFRC.getXButton()) {
      clawToggle = false;
      PCMConeGrabber.set(DoubleSolenoid.Value.kReverse); // sets the solenoid to reverse. Closes claw.
      xboxControllerFRC.setRumble(RumbleType.kRightRumble, 0.5);
    } // end if.
    // checks the condition of clawToggle. If clawToggle is true, close the claw.
    if (clawToggle == true) {
      PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward.
  } // end else.
}
} // end clawOperations

/** This function is called once each time the robot enters Disabled mode. */
@Override
public void disabledInit() {
}

@Override
public void disabledPeriodic() {
}


/**
 * This autonomous runs the autonomous command selected by your
 * {@link RobotContainer} class.
 */
@Override
public void autonomousInit() {
  m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  masterStartTime = System.currentTimeMillis();

  // schedule the autonomous command (example)
  if (m_autonomousCommand != null) {
    m_autonomousCommand.schedule();
  } // end if.
} // end autonomousInit.

/** This function is called periodically during autonomous. */
@Override
public void autonomousPeriodic() {
  // autonomousDriveForwards();
  autonomousScoreCube();
} // end autonomousPeriodic.

long timeOfAutonomous = 0;

boolean autoCorrectPositionAutonomous = true;

double driveSpeed = 0;

public void autonomousDriveForwards(){
  long autoTimer = System.currentTimeMillis();

  timeOfAutonomous = autoTimer - masterStartTime;

  // if less than 3 seconds has passed, drive the robot forward. This is extremely
  // basic (and relatively useless) autonomous code.
  if (timeOfAutonomous < 1500) {
    System.out.println("TIME OF AUTONOMOUS = " + timeOfAutonomous + ".");

    Motor0FrontLeft.set(0.55 * -1); // setting speed of front left drive motor.
    Motor1BackLeft.set(0.55); // setting speed of back left drive motor.
    Motor2FrontRight.set(0.55*-1); // setting speed of front right motor.
    Motor3BackRight.set(0.55*-1); // setting speed of back right motor.

    System.out.println("AUTONOMOUS RUNNING AND MOVING FORWARDS.");

    System.out.println("GYRO: " + gyro.getAngle());

    // courseCorrectionAutonomous(10,10);

    // robot motor diagram.
    // Front [N]
    // 0----2
    // | |
    // | |
    // 1----3

  } // end if.
  else if (timeOfAutonomous < 2500) {
    System.out.println("TIME OF AUTONOMOUS = " + timeOfAutonomous + ".");

    Motor0FrontLeft.set(0.3 * -1); // setting speed of front left drive motor.
    Motor1BackLeft.set(0.3); // setting speed of back left drive motor.
    Motor2FrontRight.set(0.3*-1); // setting speed of front right motor.
    Motor3BackRight.set(0.3*-1); // setting speed of back right motor.

    System.out.println("AUTONOMOUS RUNNING AND MOVING FORWARDS.");

    System.out.println("GYRO: " + gyro.getAngle());

    // courseCorrectionAutonomous(10,10);

    // robot motor diagram.
    // Front [N]
    // 0----2
    // | |
    // | |
    // 1----3

  } // end if.
  else{
    Motor0FrontLeft.set(0); // setting speed of front left drive motor.
    Motor1BackLeft.set(0); // setting speed of back left drive motor.
    Motor2FrontRight.set(0); // setting speed of front right motor.
    Motor3BackRight.set(0); // setting speed of back right motor.
  } // end else.
} // end autonomouDriveForwards.

public void autonomousScoreCube(){
  long autoTimer = System.currentTimeMillis();

  timeOfAutonomous = autoTimer - masterStartTime;

  if(timeOfAutonomous < 1300){
    PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward. Opens claw.
    Motor4Pulley.set(0.3); // lower arm.
    System.out.println("LOWERING AUTO ARM.");
  } // end if.
  else if(timeOfAutonomous < 3700){
    Motor5Telescope.set(0.5); // extend out arm.
    Motor4Pulley.set(-0.2); // lock posiiton.
    System.out.println("EXTENDING AUTO ARM.");
  } // end else if.
  else if(timeOfAutonomous < 7300){
    PCMConeGrabber.set(DoubleSolenoid.Value.kReverse); // sets the solenoid to forward. Opens claw.
    Motor5Telescope.set(0.2); // lock position.
    Motor4Pulley.set(-0.2); // lock position.
    System.out.println("OPENING AUTO CLAW.");
  } // end else if.
  else if(timeOfAutonomous < 7500){
    Motor5Telescope.set(-0.2); // lock position.
    Motor4Pulley.set(-0.6); // lock position.
  } // SLAM UP
  else if(timeOfAutonomous < 7700){
    Motor5Telescope.set(-0.2); // lock position.
    Motor4Pulley.set(0.6); // lock position.
  } // SLAM DOWN
  else if(timeOfAutonomous < 8200){
    Motor4Pulley.set(-0.6); // lock position.
    Motor5Telescope.set(0.3); // lock position.
  }
  else if(timeOfAutonomous < 10500){
    Motor5Telescope.set(-0.3); // lock position.
    Motor4Pulley.set(-0.2); // lock position.
  }
  else if(timeOfAutonomous < 12500){ // drives the robot in reverse.
    System.out.println("RETRACTING AUTO ARMS AND REVERSING.");

    Motor0FrontLeft.set(0.6 ); // setting speed of front left drive motor.
    Motor1BackLeft.set(-0.6); // setting speed of back left drive motor.
    Motor2FrontRight.set(0.85); // setting speed of front right motor.
    Motor3BackRight.set(0.85); // setting speed of back right motor.
    Motor5Telescope.set(0.2); // lock position.
    Motor4Pulley.set(-0.2); // lock position.
  } // end else if.
  else if(timeOfAutonomous < 13700){
    System.out.println("TURNING.");

    Motor0FrontLeft.set(0.6 ); // setting speed of front left drive motor.
    Motor1BackLeft.set(-0.6); // setting speed of back left drive motor.
    Motor2FrontRight.set(-0.7); // setting speed of front right motor.
    Motor3BackRight.set(-0.7); // setting speed of back right motor.
    Motor5Telescope.set(0.2); // lock position.
    Motor4Pulley.set(-0.2); // lock position.
  }
  else{
    System.out.println("5");
    Motor0FrontLeft.set(0); // setting speed of front left drive motor.
    Motor1BackLeft.set(0); // setting speed of back left drive motor.
    Motor2FrontRight.set(0); // setting speed of front right motor.
    Motor3BackRight.set(0); // setting speed of back right motor.
    Motor5Telescope.set(0.2); // lock position.
    Motor4Pulley.set(-0.2); // lock position.
  } // end else.
} // end autonomousScoreCube.

public void courseCorrectionAutonomous(int rightCorrectionAngle, int leftCorrectionAngle){
  if (autoCorrectPositionAutonomous == true) {
    // if the robot turns right while in autonomous, turn robot left until it is
    // facing forwards again.

    if (gyro.getAngle() > rightCorrectionAngle) {
      Motor0FrontLeft.set(leftMotors); // setting speed of front left drive motor.
      Motor1BackLeft.set(leftMotors * -1); // setting speed of back left drive motor.
      Motor2FrontRight.set(rightMotors * -1); // setting speed of front right motor.
      Motor3BackRight.set(rightMotors * -1); // setting speed of back right motor.

      System.out.println("AUTONOMOUS CORRECTING TO THE LEFT.");

      System.out.println("GYRO: " + gyro.getAngle());
    } // end if.

    // if robot turns left while in autonomous, turn robot right until it is facing
    // forwards again.
    if (gyro.getAngle() < leftCorrectionAngle) {
      Motor0FrontLeft.set(leftMotors * 1); // setting speed of front left drive motor.
      Motor1BackLeft.set(leftMotors); // setting speed of back left drive motor.
      Motor2FrontRight.set(rightMotors); // setting speed of front right motor.
      Motor3BackRight.set(rightMotors); // setting speed of back right motor.

      System.out.println("AUTONOMOUS CORRECTING TO THE RIGHT.");
    } // end if.
  } // end if.
} // end courseCorrectionAutonomous.
} // end timedRobot.