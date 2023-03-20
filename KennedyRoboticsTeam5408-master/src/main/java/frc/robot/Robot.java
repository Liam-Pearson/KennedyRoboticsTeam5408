// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.math.filter.LinearFilter;
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
  // telescoping arm motors.
  private Talon Motor5Telescope = new Talon(5); // left telescoping arm motor.

  // variables for drive motors (non-differental, hard coded solution version).
  double leftMotors;
  double rightMotors;

  // start time for autonmous.
  long time = System.currentTimeMillis();

  // solenoid object decleration. PCMConeGrabber is what grabs the game cones.
  DoubleSolenoid PCMConeGrabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  // built in RoboRio Accelerometer.
  Accelerometer accelerometer = new BuiltInAccelerometer();

  // gryoscope.
  AnalogGyro gyro = new AnalogGyro(0);

  //init of controllers/joysticks ([Name] = new [Type]([USBPort]))
  private Joystick joystick = new Joystick(0);

  // turns on functions which, unless the boolean is set to true, would otherwise not run.
  boolean testMode = false;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture(0); // front camera.
    CameraServer.startAutomaticCapture(1); // top camera.

    // this is poorly written code but it works. It moves the robots arm to the proper position to start the game. Only runs if the boolean 'testMode' is set to true.
    if(testMode == true){
      for(int i = 0; i < 250; i++){
        Motor4Pulley.set(0.8);
      } // end for.
    } // end if

  } // end robotInit.

  // all pretty self explainitory.
  double joystickXAxisPos;
  double joystickYAxisPos;
  double joystickZAxisPos; // for turning in place. Currently not in use.
  // speedAxis is controlled by the scroller on the joystick.
  double joystickSpeedAxisPos;

  // drive motor control.
  double deadzoneLimit = 0.4; // sets the value that the joystick position must exceed for movement of the robot to occur.
  double speed = 0; // speed of robot.
  boolean clawToggle = false; // claw on or off.
  
  double indexOfArmHeight = 0; // position of arm.

  // robot in half-speed.
  int speedSlowOnOff = 0;

  // dev mode.
  boolean devMode = true;

  // emergency stop condition.
  boolean emergencyStop = false;

   /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if(emergencyStop == false){
      controllerDrive(); // allows the robot to drive.
      armLifter(); // allows the arm of the robot to lift.
      armExtender(); // alows the arm to extend.
      clawOperations(); // allows the claw to open and shut.
      robotPosition(); // allows the robot to calculate its position.
    } // end if.
    
    // Initialize the DoubleSolenoid so it knows where to start.  Not required for single solenoids.
    // PCMConeGrabber.set(DoubleSolenoid.Value.kReverse);
  } // end robotPeriodic.

  public void controllerDrive(){
    joystickXAxisPos = joystick.getRawAxis(0); // x axis on joystick
    joystickYAxisPos = joystick.getRawAxis(1); // y axis on joystick
    joystickZAxisPos = joystick.getRawAxis(2); // z axis on joystick
    joystickSpeedAxisPos = joystick.getRawAxis(3); // y axis on joystick    

    // prints the current position of the joystick.
    System.out.println("X-POS: " + joystickXAxisPos);
    System.out.println("Y-POS: " + joystickYAxisPos);

    // deadzones.
    if (Math.abs(joystickXAxisPos) < deadzoneLimit) {
      System.out.println(joystickXAxisPos);
      joystickXAxisPos = 0; // sets value of joystickXAxisPos to 0 if joystick value is under the deadzoneLimit.
    } // end if.

    if (Math.abs(joystickYAxisPos) < deadzoneLimit) {
      System.out.println(joystickYAxisPos);
      joystickYAxisPos = 0; // sets value of joystickYAxisPos to 0 if joystick value is under the deadzoneLimit.
    } // end if.

    // if button 7 is pressed, the robot moves slower. If button 7 is pressed again, the robot returns to regular speed.
    if(joystick.getRawButton(7)){
      speedSlowOnOff = speedSlowOnOff + 1;
      if(speedSlowOnOff > 1){
        speedSlowOnOff = 0;
      } // end if.
    } // end if.

    // if the value of speedSlowOnOff is 1, the robot moves at half speed. If the value of speedSlowOnOff is 0, the robot moves at regular speed.
    if(speedSlowOnOff == 1){
      speed = ((Math.abs(joystickSpeedAxisPos)-1)*8.9); // sets the speed of the robot by dividing the motor speed by the speed value.
    } // end if.
    else if(speedSlowOnOff == 0){
      speed = ((Math.abs(joystickSpeedAxisPos)-1)*8.9); // sets the speed of the robot by dividing the motor speed by the speed value.
    } // end else if.

    // prints robot speed information.
    // System.out.println("Speed input: " + joystickSpeedAxisPos + "\n Re-calculated speed input: " + speed);

    // setting speed of left and right motors.
    leftMotors = (joystickYAxisPos-joystickXAxisPos)/speed; // left motors are postive values. The value is then divided to set speed.
    rightMotors = (joystickYAxisPos+joystickXAxisPos)/speed; // right motors are negative values. The value is then divided to set speed.

    System.out.println("LEFT MOTORS: " + leftMotors);
    System.out.println("RIGHT MOTORS: " + rightMotors);

    // motors are inverted depending on the side. To move forward, left motors must be positive values, while right side motors must be set to negative values, or vice-versa.
    // motors are totally weird and wrong due to wiring and positioning but they work.
    Motor0FrontLeft.set(leftMotors*-1); // setting speed of front left drive motor.
    Motor1BackLeft.set(leftMotors); // setting speed of back left drive motor.
    Motor2FrontRight.set(rightMotors*-1); // setting speed of front right motor.
    Motor3BackRight.set(rightMotors*-1); // setting speed of back right motor.
    
    // robot motor diagram.
    // Front [N]
    // 0----2
    // |    |
    // |    |
    // 1----3
  } // end controllerDrive.

  public void armLifter(){
    // controls the speed of the arm pulley system
    if(joystick.getRawButton(1)){
      Motor4Pulley.set(0.8);
      System.out.println("ARM MOVING UP");
      // motorPullLengthIndex = motorPullLengthIndex + 0.01;
    } // end if.
    else if(joystick.getRawButton(2)){
      Motor4Pulley.set(-0.15); // retract arm.
      System.out.println("ARM MOVING DOWN");
    } // end else if.
    else{
      Motor4Pulley.set(0.2);
    } // end else.
  } // end armExtender.

  public void armExtender(){
    // controls the speed of the telescoping arm pulley system.
    if(joystick.getRawButton(5)){
      Motor5Telescope.set(-0.6);
      System.out.println("TELESCOPING ARM OUT");
    } // end if.
    else if(joystick.getRawButton(6)){
      Motor5Telescope.set(0.2); // retract telescoping arm.
      System.out.println("TELESCOPING ARM IN");
    } // end else if.
    else{
      Motor5Telescope.set(-0.2);
    } // end else.
  } // end armExtender.

  public void clawOperations(){
    // if button three is pressed, clawToggle will be set to true, thus closing the claw inward.
    if (joystick.getRawButton(3)) {
      clawToggle = true;
    } // end if.

    // if button 4 is pressed, clawToggle will be set to false.
    if(joystick.getRawButton(4)) {
      clawToggle = false;
    } // end if.

    // checks the condition of clawToggle. If clawToggle is true, close the claw.
    if(clawToggle == true) {
      PCMConeGrabber.set(Value.kForward); // sets the solenoid to forward.
    } // end if.
  } // end clawOperations

  // accerlation variables.
  // previously measured acceleration in the loop.
  double prevXAccel = 0;
  double prevYAccel = 0;
  // current acceleration in the loop.
  double xAccel = 0;
  double yAccel = 0;
  // x and y jerk that occurs during the Loop timing.
  double xJerk = 0;
  double yJerk = 0;
  // velocity variables.
  double xVelocity = 0;
  double yVelocity = 0;
  // start and finish times of calculations.
  long startTimeOfPosition = 0;
  long endTimeOfPosition = 0;
  long changeInTimeForPosition = 0;
  // x and y position of the robot.
  double xPositionOfRobot = 0;
  double yPositionOfRobot = 0;

  // filter to increase the accuracy of accelerometer data.
  LinearFilter AccelFilter = LinearFilter.movingAverage(10);

  public void robotPosition(){
    // documentation can be found at https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/accelerometers-software.html#builtinaccelerometer.

    // gets the start time of the calculation.
    startTimeOfPosition = System.currentTimeMillis();

    // gets the current acceleration in the x and y directions.
    xAccel = AccelFilter.calculate(accelerometer.getX());
    yAccel = AccelFilter.calculate(accelerometer.getY());

    // calcualtes the jerk of the robot. Not used for anything.
    xJerk = (xAccel - prevXAccel)/.02;
    yJerk = (yAccel - prevYAccel)/.02;

    // sets the previous acceleration valeus to the current acceleration values.
    prevXAccel = xAccel;
    prevYAccel = yAccel;

    // gets the end time of the calculation.
    endTimeOfPosition = System.currentTimeMillis();

    // calculates the change in time during the calculations for position.
    changeInTimeForPosition = endTimeOfPosition - startTimeOfPosition;

    // calculates the velocity of the robot in each axis.
    xVelocity = (xAccel/changeInTimeForPosition);
    yVelocity = (yAccel/changeInTimeForPosition);

    // calculates position using kinematics.
    xPositionOfRobot = ((xVelocity*changeInTimeForPosition)+(0.5*xAccel*(changeInTimeForPosition*changeInTimeForPosition)));
    yPositionOfRobot = ((yVelocity*changeInTimeForPosition)+(0.5*yAccel*(changeInTimeForPosition*changeInTimeForPosition)));

    // prints position value and gyro value.
    System.out.println("X-POSITION: " + xPositionOfRobot + " Y-POSITION: " + yPositionOfRobot);
    System.out.println("GYRO: " + gyro.getAngle());
  } // end robotPosition.

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    time = System.currentTimeMillis();

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

    // if less than 3 seconds has passed, drive the robot forward. This is extremely basic (and relatively useless) autonomous code.
    if (autoTimer - time < 3000) {
      Motor0FrontLeft.set(leftMotors*-1); // setting speed of front left drive motor.
      Motor1BackLeft.set(leftMotors); // setting speed of back left drive motor.
      Motor2FrontRight.set(rightMotors*-1); // setting speed of front right motor.
      Motor3BackRight.set(rightMotors*-1); // setting speed of back right motor.

      // robot motor diagram.
      // Front [N]
      // 0----2
      // |    |
      // |    |
      // 1----3

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
