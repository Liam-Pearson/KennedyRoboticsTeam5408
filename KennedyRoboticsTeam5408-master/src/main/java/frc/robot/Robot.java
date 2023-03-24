// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
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
  // pulley system motor.
  private Talon Motor4Pulley = new Talon(4); // pulley system drive motor.
  // telescoping arm motors.
  private Talon Motor5Telescope = new Talon(5); // left telescoping arm motor.

  private static SPI.Port gyroPort = SPI.Port.kOnboardCS0;

  // solenoid object decleration. PCMConeGrabber is what grabs the game cones.
  DoubleSolenoid PCMConeGrabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  // built in RoboRio Accelerometer.
  Accelerometer accelerometer = new BuiltInAccelerometer();

  // gryoscope.
  ADXRS450_Gyro gyro = new ADXRS450_Gyro(gyroPort);

  // init of controllers/joysticks ([Name] = new [Type]([USBPort]))
  private Joystick joystick = new Joystick(0);

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

    PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward. Closes the claw.

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

  // all pretty self explainitory.
  double joystickXAxisPos;
  double joystickYAxisPos;
  double joystickZAxisPos; // for turning in place. Currently not in use.
  // speedAxis is controlled by the scroller on the joystick.
  double joystickSpeedAxisPos;

  // drive motor control.
  // double deadzoneLimit = 0.4; // sets the value that the joystick position must exceed for movement of the robot to occur.
  double speed = 0; // speed of robot.

  // variables for drive motors (non-differental, hard coded solution version).
  double leftMotors;
  double rightMotors;

  // handles all operations regarding controller-operated driving.
  public void controllerDrive(double deadzone, double speedCoefficent) {
    
    joystickXAxisPos = joystick.getRawAxis(0); // x axis on joystick
    joystickYAxisPos = joystick.getRawAxis(1); // y axis on joystick
    joystickZAxisPos = joystick.getRawAxis(2); // z axis on joystick
    joystickSpeedAxisPos = joystick.getRawAxis(3); // y axis on joystick

    // prints the current position of the joystick.
    // System.out.println("X-POS: " + joystickXAxisPos + ".");
    // System.out.println("Y-POS: " + joystickYAxisPos + ".");

    // deadzones.
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

    // speed control of robot. Hopefully this works tomorrow.
    if(joystickSpeedAxisPos <= 1 && joystickSpeedAxisPos <= 0){
      speed = ((1 - joystickSpeedAxisPos) * speedCoefficent); // sets the speed of the robot by dividing the motor speed by the speed value.
    } // end else if.
    else if(joystickSpeedAxisPos < 0){
      speed = ((1 + (joystickSpeedAxisPos*-1)) * speedCoefficent); // sets the speed of the robot by dividing the motor speed by the speed value.
    } // end else if.
    else{
      speed = 0;
    } // end else.

    // System.out.println("SPEED SLIDER VALUE: " + joystickSpeedAxisPos + ".");
    // System.out.println("SPEED: " + speed + ".");

    // setting speed of left and right motors.
    leftMotors = (joystickYAxisPos - joystickXAxisPos)*-1; // left motors are postive values. The value is then divided to set speed.
    rightMotors = (joystickYAxisPos + joystickXAxisPos)*-1; // right motors are negative values. The value is then divided to set speed.

    // System.out.println("LEFT MOTORS: " + leftMotors + ".");
    // System.out.println("RIGHT MOTORS: " + rightMotors + ".");

    // motors are inverted depending on the side. To move forward, left motors must
    // be positive values, while right side motors must be set to negative values,
    // or vice-versa.
    // motors are totally weird and wrong due to wiring and positioning but they
    // work.
    Motor0FrontLeft.set(leftMotors * -1); // setting speed of front left drive motor.
    Motor1BackLeft.set(leftMotors); // setting speed of back left drive motor.
    Motor2FrontRight.set(rightMotors * -1); // setting speed of front right motor.
    Motor3BackRight.set(rightMotors * -1); // setting speed of back right motor.

    // robot motor diagram.
    // Front [N]
    // 0----2
    // | |
    // | |
    // 1----3
  } // end controllerDrive.

  // handles all operations regarding lifting of the arm.
  public void armLifter(double armLiftUpSpeed, double armLiftDownSpeed, double armConstantBreakingSpeed) {

    // controls the speed of the arm pulley system
    if (joystick.getRawButton(1)) {
      Motor4Pulley.set(armLiftUpSpeed);
      // System.out.println("ARM MOVING UP.");
      // motorPullLengthIndex = motorPullLengthIndex + 0.01;
    } // end if.
    else if (joystick.getRawButton(2)) {
      Motor4Pulley.set(armLiftDownSpeed*-1); // retract arm.
      // System.out.println("ARM MOVING DOWN.");
    } // end else if.
    else {
      Motor4Pulley.set(armConstantBreakingSpeed);
      // System.out.println("ARM NOT MOVING.");
    } // end else.
  } // end armExtender.

  // handles all operations regarding extending of the arm.
  public void armExtender(double telescopeOutSpeed, double telescopeInSpeed, double telescopeConstantBreakSpeed) {
    // controls the speed of the telescoping arm pulley system.
    if (joystick.getRawButton(5)) {
      Motor5Telescope.set(telescopeOutSpeed);
      // System.out.println("TELESCOPING ARM OUT.");
    } // end if.
    else if (joystick.getRawButton(6)) {
      Motor5Telescope.set(telescopeInSpeed*-1); // retract telescoping arm.
      // System.out.println("TELESCOPING ARM IN.");
    } // end else if.
    else {
      Motor5Telescope.set(telescopeConstantBreakSpeed);
    } // end else.
  } // end armExtender.

  boolean clawToggle = false; // claw on or off.

  // handles all operations regarding opening and closing of the claw.
  public void clawOperations() {
    // if button three is pressed, clawToggle will be set to true, thus closing the
    // claw inward.
    if (joystick.getRawButton(3)) {
      clawToggle = true;
      PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward.
      // System.out.println("CLAW CLOSE.");
    } // end if.

    // if button 4 is pressed, clawToggle will be set to false.
    if (joystick.getRawButton(4)) {
      clawToggle = false;
      // System.out.println("CLAW OPEN.");
      PCMConeGrabber.set(DoubleSolenoid.Value.kReverse); // sets the solenoid to forward.

    } // end if.

    // checks the condition of clawToggle. If clawToggle is true, close the claw.
    if (clawToggle == true) {
      PCMConeGrabber.set(DoubleSolenoid.Value.kForward); // sets the solenoid to forward.
      // System.out.println("SOLENOID ACTIVATED");
    } // end if.

    // PCMConeGrabber.set(DoubleSolenoid.Value.kReverse);
  } // end clawOperations

  // accerlation variables.
  // =====================================================================
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
  // =====================================================================

  // handles all operations regarding determining the position of the robot in 2D
  // space.
  public void robotPosition() {
    // System.out.println("ROBOT POSITION RUNNING.");
    // documentation can be found at
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/accelerometers-software.html#builtinaccelerometer.

    // gets the start time of the calculation.
    startTimeOfPosition = System.currentTimeMillis(); // t1

    // gets the current acceleration in the x and y directions.
    xAccel = AccelFilter.calculate(accelerometer.getX());
    yAccel = AccelFilter.calculate(accelerometer.getY());

    // calcualtes the jerk of the robot. Not used for anything.
    xJerk = (xAccel - prevXAccel) / .02;
    yJerk = (yAccel - prevYAccel) / .02;

    // sets the previous acceleration valeus to the current acceleration values.
    prevXAccel = xAccel;
    prevYAccel = yAccel;

    // prints acceleration values;
    // System.out.println("X-ACCEL: " + xAccel + ". Y-ACCEL " + yAccel + ".");

    // gets the end time of the calculation.
    endTimeOfPosition = System.currentTimeMillis(); // t2.

    // calculates the change in time during the calculations for position. This
    // would be the Δt variable. [Δt = t2-t1].
    changeInTimeForPosition = endTimeOfPosition - startTimeOfPosition; // Δt
    // System.out.println("CHANGE IN TIME FOR POSITION: " + changeInTimeForPosition + ".");

    // calculates the velocity of the robot in each axis.
    xVelocity = (xAccel / changeInTimeForPosition);
    yVelocity = (yAccel / changeInTimeForPosition);

    // prints velocity values.
    // System.out.println("X-VELOCITY: " + xVelocity + ". Y-VELOCITY: " + yVelocity + ".");

    // calculates position using kinematics. [Δd = (v1)(Δt) + (0.5)(a)(Δt^2)] -
    // formual for change in distance/position.
    xPositionOfRobot = ((xVelocity * changeInTimeForPosition) + (0.5 * xAccel * (changeInTimeForPosition * changeInTimeForPosition))); // Δdx
    yPositionOfRobot = ((yVelocity * changeInTimeForPosition) + (0.5 * yAccel * (changeInTimeForPosition * changeInTimeForPosition))); // Δdy

    // prints position value and gyro value.
    // System.out.println("X-POSITION: " + xPositionOfRobot + " Y-POSITION: " + yPositionOfRobot + ".");
    // System.out.println("GYRO: " + Math.round(gyro.getAngle()));
  } // end robotPosition.

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  Timer t = new Timer();

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
      //m_autonomousCommand.schedule();
    } // end if.

    t.reset();
    t.start();
  } // end autonomousInit.

  long timeOfAutonomous = 0;

  boolean autoCorrectPositionAutonomous = true;

  double driveSpeed = 0;

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    long autoTimer = System.currentTimeMillis();

    timeOfAutonomous = autoTimer - masterStartTime;

    // if less than 3 seconds has passed, drive the robot forward. This is extremely
    // basic (and relatively useless) autonomous code.
    if (timeOfAutonomous < 2000) {
      System.out.println("TIME OF AUTONOMOUS = " + timeOfAutonomous + ".");

      // Motor0FrontLeft.set(0.4 * -1); // setting speed of front left drive motor.
      // Motor1BackLeft.set(0.4); // setting speed of back left drive motor.
      // Motor2FrontRight.set(0.4*-1); // setting speed of front right motor.
      // Motor3BackRight.set(0.4*-1); // setting speed of back right motor.

      System.out.println("AUTONOMOUS RUNNING AND MOVING FORWARDS.");

      System.out.println("GYRO: " + gyro.getAngle());

      // courseCorrectionAutonomous(10,10);

      // robot motor diagram.
      // Front [N]
      // 0----2
      // | |
      // | |
      // 1----3

      // arcade drive. Not sure how this works.
      // driveCG.arcadeDrive(0.2, 0.2);

      // PCMConeGrabber.set(DoubleSolenoid.Value.kReverse); // sets the solenoid to reverse`. Drops the cube on the ground.

    } // end if.
    else{
      Motor0FrontLeft.set(0); // setting speed of front left drive motor.
      Motor1BackLeft.set(0); // setting speed of back left drive motor.
      Motor2FrontRight.set(0); // setting speed of front right motor.
      Motor3BackRight.set(0); // setting speed of back right motor.
    }

  } // end autonomousPeriodic.

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
      controllerDrive(0.2, 8.9); // allows the robot to drive.
      armLifter(0.55, 0.15, 0.1); // allows the arm of the robot to lift.
      armExtender(0.6, 0.15, 0.1); // alows the arm to extend.
      clawOperations(); // allows the claw to open and shut.
      robotPosition(); // allows the robot to calculate its position.
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
}
