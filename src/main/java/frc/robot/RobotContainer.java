/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BallHandler;
import frc.robot.commands.CW_ColorCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ColorWheelRotationCommand;
import frc.robot.commands.FollowPath;
import frc.robot.commands.GetBall;
import frc.robot.commands.IntakeBackward;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TestDriveMotorsCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HowitzerSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.PowerMonitor;
import frc.robot.subsystems.ShooterSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ClimbSystem climbSystem = new ClimbSystem();
  private final ColorWheelSystem colorWheelSystem = new ColorWheelSystem();
  private final ConveyorSystem conveyorSystem = new ConveyorSystem();
  private final DriveSystem driveSystem = new DriveSystem();
  public HowitzerSystem howitzerSystem;
  private final IntakeSystem intakeSystem = new IntakeSystem();
  private final ShooterSystem shooterSystem = new ShooterSystem();
  //private final PowerMonitor powerMonitor = new PowerMonitor();

  private final BallHandler ballHandler = new BallHandler();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  // TODO make power budget
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() { // TODO collision avoidance
    FindPath.config(3, .5f, 0, 1.88797f);

    // ---------- DRIVER ----------
    ballHandler.perpetually();
    JoystickButton aim = new JoystickButton(ControllerMap.driver, ControllerMap.LB);
    aim.whileHeld(new TurnCommand(driveSystem, () -> Robot.horizontalOffset.getDouble(0)));
    JoystickButton ball = new JoystickButton(ControllerMap.driver, ControllerMap.RB);
    ball.whileHeld(new GetBall(driveSystem, ballHandler));

    driveSystem.setDefaultCommand(new ArcadeDrive(driveSystem, ControllerMap.driver).perpetually());

    new JoystickButton(ControllerMap.driver, ControllerMap.X).whileHeld(new IntakePivotCommand(intakeSystem, -.5));

    // ---------- IN FLUX -----------
    new JoystickButton(ControllerMap.driver, ControllerMap.A).whenHeld(new IntakeCommand(intakeSystem, conveyorSystem));//TODO automate intake
    JoystickButton intakeBackward = new JoystickButton(ControllerMap.driver, ControllerMap.B);
    intakeBackward.whileHeld(new IntakeBackward(conveyorSystem, shooterSystem, intakeSystem));
    // ---------- OPERATOR ----------

    howitzerSystem = new HowitzerSystem(ControllerMap.operator, conveyorSystem);

    new JoystickButton(ControllerMap.operator, ControllerMap.RB)
        .whileHeld(new ClimbCommand(climbSystem, ControllerMap.operator));

    JoystickButton shooterForward = new JoystickButton(ControllerMap.operator, ControllerMap.B);
    shooterForward.whileHeld(new ShootCommand(shooterSystem, conveyorSystem, Constants.shooterSpeedT, Constants.shooterSpeedB));

    new AimCommand(howitzerSystem, driveSystem).perpetually();

    new JoystickButton(ControllerMap.operator, ControllerMap.start)
        .whenPressed(new ColorWheelRotationCommand((colorWheelSystem)).andThen(new CW_ColorCommand(colorWheelSystem)));
    
    new DPad(ControllerMap.operator, DPad.Direction.UP).whenPressed(() ->
        howitzerSystem.addOffset());
    new DPad(ControllerMap.operator, DPad.Direction.DOWN).whenPressed(() ->
        howitzerSystem.subOffset());

    new DPad(ControllerMap.operator, DPad.Direction.RIGHT).whenPressed(() ->
        driveSystem.addOffset());
    new DPad(ControllerMap.operator, DPad.Direction.LEFT).whenPressed(() ->
        driveSystem.subOffset());
  }

  public boolean doubleButton(JoystickButton a, JoystickButton b) {
    return a.get() || b.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new FollowPath(driveSystem, FindPath.getStraight(2));
    FindPath.config(3, 3, 0, (float)Constants.trackWidth);
    return new TurnCommand(driveSystem, () -> 90);
    // An ExampleCommand will run in autonomous
  }
}