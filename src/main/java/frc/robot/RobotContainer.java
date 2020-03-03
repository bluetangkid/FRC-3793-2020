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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TestDriveMotorsCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HowitzerSystem;
import frc.robot.subsystems.IntakeSystem;
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
    howitzerSystem = new HowitzerSystem(new JoystickButton(ControllerMap.operator, ControllerMap.X),
        new JoystickButton(ControllerMap.operator, ControllerMap.Y));
    //ur gonna have to write a turnTo limemight thing whore <3
    //ballHandler.perpetually();
    // JoystickButton aim = new JoystickButton(ControllerMap.driver,
    // ControllerMap.LB);
    // JoystickButton ball = new JoystickButton(ControllerMap.driver,
    // ControllerMap.RB);
    // Command aimTarget = new TurnCommand(driveSystem, () ->
    // Command getBall = new GetBall(driveSystem, ballHandler);
    driveSystem.setDefaultCommand(new ArcadeDrive(driveSystem, ControllerMap.driver).perpetually());
    // new ConditionalCommand(new ConditionalComm 1and(aimTarget, getBall, aim::get),
    //    new ArcadeDrive(driveSystem, ControllerMap.driver), () -> doubleButton(aim, ball)).perpetually();

    new JoystickButton(ControllerMap.operator, ControllerMap.RB).whileHeld(new ClimbCommand(climbSystem, -1));
    new JoystickButton(ControllerMap.operator, ControllerMap.LB).whileHeld(new ClimbCommand(climbSystem, 1));

    //new JoystickButton(ControllerMap.operator, ControllerMap.back)
    //    .whenPressed(new ColorWheelRotationCommand(colorWheelSystem).andThen(new CW_ColorCommand(colorWheelSystem)));

    new JoystickButton(ControllerMap.operator, ControllerMap.A).whenHeld(new IntakeCommand(intakeSystem, conveyorSystem));

    JoystickButton shooter = new JoystickButton(ControllerMap.operator, ControllerMap.B);
    Command shoot = new ShootCommand(shooterSystem, conveyorSystem);
    shooter.whileHeld(shoot);

    new AimCommand(howitzerSystem, driveSystem).perpetually();

    new JoystickButton(ControllerMap.operator, ControllerMap.start).whenPressed(new ColorWheelRotationCommand((colorWheelSystem)).andThen(new CW_ColorCommand(colorWheelSystem)));
    // just use lambdas to do the howitzer angle stuff like below
    //new JoystickButton(ControllerMap.operator, ControllerMap.X).whenPressed(() -> howitzerSystem.addOffset());
    //new JoystickButton(ControllerMap.operator, ControllerMap.Y).whenPressed(() -> howitzerSystem.subOffset());

    //new JoystickButton(ControllerMap.driver, ControllerMap.A).whenPressed(() -> driveSystem.subOffset());
    //new JoystickButton(ControllerMap.driver, ControllerMap.B).whenPressed(() -> driveSystem.addOffset());
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
    FindPath.config(3, .5f, 0, 1.88797f);
    //return new FollowPath(driveSystem, FindPath.getStraight(2)); //straight don't work don't even thing about it whore
    return new TurnCommand(driveSystem, () -> 90);
    // An ExampleCommand will run in autonomous
  }
}