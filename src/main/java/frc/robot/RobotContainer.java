/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.ExampleCommand;<<<<<<<HEAD
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NormalDrive;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.moveHowitzer;
import frc.robot.subsystems.BallStopperSystem;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HowitzerSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final BallStopperSystem ballStopperSystem = new BallStopperSystem();
  private final ClimbSystem climbSystem = new ClimbSystem();
  private final ColorWheelSystem colorWheelSystem = new ColorWheelSystem();
  private final ConveyorSystem conveyorSystem = new ConveyorSystem();
  private final DriveSystem driveSystem = new DriveSystem();
  private final HowitzerSystem howitzerSystem = new HowitzerSystem();
  private final IntakeSystem intakeSystem = new IntakeSystem();
  private final ShooterSystem shooterSystem = new ShooterSystem();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(ControllerMap.operator, ControllerMap.A).whenHeld( new IntakeCommand(intakeSystem));
    new JoystickButton(ControllerMap.operator, ControllerMap.LB).whenHeld(new AimCommand(howitzerSystem, driveSystem));
    new JoystickButton(ControllerMap.operator, ControllerMap.RB).whenHeld(new ConveyorCommand(conveyorSystem));

    new JoystickButton(ControllerMap.operator, ControllerMap.B).whenHeld(new ShootCommand(shooterSystem, shooterSystem.topWheel(), .8));
    new JoystickButton(ControllerMap.operator, ControllerMap.B).whenHeld(new ShootCommand(shooterSystem, shooterSystem.bottomWheel(), .8));
    
    new JoystickButton(ControllerMap.operator, ControllerMap.X).whenPressed(new moveHowitzer(howitzerSystem, 5));
    new JoystickButton(ControllerMap.operator, ControllerMap.Y).whenPressed(new moveHowitzer(howitzerSystem, -5));
    driveSystem.setDefaultCommand(new ArcadeDrive(driveSystem, ControllerMap.driver).perpetually());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
  }
}
