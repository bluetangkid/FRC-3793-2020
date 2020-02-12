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
import frc.robot.commands.CW_ColorCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ColorWheelCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DisablePID;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.moveToHowitzer;
import frc.robot.subsystems.BallStopperSystem;
import frc.robot.subsystems.ClimbSystem;
import frc.robot.subsystems.ColorWheelSystem;
import frc.robot.subsystems.ConveyorSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.HowitzerSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final BallStopperSystem ballStopperSystem = new BallStopperSystem();
  private final ClimbSystem climbSystem = new ClimbSystem();
  private final ColorWheelSystem colorWheelSystem = new ColorWheelSystem();
  private final ConveyorSystem conveyorSystem = new ConveyorSystem();
  private final DriveSystem driveSystem = new DriveSystem();
  private final HowitzerSystem howitzerSystem = new HowitzerSystem();
  private final IntakeSystem intakeSystem = new IntakeSystem();
  private final ShooterSystem shooterSystem = new ShooterSystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
//TODO enable current limits for all motors and make power budget
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton aim = new JoystickButton(ControllerMap.driver, ControllerMap.LB);
    driveSystem.setDefaultCommand(new ConditionalCommand(new AimCommand(howitzerSystem, driveSystem), new ArcadeDrive(driveSystem, ControllerMap.driver), aim::get));

    JoystickButton climb = new JoystickButton(ControllerMap.driver, ControllerMap.RB);
    ConditionalCommand climbCommand = new ConditionalCommand(new ClimbCommand(climbSystem, 1), new ClimbCommand(climbSystem, -1), climb::get);
    climbCommand.initialize();

    new JoystickButton(ControllerMap.operator, ControllerMap.back).whenPressed(new ColorWheelCommand(colorWheelSystem).andThen(new CW_ColorCommand(colorWheelSystem)));

    new JoystickButton(ControllerMap.operator, ControllerMap.A).whenHeld(new IntakeCommand(intakeSystem, ballStopperSystem).alongWith(new ConveyorCommand(conveyorSystem, shooterSystem, ballStopperSystem)));
    
    JoystickButton shooter = new JoystickButton(ControllerMap.operator, ControllerMap.B);
    Command top = new ShootCommand(shooterSystem.topWheel(), Constants.shooterSpeed).andThen(new DisablePID(shooterSystem.topWheel()));
    Command bottom = new ShootCommand(shooterSystem.bottomWheel(), Constants.shooterSpeed).andThen(new DisablePID(shooterSystem.bottomWheel()));
    shooter.whenHeld(top.alongWith(bottom));
    
    new JoystickButton(ControllerMap.operator, ControllerMap.X).whenPressed(new moveToHowitzer(howitzerSystem, 5));//these shouldn't move it, just add an offset to the target that aimcommand goes to
    new JoystickButton(ControllerMap.operator, ControllerMap.Y).whenPressed(new moveToHowitzer(howitzerSystem, -5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // An ExampleCommand will run in autonomous
  }
}
