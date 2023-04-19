// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.chassisConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController driver = new XboxController(0);
  private JoystickButton kA;
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private Trajectory path1 = new Trajectory();

  //private final SwerveJoystickCmd m_autoCommand = new SwerveJoystickCmd(swerveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = false;
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, driver, fieldRelative));
    // Configure the button bindings
    configureButtonBindings();
    //kA.whileHeld(new AlignRobot(swerveSubsystem, SwerveSubsystem.tracker, fieldRelative));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    kA = new JoystickButton(driver, 1);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    PathPlannerTrajectory pathGroup1 = PathPlanner.loadPath("PathGroup1", new PathConstraints(4, 3));

    return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry(pathGroup1.getInitialHolonomicPose())), 
    new PPSwerveControllerCommand(
      pathGroup1,
      swerveSubsystem::getPose,
      chassisConstants.swerveKinematics,
      new PIDController(0.1, 0, 0),
      new PIDController(0.1, 0, 0),
      new PIDController(0.0001, 0, 0),
      swerveSubsystem::setModuleStates,
      swerveSubsystem
    ), new InstantCommand(() -> swerveSubsystem.stopModules()));
    //return null;
  }
}
