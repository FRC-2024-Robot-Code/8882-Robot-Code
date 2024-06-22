// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controle;
import frc.robot.commands.AngleRelative;
import frc.robot.commands.Collect;
import frc.robot.commands.Teleop;
import frc.robot.commands.Shoots.Amp;
import frc.robot.commands.Shoots.Speaker;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import java.io.File;

public class RobotContainer {

  static final Swerve robot = new Swerve(
      new File(Filesystem.getDeployDirectory(), "swerve"));
  public static final Intake intake = new Intake();
  public static final Angle angle = new Angle();
  public static final Shooter shooter = new Shooter();

  public static final XboxController driverControl = new XboxController(
      Controle.xboxControle);
  public static final Joystick operatorControl = new Joystick(
      Controle.controle2);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    robot.setDefaultCommand(
        new Teleop(
            robot,
            () -> MathUtil.applyDeadband(driverControl.getLeftY(), Controle.DEADBAND),
            () -> MathUtil.applyDeadband(driverControl.getLeftX(), Controle.DEADBAND),
            () -> MathUtil.applyDeadband(driverControl.getRightX(), Controle.DEADBAND),
            driverControl));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

  }

  private void configureBindings() {

    new JoystickButton(driverControl, XboxController.Button.kX.value)
        .onTrue(Commands.run(() -> {
          robot.aimToSpeakerBlue();
        }, robot));

    new JoystickButton(driverControl, XboxController.Button.kX.value).onFalse(new Teleop(
        robot,
        () -> MathUtil.applyDeadband(driverControl.getLeftY(), Controle.DEADBAND),
        () -> MathUtil.applyDeadband(driverControl.getLeftX(), Controle.DEADBAND),
        () -> MathUtil.applyDeadband(driverControl.getRightX(), Controle.DEADBAND),
        driverControl));

    new JoystickButton(driverControl, Button.kA.value)
        .onTrue(new InstantCommand(robot::zeroGyro));

    new JoystickButton(operatorControl, XboxController.Button.kA.value).onTrue(new Collect(intake, shooter))
        .onFalse(Commands.runOnce(() -> {
          shooter.stopConveyor();
          intake.stopIntake();
        }, shooter, intake));
    new JoystickButton(operatorControl, XboxController.Button.kA.value).onTrue(new AngleRelative(angle, -15));
    // new JoystickButton(operatorControl,
    // XboxController.Button.kX.value).onTrue(new AngleRelative(angle,
    // angle.getAngle()));

    new Trigger(this::getRight).onTrue(new Speaker(shooter, 0.55, 0.6, 1, 1));
    new Trigger(this::getLeft).onTrue(new Amp(shooter));
    // new Trigger(this::getLeft).onTrue(new AngleRelative(angle, -20));

  }

  private boolean getRight() {
    if (operatorControl.getRawAxis(Controle.rightTrigger) != 0) {
      return true;
    }
    return false;
  }

  private boolean getLeft() {
    if (operatorControl.getRawAxis(Controle.leftTrigger) != 0) {
      return true;
    }
    return false;

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    // return swerve.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA1, true, true);
  }

  public void setMotorBrake(boolean brake) {
    robot.setMotorBrake(brake);
  }

  public void setHeadingCorrection(boolean headingCorrection) {
    robot.swerveDrive.setHeadingCorrection(headingCorrection);
  }

}
