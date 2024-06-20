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
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.AngleCmd;
import frc.robot.commands.Collect;
import frc.robot.commands.GyroLimelight;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.AngleShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

public class RobotContainer {

  static final SwerveSubsystem swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"));
  public static final Intake subIntake = new Intake();
  public static final AngleShooter subAngle = new AngleShooter();
  public static final Shooter subShooter = new Shooter();

  public static final XboxController driverControl = new XboxController(
      Controle.xboxControle);
  public static final Joystick operatorControl = new Joystick(
      Controle.controle2);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    swerve.setDefaultCommand(
        new Teleop(
            swerve,
            () -> -MathUtil.applyDeadband(driverControl.getLeftY(), Controle.DEADBAND),
            () -> -MathUtil.applyDeadband(driverControl.getLeftX(), Controle.DEADBAND),
            () -> -MathUtil.applyDeadband(driverControl.getRightX(), Controle.DEADBAND),
            driverControl));

    autoChooser = AutoBuilder.buildAutoChooser();

    subAngle.setDefaultCommand(new AngleCmd(subAngle, operatorControl));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverControl, Button.kA.value)
        .onTrue(new InstantCommand(swerve::zeroGyro));

    new Trigger(this::getRight).onTrue(new ShootSpeaker(subShooter));

    new JoystickButton(operatorControl, XboxController.Button.kA.value).onTrue(new Collect(subShooter, subIntake))
        .onFalse(Commands.runOnce(() -> {
          subShooter.stopMotorConveyor();
          subIntake.stop();
        }, subIntake, subShooter));

    new JoystickButton(driverControl, XboxController.Button.kX.value).onTrue(new GyroLimelight(swerve, 0));

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
    // swerve.zeroGyro();
    // return autoChooser.getSelected();

    return swerve.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA, true, true);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

}
