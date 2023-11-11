// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoint;
import frc.robot.Constants.ElevatorConstants.GamePiece;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.choreolib.ChoreoSwerveControllerCommand;
import lib.choreolib.ChoreoTrajectory;
import lib.choreolib.TrajectoryManager;


public class Robot extends TimedRobot {

  //Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake();

  //Controllers
  private final CommandXboxController driveController = new CommandXboxController(DriverConstants.kDriverControllerPort);
  private final CommandGenericHID operatorController = new CommandGenericHID(DriverConstants.kOperatorControllerPort);

  ChoreoTrajectory BumpSide;
  ChoreoTrajectory Charge;
  ChoreoTrajectory SubSide;
  ChoreoTrajectory Test;
  ChoreoTrajectory BumpSideCube;

  SendableChooser<GamePiece> pieceChooser = new SendableChooser<GamePiece>();
  SendableChooser<ElevatorSetpoint> levelChooser = new SendableChooser<ElevatorSetpoint>();
  SendableChooser<ChoreoTrajectory> pathChooser = new SendableChooser<ChoreoTrajectory>();

  Field2d field = new Field2d();
  private final FieldObject2d[] modules = new FieldObject2d[4];


  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit(){

    ShuffleboardTab driverUITab = Shuffleboard.getTab("Driver UI");

    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.drive(
          -MathUtil.applyDeadband(driveController.getLeftY(), 0.05) * 0.25, 
          -MathUtil.applyDeadband(driveController.getLeftX(), 0.05) * 0.25, 
          -MathUtil.applyDeadband(driveController.getRightX(), 0.05) * 0.25, 
          true,
          RobotBase.isReal(),
          driveController.a().getAsBoolean()
        ),
        drivetrain
      )
    );


    //Driver Controls
    driveController.b().onTrue(
      Commands.runOnce(drivetrain::changeFeildRelative, drivetrain)
    );

    driveController.rightBumper().onTrue(
      Commands.runOnce(() -> intake.changeState(IntakeConstants.State.GRAB, elevator.getGamePiece(), elevator.getElevatorSetpoint()))
    );

    driveController.rightBumper().onFalse(
      Commands.runOnce(() -> intake.changeState(IntakeConstants.State.IDLE, elevator.getGamePiece(), elevator.getElevatorSetpoint()))
    );

    driveController.rightTrigger().onTrue(
      Commands.runOnce(() -> intake.changeState(IntakeConstants.State.RELEASE, elevator.getGamePiece(), elevator.getElevatorSetpoint()))
    );

    driveController.rightTrigger().onFalse(
      Commands.runOnce(() -> intake.changeState(IntakeConstants.State.IDLE, elevator.getGamePiece(), elevator.getElevatorSetpoint()))
    );

    operatorController.button(1).onTrue(
      Commands.runOnce(() -> elevator.changeSetpoint(ElevatorConstants.LEVEL1), elevator)
    );

    operatorController.button(2).onTrue(
      Commands.runOnce(() -> elevator.changeSetpoint(ElevatorConstants.LEVEL2), elevator)
    );

    operatorController.button(3).onTrue(
      Commands.runOnce(() -> elevator.changeSetpoint(ElevatorConstants.LEVEL3), elevator)
    );

    operatorController.button(4).onTrue(
      Commands.runOnce(() -> elevator.changeSetpoint(ElevatorConstants.DOUBLESUB), elevator)
    );

    operatorController.button(7).onTrue(
      Commands.runOnce(() -> elevator.changeSetpoint(ElevatorConstants.CARRY), elevator)
    );

    operatorController.button(8).onTrue(
      Commands.runOnce(() -> elevator.changeGamePiece(ElevatorConstants.CONE), elevator)
    );

    operatorController.button(9).onTrue(
      Commands.runOnce(() -> elevator.changeGamePiece(ElevatorConstants.CUBE), elevator)
    );

    //Autos
    TrajectoryManager.getInstance().LoadTrajectories();

    BumpSide = TrajectoryManager.getInstance().getTrajectory("Bump Side.json");
    BumpSideCube = TrajectoryManager.getInstance().getTrajectory("Bump Side Cube.json");
    Charge = TrajectoryManager.getInstance().getTrajectory("Charge.json");
    SubSide = TrajectoryManager.getInstance().getTrajectory("Sub Side.json");
    Test = TrajectoryManager.getInstance().getTrajectory("Y4.json");

    pathChooser.setDefaultOption("Bump Side", BumpSide);

    pathChooser.addOption("Bump Side Cube", BumpSideCube);
    pathChooser.addOption("Charge", Charge);
    pathChooser.addOption("Sub Side", SubSide);
    pathChooser.setDefaultOption("Test", Test);

    pieceChooser.setDefaultOption("Cone", ElevatorConstants.CONE);
    pieceChooser.addOption("Cube", ElevatorConstants.CUBE);

    levelChooser.setDefaultOption("Level 1", ElevatorConstants.LEVEL1);
    levelChooser.addOption("Level 2", ElevatorConstants.LEVEL2);
    levelChooser.addOption("Level 3",ElevatorConstants.LEVEL3);

    driverUITab.add("Auto Path", pathChooser).withSize(2, 1).withPosition(5, 0);
    driverUITab.add("Auto Game Piece", pieceChooser).withSize(2, 1).withPosition(5, 1);
    driverUITab.add("Auto Scoring Level", levelChooser).withSize(2, 1).withPosition(5, 2);

    for (int i = 0; i < modules.length; i++) {
      modules[i] = field.getObject("module-" + i);
    }

    SmartDashboard.putData(field);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    drivetrain.updatePoseEstimator();

    updateField2d();

    intake.log();
    elevator.log();
    drivetrain.log();

  }

  @Override
  public void simulationPeriodic() {
    drivetrain.simulate();
    elevator.simulate();
  }

  @Override
  public void autonomousInit(){

    boolean isBlue = DriverStation.getAlliance() == Alliance.Blue;

    var level = levelChooser.getSelected();

    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ChoreoSwerveControllerCommand swerveControllerCommand = 
      new ChoreoSwerveControllerCommand(
        pathChooser.getSelected(),
        drivetrain::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drivetrain::setModuleStates,
        true,
        drivetrain
      );

    Pose2d initialPose = isBlue ? pathChooser.getSelected().getInitialPose() : pathChooser.getSelected().mirrorred().getInitialPose();
    drivetrain.resetPose(initialPose);

    CommandScheduler.getInstance().schedule(
      Commands.sequence(
        Commands.runOnce(()-> intake.changeState(IntakeConstants.State.GRAB, elevator.getGamePiece(), elevator.getElevatorSetpoint()), intake),
        Commands.waitSeconds(1),
        Commands.runOnce(()-> intake.changeState(IntakeConstants.State.IDLE, elevator.getGamePiece(), elevator.getElevatorSetpoint()), intake),
        Commands.runOnce(() -> elevator.changeGamePiece(pieceChooser.getSelected()), elevator),
        Commands.runOnce(() -> elevator.changeSetpoint(level), elevator),
        Commands.waitUntil(elevator::atSetpoint),
        Commands.runOnce(() -> intake.changeState(IntakeConstants.State.RELEASE, elevator.getGamePiece(), elevator.getElevatorSetpoint()), intake),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> intake.changeState(IntakeConstants.State.IDLE, elevator.getGamePiece(), elevator.getElevatorSetpoint()), intake),
        Commands.runOnce(() -> elevator.changeSetpoint(ElevatorConstants.CARRY), elevator),
        Commands.waitUntil(elevator::atSetpoint),
        swerveControllerCommand,
        Commands.runOnce(() -> drivetrain.drive(0, 0, 0, true, true, false), drivetrain),
        Commands.runOnce(drivetrain::setX, drivetrain)
      )
    );

  }

  @Override
  public void autonomousPeriodic() {
    elevator.run();
    intake.run();
  }

  @Override
  public void teleopPeriodic() {
    elevator.run();
    intake.run();
  }

  private void updateField2d(){

    boolean isBlue = DriverStation.getAlliance() == Alliance.Blue;

    field.setRobotPose(drivetrain.getPose());
      for (int i = 0; i < 4; i++) {
        var transform = new Transform2d(DriveConstants.kModuleOffset[i], drivetrain.getModuleStates()[i].angle);
        modules[i].setPose(drivetrain.getPose().transformBy(transform));
      }


    field.getObject("traj").setPoses(
      isBlue ? pathChooser.getSelected().getInitialPose() : pathChooser.getSelected().mirrorred().getInitialPose(),
      isBlue ? pathChooser.getSelected().getFinalPose() : pathChooser.getSelected().mirrorred().getFinalPose()
    );

    field.getObject("trajPoses").setPoses(
      isBlue ? pathChooser.getSelected().getPoses() : pathChooser.getSelected().mirrorred().getPoses()
    );

  }

}
