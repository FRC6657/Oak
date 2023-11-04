// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoint;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.drivetrain.Drivetrain;
import lib.choreolib.ChoreoSwerveControllerCommand;
import lib.choreolib.ChoreoTrajectory;
import lib.choreolib.TrajectoryManager;


public class Robot extends TimedRobot {

  //Subsystems
  private final Drivetrain drivetrain = new Drivetrain();

  //Command Factory
  private final CommandFactory commandFactory = new CommandFactory(drivetrain);

  //Controllers
  private final CommandXboxController driveController = new CommandXboxController(DriverConstants.kDriverControllerPort);

  ChoreoTrajectory BumpSide;
  ChoreoTrajectory Charge;
  ChoreoTrajectory SubSide;
  ChoreoTrajectory Test;

  SendableChooser<Boolean> pieceChooser = new SendableChooser<Boolean>();
  SendableChooser<ElevatorSetpoint> levelChooser = new SendableChooser<ElevatorSetpoint>();
  SendableChooser<ChoreoTrajectory> pathChooser = new SendableChooser<ChoreoTrajectory>();

  Field2d field = new Field2d();
  private final FieldObject2d[] modules = new FieldObject2d[4];

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit(){

    //Set the default command for the drivetrain to allow for teleop control
    drivetrain.setDefaultCommand(
      commandFactory.TeleopSwerve(
        () -> driveController.getLeftY(),
        () -> driveController.getLeftX(),
        () -> driveController.getRightX()
      )
    );

    TrajectoryManager.getInstance().LoadTrajectories();

    BumpSide = TrajectoryManager.getInstance().getTrajectory("Bump Side.json");
    Charge = TrajectoryManager.getInstance().getTrajectory("Charge.json");
    SubSide = TrajectoryManager.getInstance().getTrajectory("Sub Side.json");
    Test = TrajectoryManager.getInstance().getTrajectory("Test.json");

    pathChooser.setDefaultOption("Bump Side", BumpSide);
    pathChooser.addOption("Charge", Charge);
    pathChooser.addOption("Sub Side", SubSide);
    pathChooser.addOption("Test", Test);

    pieceChooser.setDefaultOption("Cone", true);
    pieceChooser.addOption("Cube", false);

    levelChooser.setDefaultOption("Level 1", ElevatorConstants.LEVEL1);
    levelChooser.addOption("Level 2", ElevatorConstants.LEVEL2);
    levelChooser.addOption("Level 3",ElevatorConstants.LEVEL3);
    
    SmartDashboard.putData(pathChooser);
    SmartDashboard.putData(pieceChooser);
    SmartDashboard.putData(levelChooser);
    

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

  }

  @Override
  public void simulationPeriodic() {
    drivetrain.simulate();
  }

  @Override
  public void autonomousInit(){

    boolean isBlue = DriverStation.getAlliance() == Alliance.Blue;

    var level = levelChooser.getSelected();
    double elevatorSetpoint = pieceChooser.getSelected() ? level.cone : level.cube;

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
        // Commands.runOnce(() -> elevator.setHeight(elevatorSetpoint), elevator),
        // Commands.waitUntil(elevator::atSetpoint),
        // Commands.runOnce(() -> intake.setSpeed(0.5), intake),
        // Commands.waitSeconds(0.5),
        // Commands.runOnce(() -> intake.setSpeed(0), intake),
        // Commands.runOnce(() -> elevator.setHeight(ElevatorConstants.CARRY), elevator),
        // Commands.waitUntil(elevator::atSetpoint),
        swerveControllerCommand,
        Commands.runOnce(() -> drivetrain.drive(0, 0, 0, true, true), drivetrain),
        Commands.runOnce(drivetrain::setX, drivetrain)
      )
    );


    // if(autonomousCommand != null){
    //   CommandScheduler.getInstance().schedule(autonomousCommand);
    // }
  }

  @Override
  public void teleopInit() {
    // if (autonomousCommand != null) {
    //   autonomousCommand.cancel();
    // }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
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
