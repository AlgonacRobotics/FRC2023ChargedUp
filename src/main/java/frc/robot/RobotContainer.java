// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.CommandGroups.AutonBlueLeftCommandGroup;
import frc.robot.commands.CommandGroups.AutonBlueMiddleCommandGroup;
import frc.robot.commands.CommandGroups.AutonBlueRightCommandGroup;
import frc.robot.commands.CommandGroups.AutonMiddleBalanceCommandGroup;
import frc.robot.commands.CommandGroups.AutonRedLeftCommandGroup;
import frc.robot.commands.CommandGroups.AutonRedMiddleCommandGroup;
import frc.robot.commands.CommandGroups.AutonRedRightCommandGroup;
import frc.robot.commands.CommandGroups.LeftAutonEnd;
import frc.robot.commands.CommandGroups.ScoreGroundCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHighCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeBlueLeftCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeBlueMiddleCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeBlueRightCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeMiddleBalanceCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeRedLeftCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeRedMiddleCommandGroup;
import frc.robot.commands.CommandGroups.ScoreHomeRedRightCommandGroup;
import frc.robot.commands.CommandGroups.ScoreMediumCommandGroup;
import frc.robot.commands.ElbowCommands.ElbowDownCommand;
import frc.robot.commands.ElbowCommands.ElbowGroundAutoCommand;
import frc.robot.commands.ElbowCommands.ElbowHalfLowerAutoCommand;
import frc.robot.commands.ElbowCommands.ElbowHighAutoCommand;
import frc.robot.commands.ElbowCommands.ElbowHomeAutoCommand;
import frc.robot.commands.ElbowCommands.ElbowMediumAutoCommand;
import frc.robot.commands.ElbowCommands.ElbowUpCommand;
import frc.robot.commands.ElevatorCommands.ElevatorDownCommand;
import frc.robot.commands.ElevatorCommands.ElevatorHighAutoCommand;
import frc.robot.commands.ElevatorCommands.ElevatorUpCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.GrabberCommands.GrabberIn;
import frc.robot.commands.GrabberCommands.GrabberOut;
import frc.robot.commands.ParallelCommandGroups.LeftAutonScoreSetUp;
import frc.robot.commands.ParallelCommandGroups.LeftAutonStartLower;
import frc.robot.commands.ParallelCommandGroups.LeftAutonStartScore;
import frc.robot.commands.ParallelCommandGroups.Test;
import frc.robot.commands.PneuCommands.CollectorClosedCommand;
import frc.robot.commands.PneuCommands.CollectorOpenCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.MK4RobotConfig;
import frc.robot.configs.SierraRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FanSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PneuSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private Drivetrain drivetrain;

  // Subsystems
  public static final PneuSubsystem m_pneuSubsystem = new PneuSubsystem();
  public static final ElbowSubsystem m_elbowSubsystem = new ElbowSubsystem();
  public static final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public static final FanSubsystem m_fanSubsystem = new FanSubsystem();
  public static final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
  // Commands
  // private final GrabberIn m_GrabberInCommand = new GrabberIn();
  private final GrabberIn m_GrabberInCommand = new GrabberIn();
  private final GrabberOut m_GrabberOutCommand = new GrabberOut();
  private final CollectorClosedCommand m_CollectorClosedCommand = new CollectorClosedCommand();
  private final CollectorOpenCommand m_CollectorOpenCommand = new CollectorOpenCommand();
  private final ElbowDownCommand m_ElbowDownCommand = new ElbowDownCommand();
  private final ElbowUpCommand m_ElbowUpCommand = new ElbowUpCommand();
  private final ElevatorDownCommand m_ElevatorDownCommand = new ElevatorDownCommand();
  private final ElevatorUpCommand m_ElevatorUpCommand = new ElevatorUpCommand();
  private final ElbowGroundAutoCommand m_ElbowGroundAutoCommand = new ElbowGroundAutoCommand(0);
  private final ElbowMediumAutoCommand m_ElbowMediumAutoCommand = new ElbowMediumAutoCommand(0);
  private final ElbowHighAutoCommand m_ElbowHighAutoCommand = new ElbowHighAutoCommand(0);
  private final ElbowHomeAutoCommand m_ElbowHomeAutoCommand = new ElbowHomeAutoCommand(0);
  private final ScoreGroundCommandGroup m_ScoreGroundCommandGroup = new ScoreGroundCommandGroup();
  private final ScoreHighCommandGroup m_ScoreHighCommandGroup = new ScoreHighCommandGroup();
  private final ScoreHomeCommandGroup m_ScoreHomeCommandGroup = new ScoreHomeCommandGroup();
  private final ScoreMediumCommandGroup m_ScoreMediumCommandGroup = new ScoreMediumCommandGroup();
  private final ElbowHalfLowerAutoCommand m_ElbowHalfLowerAutoCommand =
      new ElbowHalfLowerAutoCommand(0);
  private final ElevatorHighAutoCommand m_ElevatorHighAutoCommand = new ElevatorHighAutoCommand(0);
  private final AutonBlueLeftCommandGroup m_AutonBlueLeftCommandGroup =
      new AutonBlueLeftCommandGroup();
  private final AutonBlueMiddleCommandGroup m_AutonBlueMiddleCommandGroup =
      new AutonBlueMiddleCommandGroup();
  private final AutonMiddleBalanceCommandGroup m_AutonMiddleBalanceCommandGroup =
      new AutonMiddleBalanceCommandGroup();
  private final AutonBlueRightCommandGroup m_AutonBlueRightCommandGroup =
      new AutonBlueRightCommandGroup();
  private final AutonRedLeftCommandGroup m_AutonRedLeftCommandGroup =
      new AutonRedLeftCommandGroup();
  private final AutonRedMiddleCommandGroup m_AutonRedMiddleCommandGroup =
      new AutonRedMiddleCommandGroup();
  private final AutonRedRightCommandGroup m_AutonRedRightCommandGroup =
      new AutonRedRightCommandGroup();
  private final ScoreHomeBlueLeftCommandGroup m_ScoreHomeBlueLeftCommandGroup =
      new ScoreHomeBlueLeftCommandGroup();
  private final ScoreHomeBlueMiddleCommandGroup m_ScoreHomeBlueMiddleCommandGroup =
      new ScoreHomeBlueMiddleCommandGroup();
  private final ScoreHomeMiddleBalanceCommandGroup m_ScoreHomeMiddleBalanceCommandGroup =
      new ScoreHomeMiddleBalanceCommandGroup();
  private final ScoreHomeBlueRightCommandGroup m_ScoreHomeBlueRightCommandGroup =
      new ScoreHomeBlueRightCommandGroup();
  private final ScoreHomeRedLeftCommandGroup m_ScoreHomeRedLeftCommandGroup =
      new ScoreHomeRedLeftCommandGroup();
  private final ScoreHomeRedMiddleCommandGroup m_ScoreHomeRedMiddleCommandGroup =
      new ScoreHomeRedMiddleCommandGroup();
  private final ScoreHomeRedRightCommandGroup m_ScoreHomeRedRightCommandGroup =
      new ScoreHomeRedRightCommandGroup();
  private final LeftAutonScoreSetUp m_LeftAutonScoreSetUp = new LeftAutonScoreSetUp();
  private final LeftAutonStartLower m_LeftAutonStartLower = new LeftAutonStartLower();
  private final LeftAutonStartScore m_LeftAutonStartScore = new LeftAutonStartScore();
  private final Test m_Test = new Test();
  private final LeftAutonEnd m_LeftAutonEnd = new LeftAutonEnd();

  // Joysticks
  private final XboxController m_operatorController = new XboxController(Constants.operatorStick);

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();
  private final Map<String, Command> autoEventMap = new HashMap<>();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_DEFAULT:
          // case ROBOT_2023_MK4I:
        case ROBOT_2023_MK4:
        case ROBOT_2022_SIERRA:
          {
            // create the specific RobotConfig subclass instance first
            if (Constants.getRobot() == Constants.RobotType.ROBOT_2023_MK4) {
              config = new MK4RobotConfig();
            } else if (Constants.getRobot() == Constants.RobotType.ROBOT_2022_SIERRA) {
              config = new SierraRobotConfig();
            } else {
              config = new DefaultRobotConfig();
            }

            GyroIO gyro = new GyroIOPigeon2(config.getGyroCANID());

            int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
            int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
            int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
            double[] steerOffsets = config.getSwerveSteerOffsets();
            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        driveMotorCANIDs[0],
                        steerMotorCANDIDs[0],
                        steerEncoderCANDIDs[0],
                        steerOffsets[0]),
                    0,
                    config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        driveMotorCANIDs[1],
                        steerMotorCANDIDs[1],
                        steerEncoderCANDIDs[1],
                        steerOffsets[1]),
                    1,
                    config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        driveMotorCANIDs[2],
                        steerMotorCANDIDs[2],
                        steerEncoderCANDIDs[2],
                        steerOffsets[2]),
                    2,
                    config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        driveMotorCANIDs[3],
                        steerMotorCANDIDs[3],
                        steerEncoderCANDIDs[3],
                        steerOffsets[3]),
                    3,
                    config.getRobotMaxVelocity());

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            // new Pneumatics(new PneumaticsIORev());
            new Vision(new VisionIOPhotonVision(config.getCameraName()));
            break;
          }
        case ROBOT_SIMBOT:
          {
            config = new MK4RobotConfig();
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, config.getRobotMaxVelocity());
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            // new Pneumatics(new PneumaticsIO() {});
            AprilTagFieldLayout layout;
            try {
              layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }
            new Vision(
                new VisionIOSim(
                    layout,
                    drivetrain::getPose,
                    RobotConfig.getInstance().getRobotToCameraTransform()));

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, config.getRobotMaxVelocity());

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, config.getRobotMaxVelocity());

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, config.getRobotMaxVelocity());

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, config.getRobotMaxVelocity());
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      // new Pneumatics(new PneumaticsIO() {});
      new Vision(new VisionIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    configureAutoCommands();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // Open And Closed Collector
    new Trigger(m_operatorController::getAButton).whileTrue(m_GrabberInCommand);
    new Trigger(m_operatorController::getBButton).whileTrue(m_GrabberOutCommand);

    oi.getRobotNormalSpeedButton()
        .onTrue(Commands.runOnce(drivetrain::enableNormalSpeed, drivetrain));
    oi.getRobotSlowSpeedButton().onTrue(Commands.runOnce(drivetrain::enableSlowSpeed, drivetrain));

    // new
    // Trigger(m_operatorController::getAButton).onTrue(Commands.runOnce(m_pneuSubsystem::CollectorOpen, m_pneuSubsystem));
    // new
    // Trigger(m_operatorController::getAButton).onFalse(Commands.runOnce(m_pneuSubsystem::CollectorClosed, m_pneuSubsystem));

    // Raise and Lower Elevator
    new Trigger(m_operatorController::getRightBumper).whileTrue(m_ElevatorUpCommand);
    new Trigger(m_operatorController::getLeftBumper).whileTrue(m_ElevatorDownCommand);

    // Raise and Lower Elbow
    new Trigger(m_operatorController::getXButton).whileTrue(m_ElbowUpCommand);
    new Trigger(m_operatorController::getYButton).whileTrue(m_ElbowDownCommand);

    // automate elbow placement
    new Trigger(m_operatorController::getRightStickButton)
        .onTrue(m_ScoreGroundCommandGroup); // change back to m_ScoreGroundCommandGroup
    new Trigger(m_operatorController::getLeftStickButton).onTrue(m_ScoreMediumCommandGroup);
    // new Trigger(m_operatorController::getBackButton).onTrue(m_ScoreHighCommandGroup);
    new Trigger(m_operatorController::getStartButton).onTrue(m_ScoreHomeCommandGroup);
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));

    // build auto test path
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "testPaths1", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command autoTest =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), drivetrain, true),
                auto1Paths.get(0).getMarkers(),
                autoEventMap),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(1), drivetrain, false),
                auto1Paths.get(1).getMarkers(),
                autoEventMap));

    // build NewPath2023 Path
    List<PathPlannerTrajectory> autoNewPath2023 =
        PathPlanner.loadPathGroup(
            "NewPath2023", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command autoNewPath =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(autoNewPath2023.get(0), drivetrain, true),
                autoNewPath2023.get(0).getMarkers(),
                autoEventMap));
    /*  Commands.runOnce(drivetrain::enableXstance, drivetrain),
    Commands.waitSeconds(5.0),
    Commands.runOnce(drivetrain::disableXstance, drivetrain),

    new FollowPathWithEvents(
        new FollowPath(auto1Paths.get(1), drivetrain, false),
        auto1Paths.get(1).getMarkers(),
        autoEventMap));*/

    // BlueLeftPath
    List<PathPlannerTrajectory> blueLeftPath =
        PathPlanner.loadPathGroup(
            "BlueLeft", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command blueLeft =
        Commands.sequence(
            // m_ScoreHighCommandGroup,
          //  m_AutonBlueLeftCommandGroup,
            m_ScoreHomeBlueLeftCommandGroup,
            new FollowPathWithEvents(
                new FollowPath(blueLeftPath.get(0), drivetrain, true),
                blueLeftPath.get(0).getMarkers(),
                autoEventMap));
    // bluemid
    List<PathPlannerTrajectory> blueMiddlePath =
        PathPlanner.loadPathGroup(
            "BlueMiddle", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command blueMiddle =
        Commands.sequence(

            // m_ScoreHighCommandGroup,
            //m_AutonBlueMiddleCommandGroup,
            m_ScoreHomeBlueMiddleCommandGroup,
            new FollowPathWithEvents(
                new FollowPath(blueMiddlePath.get(0), drivetrain, true),
                blueMiddlePath.get(0).getMarkers(),
                autoEventMap));

    // MiddleBalance
    List<PathPlannerTrajectory> middleBalancePath =
        PathPlanner.loadPathGroup(
            "MiddleBalance", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command middleBalance =
        Commands.sequence(
            //   m_ScoreHighCommandGroup,
           // m_AutonMiddleBalanceCommandGroup,
            m_ScoreHomeMiddleBalanceCommandGroup,
            new FollowPathWithEvents(
                new FollowPath(middleBalancePath.get(0), drivetrain, true),
                middleBalancePath.get(0).getMarkers(),
                autoEventMap),
            new AutoBalance(drivetrain));

    // LeftDouble
    List<PathPlannerTrajectory> leftDoublePath =
        PathPlanner.loadPathGroup("LeftDouble", 0.5, config.getAutoMaxAcceleration());
    Command leftDouble =
        Commands.sequence(
            m_LeftAutonStartScore,
            Commands.waitSeconds(0.01),
            Commands.runOnce(m_grabberSubsystem::grabberOut, m_grabberSubsystem),
            Commands.waitSeconds(0.3),
            m_LeftAutonStartLower,
            Commands.waitSeconds(0.01),
            new FollowPathWithEvents(
                new FollowPath(leftDoublePath.get(0), drivetrain, true),
                leftDoublePath.get(0).getMarkers(),
                autoEventMap),
            Commands.waitSeconds(1),
            Commands.runOnce(m_grabberSubsystem::grabberIn, m_grabberSubsystem),
            Commands.waitSeconds(0.01),
            m_LeftAutonScoreSetUp,
            Commands.waitSeconds(0.01),
            new FollowPathWithEvents(
                new FollowPath(leftDoublePath.get(1), drivetrain, false),
                auto1Paths.get(1).getMarkers(),
                autoEventMap),
            m_LeftAutonEnd,
            Commands.runOnce(m_grabberSubsystem::grabberOut, m_grabberSubsystem));

    // RightDouble
    /*List<PathPlannerTrajectory> rightDoublePath =
        PathPlanner.loadPathGroup(
            "RightDouble", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command rightDouble =
        Commands.sequence(
            // m_LeftAutonStartScore,
            Commands.waitSeconds(0.01),
            Commands.runOnce(m_pneuSubsystem::CollectorOpen, m_pneuSubsystem),
            Commands.waitSeconds(0.3),
            // m_LeftAutonStartLower,
            Commands.waitSeconds(0.01),
            new FollowPathWithEvents(
                new FollowPath(rightDoublePath.get(0), drivetrain, true),
                rightDoublePath.get(0).getMarkers(),
                autoEventMap),
            Commands.runOnce(m_pneuSubsystem::CollectorClosed, m_pneuSubsystem),
            Commands.waitSeconds(0.01),
            // m_LeftAutonScoreSetUp,
            Commands.waitSeconds(0.01),
            new FollowPathWithEvents(
                new FollowPath(rightDoublePath.get(1), drivetrain, false),
                auto1Paths.get(1).getMarkers(),
                autoEventMap),
            // m_LeftAutonEnd,
            Commands.runOnce(m_pneuSubsystem::CollectorOpen, m_pneuSubsystem));*/

    // blueright
    List<PathPlannerTrajectory> blueRightPath =
        PathPlanner.loadPathGroup(
            "BlueRight", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command blueRight =
        Commands.sequence(
            // m_ScoreHighCommandGroup,
         //   m_AutonBlueRightCommandGroup,
            m_ScoreHomeBlueRightCommandGroup,
            new FollowPathWithEvents(
                new FollowPath(blueRightPath.get(0), drivetrain, true),
                blueRightPath.get(0).getMarkers(),
                autoEventMap));

    // redleft
    List<PathPlannerTrajectory> redLeftPath =
        PathPlanner.loadPathGroup(
            "RedLeft", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command redLeft =
        Commands.sequence(
            // m_ScoreHighCommandGroup,
           // m_AutonRedLeftCommandGroup,
            m_ScoreHomeRedLeftCommandGroup,
            new FollowPathWithEvents(
                new FollowPath(redLeftPath.get(0), drivetrain, true),
                redLeftPath.get(0).getMarkers(),
                autoEventMap));

    // redmid
    List<PathPlannerTrajectory> redMiddlePath =
        PathPlanner.loadPathGroup(
            "RedMiddle", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command redMiddle =
        Commands.sequence(
         //   m_AutonRedMiddleCommandGroup,
            m_ScoreHomeRedMiddleCommandGroup,
            new FollowPathWithEvents(
                new FollowPath(redMiddlePath.get(0), drivetrain, true),
                redMiddlePath.get(0).getMarkers(),
                autoEventMap));

    // redright
    List<PathPlannerTrajectory> redRightPath =
        PathPlanner.loadPathGroup(
            "RedRight", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command redRight =
        Commands.sequence(
            // m_ScoreHighCommandGroup,
          //  m_AutonRedRightCommandGroup,
            m_ScoreHomeRedRightCommandGroup,
            new FollowPathWithEvents(
                new FollowPath(redRightPath.get(0), drivetrain, true),
                redRightPath.get(0).getMarkers(),
                autoEventMap));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);

    // NewPath2023 Autonomous of PathPlanner path group with event markers
    autoChooser.addOption("NewPath2023", autoNewPath);

    // Autonomous for PathPlanner
    autoChooser.addOption("BlueLeft", blueLeft);
    autoChooser.addOption("BlueMiddle", blueMiddle);
    autoChooser.addOption("BlueRight", blueRight);
    autoChooser.addOption("RedLeft", redLeft);
    autoChooser.addOption("RedMiddle", redMiddle);
    autoChooser.addOption("RedRight", redRight);
    autoChooser.addOption("MiddleBalance", middleBalance);
    autoChooser.addOption("LeftDouble", leftDouble);
    // autoChooser.addOption("RightDouble", rightDouble);

    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0, false), drivetrain))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
