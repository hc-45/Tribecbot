package com.stuypulse.robot.util.simulation;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.utils.FieldMirroringUtils;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.robot.commands.auton.regular.DepotAuton;
import com.stuypulse.robot.constants.DriverConstants.Driver.Drive;
import com.stuypulse.robot.constants.DriverConstants.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public class OtherRobotInSimulation implements Subsystem {
	private static int topId = 0;
	public static final Twist2d backLeft = new Twist2d(-0.5, -0.5, 0);
	public static final Twist2d frontRight = new Twist2d(0.5, 0.5, 0);

	private static final List<OtherRobotInSimulation> opponents = new ArrayList<>();
	private static final List<OtherRobotInSimulation> partners = new ArrayList<>();

	private static final DriveTrainSimulationConfig AI_ROBOT_CONFIG = DriveTrainSimulationConfig.Default()
			.withRobotMass(SimulationConstants.ROBOT_WEIGHT);

	private static final RobotConfig robotConfig = new RobotConfig(
			SimulationConstants.ROBOT_WEIGHT,
			KilogramSquareMeters.of(8),
			new ModuleConfig(
					0.051,
					3.5,
					1.2,
					DCMotor.getKrakenX60(1).withReduction(8.14),
					60,
					1),
			SimulationConstants.Drivetrain.MODULE_TRANSLATIONS);
	
	private static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
			SimulationConstants.Drivetrain.XY, SimulationConstants.Drivetrain.THETA);

	public static void startOpponentRobotSimulations() {
		try {
			opponents.add(new OtherRobotInSimulation(
					SimulationConstants.ROBOT_QUEENING_POSITIONS[0],
					new AutonConfig("Depot Auton", DepotAuton::new, "Left Bump To Depot", "Depot To Tower Left")));
		} catch (Exception e) {
			DriverStation.reportError(
					"Unable to load path:" + e.getMessage(),
					false);
		}
	}

	public static OtherRobotInSimulation getOpponentRobot(int id) {
		return opponents.get(id);
	}

	public static OtherRobotInSimulation getAllianceRobot(int id) {
		return partners.get(id);
	}

	public static Pose2d[] getOpponentRobotPoses() {
		return getRobotPoses(opponents);
	}

	public static Pose2d[] getAlliancePartnerRobotPoses() {
		return getRobotPoses(partners);
	}

	public static Pose2d[] getRobotPoses() {
		return Stream.concat(opponents.stream()
				.map(instance -> instance.swerve.getActualPoseInSimulationWorld()),
				partners.stream()
						.map(instance -> instance.swerve.getActualPoseInSimulationWorld()))
				.toArray(Pose2d[]::new);
	}

	public static Pose2d[] getRobotPoses(List<OtherRobotInSimulation> instances) {
		return instances.stream()
				.map(instance -> instance.swerve.getActualPoseInSimulationWorld())
				.toArray(Pose2d[]::new);
	}

	public final SwerveDriveSimulation mapleSimSwerve;
	public final SelfControlledSwerveDriveSimulation swerve;
	private final int id;
	private final IntakeSimulation intake; 

	private final Gamepad gamepad;
	private final VStream speed;
	private final IStream turn;
	private final ChassisSpeeds gamepadSpeeds;

	public OtherRobotInSimulation(
			Pose2d queeningPose,
			AutonConfig... autons) {
		this.id = topId++; // uses the value and then increments topId
		this.mapleSimSwerve = new SwerveDriveSimulation(AI_ROBOT_CONFIG, queeningPose);
		this.swerve = new SelfControlledSwerveDriveSimulation(mapleSimSwerve);
		
		// clara make the intake variable
		this.intake = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            this.mapleSimSwerve,
            Meters.of(SimulationConstants.Intake.INTAKE_WIDTH),
            Meters.of(SimulationConstants.Intake.INTAKE_LENGTH),
            IntakeSimulation.IntakeSide.FRONT,
            SimulationConstants.Hopper.FUEL_CAPACITY
        );

		gamepad = new AutoGamepad(id);

		speed = VStream
				.create(() -> new Vector2D(gamepad.getLeftStick().y, -gamepad.getLeftStick().x))
				.filtered(
						new VDeadZone(Drive.DEADBAND),
						x -> x.clamp(1),
						x -> x.pow(Drive.POWER),
						x -> x.mul(Swerve.Constraints.MAX_VELOCITY_M_PER_S),
						new VRateLimit(Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED),
						new VLowPassFilter(Drive.RC));

		turn = IStream.create(gamepad::getRightX)
				.filtered(
						x -> SLMath.deadband(x, Turn.DEADBAND),
						x -> SLMath.spow(x, Turn.POWER),
						x -> x * Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S,
						new LowPassFilter(Turn.RC));

		gamepadSpeeds = new ChassisSpeeds(
				speed.get().x,
				speed.get().y,
				turn.get());

		SendableChooser<Command> behaviorChooser = new SendableChooser<>();
		behaviorChooser.setDefaultOption(
				"None", Commands.run(() -> swerve.setSimulationWorldPose(queeningPose), this));
		for (AutonConfig auton : autons)
			auton.register(behaviorChooser);
		behaviorChooser.addOption("Drive w/ Controller", getDriveCommand());
		behaviorChooser.onChange(Command::schedule);
		RobotModeTriggers.teleop()
				.onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));
		RobotModeTriggers.autonomous()
				.onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));
		RobotModeTriggers.disabled()
				.onTrue(Commands.runOnce(
						() -> {
							swerve.setSimulationWorldPose(queeningPose);
							swerve.runChassisSpeeds(
									new ChassisSpeeds(), new Translation2d(), false,
									false);
						},
						this)
						.ignoringDisable(true));

		SmartDashboard.putData("AIRobots/Robot_Behavior_" + id, behaviorChooser);
		SimulatedArena.getInstance().addDriveTrainSimulation(swerve.getDriveTrainSimulation());
	}

	private Command followPath(PathPlannerPath path) {
		return new FollowPathCommand(
				path,
				swerve::getActualPoseInSimulationWorld,
				swerve::getActualSpeedsRobotRelative,
				(speeds, feedForwards) -> swerve.runChassisSpeeds(speeds, new Translation2d(),
						false, false),
				driveController,
				robotConfig,
				FieldMirroringUtils::isSidePresentedAsRed,
				this);
	}

	/*
	 * <h2>Default command for the AI Robot</h2>
	 * <p>Shoots when in alliance zone, ferries in neutral and opponent alliance zone,
	 * and is always intaking. <- notice this
	 */
	private Command getAIDefaultCommand() {
		return Commands.run(
			() -> {
				this.swerve.getActualPoseInSimulationWorld();
				Pose2d pose = this.swerve.getActualPoseInSimulationWorld();
				if (pose.getX() < Field.getFerryZonePose(pose.getTranslation()).getX()){
					// SUGGESTION GUYS
					// MAKE ANOTHER METHOD CALLED UPDATESHOOTING WITHIN THIS CLASS
					// public method ^
					// and make it similar to the one from simulation.java
					// and later we'll add a call to the 				function within simulaton.java's updateshooting so they run at the same rate
					
				} else {
				}
			},
			this
		);
	}

	private void updateShooting() {
        Timer.delay(Math.abs(Arena2026Rebuilt.randomInRange(0.5)));
		Arena2026Rebuilt ARENA;
		ARENA = new Arena2026Rebuilt(true);
        // in here use the fuel creation methods from simulation.java,
		// and shoot using the target turret angle from a call to SOTM/InterpolationCalculator
	}
	
	// yo team, write the function based on the description comment block up top
	// you guys got this yo </3
	// if you have any questions I'm in the far corner from the door but if you ask me to write the code ima slime u ong
	// you should use documentation and the method below as an example of how to format this stuff
	// additionally, look within Simulation.java for methods for controlling the simulations
	private Command getDriveCommand() {
		return Commands.run(
				() -> {
					final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
							gamepadSpeeds,
							FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
									.plus(Rotation2d.fromDegrees(180)));
					swerve.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true,
							true);
				},
				this).beforeStarting(
						() -> swerve.setSimulationWorldPose(
								FieldMirroringUtils.toCurrentAlliancePose(
										SimulationConstants.ROBOTS_STARTING_POSITIONS[id - 1])));
	}
}