package com.stuypulse.robot.util.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Simulation {
    public static final Simulation instance;
    public final Arena2026Rebuilt ARENA;
    
    static {
        instance = new Simulation();
    }

    public static Simulation getInstance() {
        return instance;
    }

    private final SwerveDriveSimulation mapleSimDrive;

    /******************/
    /*** PUBLISHERS ***/
    /******************/
    private final StructPublisher<Pose2d> drivetrain;
    private final StructArrayPublisher<Pose3d> fuel; 

    private Simulation() {
        ARENA = new org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt(false);

        mapleSimDrive = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();
        drivetrain = NetworkTableInstance.getDefault().getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
        fuel = NetworkTableInstance.getDefault().getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
    }

    /**
     * <h3>Configures arena and gamepieces</h3>
     * <p>This method overrides the arena instance and resets the field. Should only be called once.
     */
    public void configure() {
        ARENA.resetFieldForAuto();
        ARENA.addDriveTrainSimulation(mapleSimDrive);
        SimulatedArena.overrideInstance(ARENA);
        // consider using addPieceWithVariance for shooting
    }

    
    // public void launchFuel() {
    //     RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
    //     // Specify the position of the chassis when the note is launched
    //     CommandSwerveDrivetrain.getInstance().getPose(),
    //     // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
    //     new Translation2d(0.2, 0),
    //     // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
    //     chassisSpeedsFieldRelative,
    //     // The shooter facing direction is the same as the robot’s facing direction
    //     SwerveDriveSim.getInstance().getHeading()
    //             // Add the shooter’s rotation
    //             + TurretSim.getInstance().getAngle(),
    //     // Initial height of the flying note
    //     0.45,
    //     // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
    //     getVelocity() / 6000 * 20,
    //     // The angle at which the note is launched
    //     Math.toRadians(55)
    //     );
    //     ARENA
    // }

    /**
     * <h3>Sends all simulation data to NetworkTables.</h3>
     */
    public void publish() {
        if (mapleSimDrive != null) {
            drivetrain.set(mapleSimDrive.getSimulatedDriveTrainPose());
            fuel.set(ARENA.getGamePiecesArrayByType("Fuel"));
        }
    }
}