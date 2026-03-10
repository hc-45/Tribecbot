package com.stuypulse.robot.util.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Simulation {
    public static final Simulation instance;
    public final SimulatedArena ARENA;
    
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
        ARENA = SimulatedArena.getInstance();

        mapleSimDrive = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();
        drivetrain = NetworkTableInstance.getDefault().getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
        fuel = NetworkTableInstance.getDefault().getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        configure();
    }

    /**
     * <h3>Configures arena and gamepieces</h3>
     * <p>This method also acts as a field reset method, and can be called more than once.
     */
    public void configure() {
        ARENA.placeGamePiecesOnField();
        // don't overrideInstance for the arena, it'll break sim for some reason
        // consider using addPieceWithVariance for shooting
    }

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