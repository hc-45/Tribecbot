package com.stuypulse.robot.util.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Simulation {
    public static final Simulation instance;
    
    static {
        instance = new Simulation();
        configure();
    }

    public static Simulation getInstance() {
        return instance;
    }

    public static final Arena2026Rebuilt ARENA = new Arena2026Rebuilt(true);
    public static void configure() {
        ARENA.setEfficiencyMode(false);
        SimulatedArena.overrideInstance(ARENA);
    }

    private final SwerveDriveSimulation mapleSimDrive;
    // publishers
    private final StructPublisher<Pose2d> drivetrain;
    private final StructArrayPublisher<Pose3d> fuel; 
    public Simulation() {
        mapleSimDrive = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();
        drivetrain = NetworkTableInstance.getDefault().getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
        fuel = NetworkTableInstance.getDefault().getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
    }

    public void publish() {
        if (mapleSimDrive != null) {
            drivetrain.set(mapleSimDrive.getSimulatedDriveTrainPose());
            fuel.set(SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Fuel"));
        }
    }
}