import numpy as np
from manipulation.scenarios import AddIiwa
from pydrake.all import (
    Adder, AddMultibodyPlantSceneGraph, Demultiplexer, DiagramBuilder, InverseDynamicsController, MultibodyPlant, 
    Parser, PassThrough, RigidTransform, RollPitchYaw, StateInterpolatorWithDiscreteDerivative
)

def MakeIiwaPaddleStation(time_step=0.002):
    """
    Modified form of ManipulationStation from here:
    * https://deepnote.com/project/Manipulation-Station-p5S24ob_QPipeARm6RkGqQ/%2Fmanipulation_station.ipynb/#00003-c02ff458-1ca6-4059-9c48-a96ad5962c94
    * https://piazza.com/class/kt3s6xjnxl84xx?cid=336

    Removes the gripper. Adds a paddle and a ball
    """

    builder = DiagramBuilder()

    # Add (only) the iiwa to the scene.
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    iiwa = AddIiwa(plant, collision_model="with_box_collision")
    parser = Parser(plant)
    parser.AddModelFromFile("models/floor.sdf")
    parser.AddModelFromFile("models/paddle.sdf")
    ball = parser.AddModelFromFile("models/ball.sdf")
    plant.WeldFrames(
        plant.GetFrameByName("iiwa_link_7"),
        plant.GetFrameByName("base_link"),
        RigidTransform(RollPitchYaw(0, -np.pi / 2, 0), [0, 0, 0.25]),
    )
    plant.Finalize()

    num_iiwa_positions = plant.num_positions(iiwa)

    # Need a PassThrough system to export the input port.
    iiwa_position = builder.AddSystem(PassThrough(num_iiwa_positions))
    builder.ExportInput(iiwa_position.get_input_port(), "iiwa_position")
    builder.ExportOutput(iiwa_position.get_output_port(), "iiwa_position_command")

    # Export the iiwa "state" outputs.
    demux = builder.AddSystem(Demultiplexer(2 * num_iiwa_positions, num_iiwa_positions))
    builder.Connect(plant.get_state_output_port(iiwa), demux.get_input_port())
    builder.ExportOutput(demux.get_output_port(0), "iiwa_position_measured")
    builder.ExportOutput(demux.get_output_port(1), "iiwa_velocity_estimated")
    builder.ExportOutput(plant.get_state_output_port(iiwa), "iiwa_state_estimated")

    # Export the ball state
    builder.ExportOutput(plant.get_state_output_port(ball), "ball_state")

    # Make the plant for the iiwa controller to use.
    controller_plant = MultibodyPlant(time_step=time_step)
    controller_iiwa = AddIiwa(controller_plant, collision_model="with_box_collision")
    controller_parser = Parser(controller_plant)
    controller_parser.AddModelFromFile("models/paddle.sdf")
    controller_plant.WeldFrames(
        controller_plant.GetFrameByName("iiwa_link_7"),
        controller_plant.GetFrameByName("base_link"),
        RigidTransform(RollPitchYaw(0, -np.pi / 2, 0), [0, 0, 0.25]),
    )
    controller_plant.Finalize()

    kp = 100
    ki = 1
    kd = 20

    # Add the iiwa controller
    iiwa_controller = builder.AddSystem(
        InverseDynamicsController(
            controller_plant,
            kp=[kp] * num_iiwa_positions,
            ki=[ki] * num_iiwa_positions,
            kd=[kd] * num_iiwa_positions,
            has_reference_acceleration=False,
        )
    )
    iiwa_controller.set_name("iiwa_controller")
    builder.Connect(
        plant.get_state_output_port(iiwa),
        iiwa_controller.get_input_port_estimated_state(),
    )

    # Add in the feed-forward torque
    adder = builder.AddSystem(Adder(2, num_iiwa_positions))
    builder.Connect(iiwa_controller.get_output_port_control(), adder.get_input_port(0))
    
    # Use a PassThrough to make the port optional (it will provide zero values if not connected).
    torque_passthrough = builder.AddSystem(PassThrough([0] * num_iiwa_positions))
    builder.Connect(torque_passthrough.get_output_port(), adder.get_input_port(1))
    builder.ExportInput(torque_passthrough.get_input_port(), "iiwa_feedforward_torque")
    builder.Connect(adder.get_output_port(), plant.get_actuation_input_port(iiwa))

    # Add discrete derivative to command velocities.
    desired_state_from_position = builder.AddSystem(
        StateInterpolatorWithDiscreteDerivative(
            num_iiwa_positions, time_step, suppress_initial_transient=True
        )
    )
    desired_state_from_position.set_name("desired_state_from_position")
    builder.Connect(
        desired_state_from_position.get_output_port(),
        iiwa_controller.get_input_port_desired_state(),
    )
    builder.Connect(
        iiwa_position.get_output_port(), desired_state_from_position.get_input_port()
    )

    # Export "cheat" ports.
    builder.ExportOutput(scene_graph.get_query_output_port(), "geometry_query")
    builder.ExportOutput(plant.get_contact_results_output_port(), "contact_results")
    builder.ExportOutput(plant.get_state_output_port(), "plant_continuous_state")

    diagram = builder.Build()

    return diagram, plant, scene_graph