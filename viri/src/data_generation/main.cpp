#include <ros/ros.h>
#include "flightros/viri/EnvironmentManager.hpp"
#include "flightros/viri/TrajectoryGenerator.hpp"
#include "flightros/viri/SensorPublisher.hpp"

int main(int argc, char* argv[]) {
    // Initialize ROS
    ros::init(argc, argv, "flightmare_data_generator");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
    ros::Rate rate(50.0);

    // Initialize our modular systems
    EnvironmentManager env_manager;
    TrajectoryGenerator traj_gen;
    SensorPublisher sensor_pub(pnh, env_manager.getRGBCamera());

    // Connect to Unity
    if (!env_manager.connectToUnity()) {
        ROS_ERROR("Failed to start Unity environment. Shutting down.");
        return -1;
    }

    ROS_INFO("Waiting for Unity buffers to allocate...");
    // ros::Duration(5.0).sleep();
    
    flightlib::FrameID frame_id = 0;
    const double dt = 0.02;

    while (ros::ok()) {
        double current_time = frame_id * dt;

        // 1. Update Kinematics
        traj_gen.updateTargetState(current_time, dt, env_manager.getTargetQuad());
        traj_gen.updateChaserState(current_time, env_manager.getChaserQuad());
        traj_gen.updateRingStates(dt, env_manager.getRingQuads());

        // 2. Render and Publish (Every frame in this setup, or add your % 5 logic here)
        if (frame_id % 1 == 0) {
            env_manager.renderFrame(frame_id);
            sensor_pub.publishImages();
        }

        // 3. Tick
        frame_id++;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}