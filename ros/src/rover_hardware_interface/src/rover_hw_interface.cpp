#include <rover_hardware_interface/rover_hw_interface.h>

#include <std_msgs/Int32.h>

#include <iomanip>


namespace rover_base
{
    RoverHWInterface::RoverHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh)
    { 
        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;

        nh_.param<std::string>("mobile_base_controller/left_wheel", left_joint_name_, "left_wheel_joint");
        nh_.param<std::string>("mobile_base_controller/right_wheel", right_joint_name_, "right_wheel_joint");
        nh_.param<double>("mobile_base_controller/wheel_radius", wheel_radius_, 0.05);
        nh_.param<double>("mobile_base_controller/linear/x/max_velocity", max_velocity_, 0.2);
        nh_.param<double>("encoder_ticks_per_rev", encoder_ticks_per_rev_, 542.0);

        joint_names_[0] = left_joint_name_;
        joint_names_[1] = right_joint_name_;

        wheel_diameter_ = 2.0 * wheel_radius_;
        //max_velocity_ = 0.2; // m/s
        // ros_control RobotHW needs velocity in rad/s but in the config its given in m/s
        max_velocity_ = linearToAngular(max_velocity_);

        // Setup publisher for the motor driver 
        pub_left_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_left", 10);
        pub_right_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_right", 10);

        // Setup subscriber for the wheel encoders
        sub_left_encoder_ticks_ = nh_.subscribe("left_encoder_ticks", 10, &RoverHWInterface::leftEncoderTicksCallback, this);
        sub_right_encoder_ticks_ = nh_.subscribe("right_encoder_ticks", 10, &RoverHWInterface::rightEncoderTicksCallback, this);

        // Initialize the hardware interface
        init(nh_, nh_);
    }

    bool RoverHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing Rover Hardware Interface ...");
        num_joints_ = 2;
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor"};
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create a JointHandle (read and write) for each controllable joint
            // using the read-only joint handles within the JointStateInterface and 
            // register them with the JointVelocityInterface.
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
            velocity_joint_interface_.registerHandle(joint_handle);

            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller

            joint_velocity_commands_[i] = 0.0;

            // Initialize the pid controllers for the motors using the robot namespace
            std::string pid_namespace = "pid/" + motor_names[i];
            ROS_INFO_STREAM("pid namespace: " << pid_namespace);
            ros::NodeHandle nh(root_nh, pid_namespace);
            // TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
            pids_[i].init(nh, 0.8, 0.35, 0.5, 0.01, 3.5, -3.5, false, max_velocity_, -max_velocity_);
            pids_[i].setOutputLimits(-max_velocity_, max_velocity_);
        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&velocity_joint_interface_);

        ROS_INFO("... Done Initializing DiffBot Hardware Interface");
        return true;
    }

    void RoverHWInterface::read(const ros::Time& time, const ros::Duration& period)
    {
        //ROS_INFO_THROTTLE(1, "Read");
        ros::Duration elapsed_time = period;

        // Read from robot hw (motor encoders)
        // Fill joint_state_* members with read values
        double wheel_angles[2];
        double wheel_angle_deltas[2];
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            wheel_angles[i] = ticksToAngle(encoder_ticks_[i]);
            //double wheel_angle_normalized = normalizeAngle(wheel_angle);
            wheel_angle_deltas[i] = wheel_angles[i] - joint_positions_[i];
            
            joint_positions_[i] += wheel_angle_deltas[i];
            joint_velocities_[i] = wheel_angle_deltas[i] / period.toSec();
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller
        }
        const int width = 10;
        const char sep = ' ';
        std::stringstream ss;
        ss << std::left << std::setw(width) << std::setfill(sep) << "Read" << std::left << std::setw(width) << std::setfill(sep) << "ticks" << std::left << std::setw(width) << std::setfill(sep) << "angle" << std::left << std::setw(width) << std::setfill(sep) << "dangle" << std::setw(width) << std::setfill(sep) << "velocity" << std::endl;
        ss << std::left << std::setw(width) << std::setfill(sep) << "j0:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[0] << std::left << std::setw(width) << std::setfill(sep) << wheel_angles[0] << std::left << std::setw(width) << std::setfill(sep) << wheel_angle_deltas[0] << std::setw(width) << std::setfill(sep) << joint_velocities_[0] << std::endl;
        ss << std::left << std::setw(width) << std::setfill(sep) << "j1:" << std::left << std::setw(width) << std::setfill(sep) << encoder_ticks_[1] << std::left << std::setw(width) << std::setfill(sep) << wheel_angles[1] << std::left << std::setw(width) << std::setfill(sep) << wheel_angle_deltas[1] << std::setw(width) << std::setfill(sep) << joint_velocities_[1];
        ROS_INFO_STREAM(std::endl << ss.str());
        //printState();
    }

    void RoverHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // Write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s
        // Convert the velocity command to a percentage value for the motor
        // This maps the velocity to a percentage value which is used to apply
        // a percentage of the highest possible battery voltage to each motor.
        std_msgs::Int32 left_motor;
        std_msgs::Int32 right_motor;

        double pid_outputs[NUM_JOINTS];
        double motor_cmds[NUM_JOINTS];
        pid_outputs[0] = pids_[0](joint_velocities_[0], joint_velocity_commands_[0], period);
        pid_outputs[1] = pids_[1](joint_velocities_[1], joint_velocity_commands_[1], period);

        motor_cmds[0] = pid_outputs[0] / max_velocity_ * 100.0;
        motor_cmds[1] = pid_outputs[1] / max_velocity_ * 100.0;
        left_motor.data = motor_cmds[0];
        right_motor.data = motor_cmds[1];

        // Calibrate motor commands to deal with different gear friction in the
        // left and right motors and possible differences in the wheels.
        // Add calibration offsets to motor output in low regions
        // To tune these offset values command the robot to drive in a straight line and
        // adjust if it isn't going straight.
        // int left_offset = 10;
        // int right_offset = 5;
        // int threshold = 55;
        // if (0 < left_motor.data && left_motor.data < threshold)
        // {
        //     // the second part of the multiplication lets the offset decrease with growing motor values
        //     left_motor.data += left_offset * (threshold - left_motor.data) / threshold;
        // }
        // if (0 < right_motor.data && right_motor.data < threshold)
        // {
        //     // the second part of the multiplication lets the offset decrease with growing motor values
        //     right_motor.data += right_offset * (threshold - right_motor.data) / threshold;
        // }

        pub_left_motor_value_.publish(left_motor);
        pub_right_motor_value_.publish(right_motor);


        const int width = 10;
        const char sep = ' ';
        std::stringstream ss;
        // Header
        ss << std::left << std::setw(width) << std::setfill(sep) << "Write"
           << std::left << std::setw(width) << std::setfill(sep) << "velocity"
           << std::left << std::setw(width) << std::setfill(sep) << "p_error"
           << std::left << std::setw(width) << std::setfill(sep) << "i_error"
           << std::left << std::setw(width) << std::setfill(sep) << "d_error"
           << std::left << std::setw(width) << std::setfill(sep) << "pid out"
           << std::left << std::setw(width) << std::setfill(sep) << "percent"
           << std::endl;
        double p_error, i_error, d_error;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            pids_[i].getCurrentPIDErrors(&p_error, &i_error, &d_error);
            
            // Joint i
            std::string j = "j" + std::to_string(i) + ":";
            ss << std::left << std::setw(width) << std::setfill(sep) << j
               << std::left << std::setw(width) << std::setfill(sep) << joint_velocity_commands_[i]
               << std::left << std::setw(width) << std::setfill(sep) << p_error
               << std::left << std::setw(width) << std::setfill(sep) << i_error
               << std::left << std::setw(width) << std::setfill(sep) << d_error
               << std::left << std::setw(width) << std::setfill(sep) << pid_outputs[i]
               << std::left << std::setw(width) << std::setfill(sep) << motor_cmds[i]
               << std::endl;
        }
        ROS_INFO_STREAM(std::endl << ss.str());
    }

    void RoverHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

    void RoverHWInterface::printState()
    {
        // WARNING: THIS IS NOT REALTIME SAFE
        // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
        ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
    }

    std::string RoverHWInterface::printStateHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);

        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << "j" << i << ": " << std::fixed << joint_positions_[i] << "\t ";
            ss << std::fixed << joint_velocities_[i] << "\t ";
            ss << std::fixed << joint_efforts_[i] << std::endl;
        }
        return ss.str();
    }

    std::string RoverHWInterface::printCommandHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);
        ss << "    position     velocity         effort  \n";
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << std::fixed << joint_velocity_commands_[i] << "\t ";
        }
        return ss.str();
    }


    /// Process updates from encoders
    void RoverHWInterface::leftEncoderTicksCallback(const std_msgs::Int32& msg)
    {
        /// Update current encoder ticks in encoders array
        encoder_ticks_[0] = msg.data;
        ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << encoder_ticks_[0]);
    }

    void RoverHWInterface::rightEncoderTicksCallback(const std_msgs::Int32& msg)
    {
        /// Update current encoder ticks in encoders array
        encoder_ticks_[1] = msg.data;
        ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << encoder_ticks_[1]);
    }

    double RoverHWInterface::ticksToAngle(const int &ticks) const
    {
        // Convert number of encoder ticks to angle in radians
        double angle = (double)ticks * (2.0 * M_PI / encoder_ticks_per_rev_);
        ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
        return angle;
    }

    double RoverHWInterface::normalizeAngle(double &angle) const
    {
        // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
        angle = fmod(angle, 2.0*M_PI);

        if (angle < 0)
            angle += 2.0*M_PI;

        ROS_DEBUG_STREAM_THROTTLE(1, "Normalized angle: " << angle);
        return angle;
    }


    double RoverHWInterface::linearToAngular(const double &distance) const
    {
        return distance / wheel_diameter_ * 2.0;
    }

    double RoverHWInterface::angularToLinear(const double &angle) const
    {
        return angle * wheel_diameter_ / 2.0;
    }

};
