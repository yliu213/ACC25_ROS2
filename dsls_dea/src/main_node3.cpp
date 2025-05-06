// px4 msgs
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp> // attitude setpoint
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp> // body rate setpoint
#include "px4_ros_com/frame_transforms.h" // for frame transforms

// ros2 libraries
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <cfloat> // for DBL_MIN
#include <iostream>
#include <rcl_interfaces/msg/set_parameters_result.hpp> // for setting parameters at runtime

// pkg related
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <gazebo_msgs/msg/link_states.hpp> // for gazebo
#include <Eigen/Dense>
#include "dsls_dea/common.h"
#include "dsls_dea/DSLSDEAController.h" // MATLAB generated
#include "dsls_dea_msgs/msg/d_sls_state.hpp" // for custom msg
#include "dsls_dea_msgs/msg/dea_state.hpp"
#include "dsls_dea_msgs/msg/lpf_data.hpp"
#include "dsls_dea_msgs/msg/mission_ref.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std::placeholders;
using namespace dsls_dea_msgs::msg;
using namespace geometry_msgs::msg;
using namespace px4_ros_com::frame_transforms;

#define PI 3.1415926535
#define MIN_DELTA_TIME 1e-16
#define FD_EPSILON DBL_MIN

enum class ControllerPhase {
    WAIT_FOR_TAKEOFF,
    DSLS_ENABLED
};

class DSLS_DEA : public rclcpp::Node
{
public:
	DSLS_DEA() : Node("dsls_dea_controller")
	{
        // ROS2 node initialization
        // initialize publishers
        // uav0
		offboard0_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory0_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle0_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        rate_setpoint_pub_0_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10); 
        // uav1
        offboard1_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
		trajectory1_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
        vehicle1_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
        rate_setpoint_pub_1_ = this->create_publisher<VehicleRatesSetpoint>("/px4_1/fmu/in/vehicle_rates_setpoint", 10); 
        // custom msgs
        dsls_state_pub_ = this->create_publisher<DSlsState>("/dsls_controller/dsls_state", 10);
        dea_state_pub_ = this->create_publisher<DEAState>("/dsls_controller/dea_state", 10);
        dea_force_0_pub_ = this->create_publisher<Vector3Stamped>("/dsls_controller/dea_force_0", 10);
        dea_force_1_pub_ = this->create_publisher<Vector3Stamped>("/dsls_controller/dea_force_1", 10);
        ratesp_0_pub_ = this->create_publisher<VehicleRatesSetpoint>("/dsls_controller/rate_setpoint_0", 10);
        ratesp_1_pub_ = this->create_publisher<VehicleRatesSetpoint>("/dsls_controller/rate_setpoint_1", 10);
        lpf_data_pub_ = this->create_publisher<LPFData>("/dsls_controller/lpf_data", 10);
        mission_ref_pub_ = this->create_publisher<MissionRef>("/dsls_controller/mission_ref", 10);

        // initialize subscribers
        // gazebo
        gazebo_state_sub_ = this->create_subscription<gazebo_msgs::msg::LinkStates>("/gazebo/link_states",1000,std::bind(&DSLS_DEA::gazeboCb, this, _1));

        // initialize parameters
        // Mission
        mission_enabled_ = this->declare_parameter<bool>("mission_enabled_", false);
        // Inner-loop
        norm_thrust_offset_ = this->declare_parameter<double>("norm_thrust_offset_", 0.0);
        att_ctrl_tau_ = this->declare_parameter<double>("att_ctrl_tau_", 0.8);
        /* DEA Controller */
        // Switch
        dea_enabled_ = this->declare_parameter<bool>("dea_enabled_", false); 
        dea_preintegrate_enabled_ = this->declare_parameter<bool>("dea_preintegrate_enabled_", false);
        // Reference
        radium_scalar_ = this->declare_parameter<double>("radium_scalar_", 0.0);
        freq_scalar_ = this->declare_parameter<double>("freq_scalar_", 0.0);
        r_1_ = this->declare_parameter<double>("r_1_", 1.0);
        r_2_ = this->declare_parameter<double>("r_2_", 1.0);
        r_3_ = this->declare_parameter<double>("r_3_", 1.0);
        fr_1_ = this->declare_parameter<double>("fr_1_", 1.0);
        fr_2_ = this->declare_parameter<double>("fr_2_", 1.0);
        fr_3_ = this->declare_parameter<double>("fr_3_", 1.0);
        c_1_ = this->declare_parameter<double>("c_1_", 0.0);
        c_2_ = this->declare_parameter<double>("c_2_", 0.0);
        c_3_ = this->declare_parameter<double>("c_3_", -2.0);
        ph_1_ = this->declare_parameter<double>("ph_1_", 0.0);
        ph_2_ = this->declare_parameter<double>("ph_2_", PI/2);
        ph_3_ = this->declare_parameter<double>("ph_3_", 0.0);
        q_1_3_r_ = this->declare_parameter<double>("q_1_3_r_", 0.7406322196911044);
        q_2_1_r_ = this->declare_parameter<double>("q_2_1_r_", 0.0);
        q_2_2_r_ = this->declare_parameter<double>("q_2_2_r_", -0.6717825272800765);

        /* LPF */ 
        lpf_tau_ = this->declare_parameter<double>("lpf_tau", 0.01);
        lpf_xi_ = this->declare_parameter<double>("lpf_xi", 0.7);
        lpf_omega_ = this->declare_parameter<double>("lpf_omega", 100);

        if(!param_tuning_enabled_){
            for(int i = 0; i < 4; i++){
                dea_k_[0 + i*6] = dea_k1_[i];
                dea_k_[1 + i*6] = dea_k2_[i];
                dea_k_[2 + i*6] = dea_k3_[i];
                dea_k_[3 + i*6] = dea_k4_[i];
                dea_k_[4 + i*6] = dea_k5_[i];
                dea_k_[5 + i*6] = dea_k6_[i];
            }
        }

        for(int j=0; j<4; j++) dea_xi4_.dea_xi4[j] = dea_xi4_ic_[j]; 
        
        dea_last_status_ = dea_enabled_;

        // takeoff counter
		offboard_setpoint_counter_ = 0;
        mission_time_last_ = this->get_clock()->now().seconds();
        RCLCPP_INFO(this->get_logger(),"Initialization Complete");

		auto timer_callback = [this]() -> void {
            switch (phase_) {
            case ControllerPhase::WAIT_FOR_TAKEOFF:
            {
                OffboardControlMode mode_msg{};
                mode_msg.position     = true;  
                mode_msg.velocity     = false;
                mode_msg.acceleration = false;
                mode_msg.attitude     = false;
                mode_msg.body_rate    = false;
                mode_msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
                offboard0_control_mode_publisher_->publish(mode_msg);
                offboard1_control_mode_publisher_->publish(mode_msg);
        
                publish_trajectory_setpoint();  
        
                if (offboard_setpoint_counter_ == 10) {
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, 1); // OFFBOARD for vehicle 0
                    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, 2); // OFFBOARD for vehicle 1
                    arm();  
                    armed_time_ = this->get_clock()->now(); 
                }
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }
        
                if (is_armed_) {
                    auto now = this->get_clock()->now();
                    double elapsed = (now - armed_time_).seconds();
                    if (elapsed > 9.0) {
                        RCLCPP_INFO(this->get_logger(), "9s after arming, switching to DSLS controller");
                        phase_ = ControllerPhase::DSLS_ENABLED; 
                    }
                }
                break;
            }
            case ControllerPhase::DSLS_ENABLED:
            {
                // Switch to body_rate offboard mode
                OffboardControlMode mode_msg{};
                mode_msg.position     = false;
                mode_msg.velocity     = false;
                mode_msg.acceleration = false;
                mode_msg.attitude     = false;
                mode_msg.body_rate    = true;
                mode_msg.timestamp = this->get_clock()->now().nanoseconds()/1000;
                offboard0_control_mode_publisher_->publish(mode_msg);
                offboard1_control_mode_publisher_->publish(mode_msg);
                this-> dea_enabled_ = true; // set manually, o.w. bugs
                break;
            }
            }
        };
        // parameter callback
        // this is needed to change the parameters at runtime using rqt_reconfigure
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            -> rcl_interfaces::msg::SetParametersResult
            {
              rcl_interfaces::msg::SetParametersResult result;
              result.successful = true;
              for (auto & param : params) {
                if (param.get_name() == "mission_enabled_") {
                    mission_enabled_ = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), "Param changed: mission_enabled_=%s", mission_enabled_ ? "true" : "false");
                } else if (param.get_name() == "norm_thrust_offset_"){
                    norm_thrust_offset_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Param changed: norm_thrust_offset_=%f", norm_thrust_offset_);
                } else if (param.get_name() == "att_ctrl_tau_"){
                    att_ctrl_tau_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Param changed: att_ctrl_tau_=%f", att_ctrl_tau_);
                } else if(param.get_name() == "dea_enabled_") {
                    dea_enabled_ = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), "Param changed: dea_enabled_=%s", dea_enabled_ ? "true" : "false");
                } else if(param.get_name() == "dea_preintegrate_enabled_") {
                    dea_preintegrate_enabled_ = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), "Param changed: dea_preintegrate_enabled_=%s", dea_preintegrate_enabled_ ? "true" : "false");
                } else {
                    result.successful = false;
                }
              }
              return result;
            }
        );          
        timer_ = this->create_wall_timer(50ms, timer_callback); // 50ms
    }

	void arm();

private:
    // ros2 timer and time
	rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    double gazebo_last_called_;
    double dea_start_time_; 
    double dea_end_time_;
    double controller_last_called_; 

    // publishers
    // uav0
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard0_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory0_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle0_command_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rate_setpoint_pub_0_;
    // uav1
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard1_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory1_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle1_command_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rate_setpoint_pub_1_;
    // custom msgs
    rclcpp::Publisher<DSlsState>::SharedPtr dsls_state_pub_;
    rclcpp::Publisher<DEAState>::SharedPtr dea_state_pub_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr dea_force_0_pub_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr dea_force_1_pub_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr ratesp_0_pub_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr ratesp_1_pub_;
    rclcpp::Publisher<LPFData>::SharedPtr lpf_data_pub_;
    rclcpp::Publisher<MissionRef>::SharedPtr mission_ref_pub_;

    // subscribers
    rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr gazebo_state_sub_;

    // pre-takeoff
    ControllerPhase phase_ = ControllerPhase::WAIT_FOR_TAKEOFF;
    rclcpp::Time armed_time_;
    bool is_armed_ = false;
	uint64_t offboard_setpoint_counter_;   

    // shared ptrs
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_; // for parameter callback

    /* Mission */
    bool traj_tracking_enabled_ = false;
    bool mission_enabled_ = false;
    bool mission_initialized_ = false;
    bool mission_start_ = false;
    bool mission_ref_updated_ = false;
    int mission_stage_ = 0;
    double mission_time_last_;
    double dea_mission_setpoint0_[6] = {0.0, 0.0, -2.0, q_1_3_r_, q_2_1_r_, q_2_2_r_}; 
    double dea_mission_setpoint1_[6] = {1.0, 0.0, -2.0, q_1_3_r_, q_2_1_r_, q_2_2_r_};
    double dea_mission_setpoint2_[6] = {0.0, 1.0, -2.0, q_1_3_r_, q_2_1_r_, q_2_2_r_};
    double dea_mission_setpoint3_[6] = {0.0, 0.0, -2.5, q_1_3_r_, q_2_1_r_, q_2_2_r_};
    double dea_mission_trajectory_[15] = {
        1.0, 0.5, 0.0, 0.0,
        1.0, 0.5, 0.0, PI/2,
        0.0, 0.0, -2.0, 0.0,
        q_1_3_r_, q_2_1_r_, q_2_2_r_
    };

    // gazebo index matching
    bool gazebo_link_name_matched_ = false;
    int uav0_link_index_;
    int uav1_link_index_;
    int pend0_link_index_;
    int pend1_link_index_;
    int load_link_index_;
    const char* link_name_[5] = {
        "px4vision_0::base_link", 
        "px4vision_1::base_link",
        "slung_load::pendulum_0::base_link",
        "slung_load::pendulum_1::base_link",
        "slung_load::load::base_link"
        };

    // px4 msgs
    VehicleAttitudeSetpoint attitude_dea_0_, attitude_dea_1_;
    VehicleRatesSetpoint rate_dea_0_, rate_dea_1_; 

    // geometry_msgs
    /* uav0 */
    PoseStamped uav0_pose_, uav0_pose_last_;
    TwistStamped uav0_twist_, uav0_twist_last_;
    /* uav1 */
    PoseStamped uav1_pose_, uav1_pose_last_;
    TwistStamped uav1_twist_, uav1_twist_last_;
    /* load */
    PoseStamped load_pose_, load_pose_last_;
    TwistStamped load_twist_;
    /* pend0 */
    Vector3 pend0_q_, pend0_q_last_, pend0_q_last_2_;
    Vector3 pend0_q_dot_, pend0_q_dot_last_, pend0_q_dot_last_2_;
    Vector3 pend0_q_dot_lpf1_, pend0_q_dot_lpf1_last_, pend0_q_dot_lpf1_last_2_;
    Vector3 pend0_q_dot_lpf2_, pend0_q_dot_lpf2_last_, pend0_q_dot_lpf2_last_2_;
    Vector3 pend0_omega_;
    /* pend1 */
    Vector3 pend1_q_, pend1_q_last_, pend1_q_last_2_;
    Vector3 pend1_q_dot_, pend1_q_dot_last_, pend1_q_dot_last_2_;
    Vector3 pend1_q_dot_lpf1_, pend1_q_dot_lpf1_last_, pend1_q_dot_lpf1_last_2_;
    Vector3 pend1_q_dot_lpf2_, pend1_q_dot_lpf2_last_, pend1_q_dot_lpf2_last_2_;
    Vector3 pend1_omega_;
    /* DEA Controller Output Force */
    Vector3Stamped dea_force_0_;
    Vector3Stamped dea_force_1_;

    // bools
    bool use_ned_frame_ = false; //false
    bool gazebo_omega_enabled_ = false;
    bool dea_enabled_ = false; //false
    bool dea_last_status_ = false;
    bool open_loop_ctrl_enabled_ = false;
    bool param_tuning_enabled_ = false;
    bool lpf_debug_enabled_ = false;
    bool time_sync_debug_enabled_ = false;
    bool force_clip_enabled_ = false;
    bool dea_preintegrate_enabled_ = false;

    /* physical parameters */
    double max_fb_acc_ = 10.0;
    double uav_mass_ = 1.56;
    double load_mass_ = 0.25;
    double cable_length_ = 0.85;
    double gravity_acc_ = 9.80665;
    double max_tilt_angle_ = 0.78598163; //45 deg
    double max_xi_value_ = 1e5; 
    double max_thrust_force_ = 31.894746920044025;
    double att_ctrl_tau_ = 0.8;
    double norm_thrust_offset_ = 0.0; // 0.0
    const double dea_param_[4] = {load_mass_, uav_mass_, cable_length_, gravity_acc_};

    /* Reference Trajectory & Pend Angle */
    double radium_scalar_ = 0;
    double freq_scalar_ = 0;
    double r_1_ = 5.0 ;
    double fr_1_ = 1.0;
    double c_1_ = 0;
    double ph_1_ = 0;
    double r_2_ = 5.0 ;
    double fr_2_ = 1.0;
    double c_2_ = 0.0;
    double ph_2_ = PI/2;
    double r_3_ = 0;
    double fr_3_ = 0.0;
    double c_3_ = -2.0;
    double ph_3_ = 0;
    double pend_angle_deg_ = 90; // deprecated
    double q_1_3_r_ = 0.7406322196911044; //sqrt(2)/2; 
    double q_2_1_r_ = 0;
    double q_2_2_r_ = -0.6717825272800765; //-sqrt(2)/2;
    double dsls_dea_ref_temp_[15];

    /* Gains */
    double dea_k1_[4] = {31.6228,   37.4556,   20.6010,    6.4963};
    double dea_k2_[4] = {31.6228,   37.4556,   20.6010,    6.4963};
    double dea_k3_[4] = {24.0000,   50.0000,   35.0000,   10.0000}; 
    double dea_k4_[4] = {2.0000,    3.0000,         0,         0};
    double dea_k5_[4] = {2.0000,    3.0000,         0,         0};
    double dea_k6_[4] = {2.0000,    3.0000,         0,         0};
    double dea_k_[24];

    /* lpf parameters*/
    double lpf_tau_;
    double lpf_xi_;
    double lpf_omega_;

    /* (Primed) Initial Conditions */
    double dea_xi4_ic_[4] = {-gravity_acc_, -5.10, 0, 0}; 

    // custom msgs
    DSlsState state18_;
    DEAState dea_xi4_;
    LPFData lpf_data_;
    MissionRef mission_ref_;

    // methods
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t target_sys_id = 1);
    void gazeboCb(const gazebo_msgs::msg::LinkStates::SharedPtr msg);
    int enu2ned(void);
    int enu2esd(void);
    int applyOpenLoopController(void);
    int applyDSLSDEAController(
        DSlsState state18, 
        DEAState &dea_xi4, 
        double t
    );
    Vector3 applyFiniteDiffVector3(Vector3 v, Vector3 v_last, Vector3 v_dot_last, double diff_time);
    double applyFiniteDiff(double x, double x_last, double x_dot_last, double diff_time);
    Vector3 applyLPFVector3(
        Vector3 v, Vector3 v_last, Vector3 v_last_2, Vector3 v_dot_last, Vector3 v_dot_last_2, 
        double diff_time, double tau, double xi, double omega, int order
    );
    double applyLPF(double u, double u_last, double u_last_2, double y_last, double y_last_2, double Ts, double tau, double xi, double omega, int order);
    Vector3 crossProduct(const Vector3 v1, const Vector3 v2);
    void force_rate_convert(double controller_output[3], VehicleAttitudeSetpoint &attitude, VehicleRatesSetpoint &rate,int uav);
    void pubDebugData();
    void executeMission(void);
    void checkMissionStage(double mission_time_span);
};


void DSLS_DEA::arm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 1); // arm vehicle 0
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 2); // arm vehicle 1
    is_armed_ = true;
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void DSLS_DEA::publish_trajectory_setpoint(){
	TrajectorySetpoint msg0{};
	msg0.position = {0.0, 0.35, -1.0}; 
	msg0.yaw = -3.14; 
	msg0.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    TrajectorySetpoint msg1{};
	msg1.position = {0.0, -0.35, -1.0};
	msg1.yaw = -3.14; 
	msg1.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	trajectory0_setpoint_publisher_->publish(msg0);
    trajectory1_setpoint_publisher_->publish(msg1);
}

void DSLS_DEA::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t target_sys_id){
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = target_sys_id;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    if (target_sys_id == 1) {
        vehicle0_command_publisher_->publish(msg);
    } else {
        vehicle1_command_publisher_->publish(msg);
    }
}

void DSLS_DEA::gazeboCb(const gazebo_msgs::msg::LinkStates::SharedPtr msg){
    /* Match links on the first call*/
    if(!gazebo_link_name_matched_){
        RCLCPP_INFO(this->get_logger(), "[gazeboCb] Matching Gazebo Links");
        //RCLCPP_INFO(this->get_logger(), "[gazeboCb] msg->name.size() = %zu", msg->name.size()); // 21

        int temp_index[5];
        for(int i=0; i<21; i++){
            for(int j=0; j<5; j++){
                if(msg->name[i] == link_name_[j]){
                    temp_index[j] = i;
                };
            }
        }
        uav0_link_index_ = temp_index[0];    RCLCPP_INFO(this->get_logger(), "[gazeboCb] uav0_link_index=%d", uav0_link_index_);
        uav1_link_index_ = temp_index[1];    RCLCPP_INFO(this->get_logger(), "[gazeboCb] uav1_link_index=%d", uav1_link_index_);
        pend0_link_index_ = temp_index[2];   RCLCPP_INFO(this->get_logger(), "[gazeboCb] pend0_link_index=%d", pend0_link_index_);
        pend1_link_index_ = temp_index[3];   RCLCPP_INFO(this->get_logger(), "[gazeboCb] pend0_link_index=%d", pend1_link_index_);
        load_link_index_ = temp_index[4];    RCLCPP_INFO(this->get_logger(), "[gazeboCb] load_link_index=%d", load_link_index_);
        gazebo_link_name_matched_ = true;
        RCLCPP_INFO(this->get_logger(), "[gazeboCb] Matching Complete");
    }

    uav0_pose_.pose = msg -> pose[uav0_link_index_];
    uav0_twist_.twist = msg -> twist[uav0_link_index_];
    uav1_pose_.pose = msg -> pose[uav1_link_index_];
    uav1_twist_.twist = msg -> twist[uav1_link_index_];
    load_pose_.pose = msg -> pose[load_link_index_];
    load_twist_.twist = msg -> twist[load_link_index_];

    // coordinate transform
    if(use_ned_frame_) enu2ned();
    else enu2esd();
    
    // q1
    pend0_q_.x = (load_pose_.pose.position.x - uav0_pose_.pose.position.x);
    pend0_q_.y = (load_pose_.pose.position.y - uav0_pose_.pose.position.y);
    pend0_q_.z = (load_pose_.pose.position.z - uav0_pose_.pose.position.z);
    double q0_norm = sqrt(pend0_q_.x*pend0_q_.x + pend0_q_.y*pend0_q_.y + pend0_q_.z*pend0_q_.z);
    pend0_q_.x = pend0_q_.x / q0_norm;
    pend0_q_.y = pend0_q_.y / q0_norm;
    pend0_q_.z = pend0_q_.z / q0_norm;
    // q2
    pend1_q_.x = (load_pose_.pose.position.x - uav1_pose_.pose.position.x);
    pend1_q_.y = (load_pose_.pose.position.y - uav1_pose_.pose.position.y);
    pend1_q_.z = (load_pose_.pose.position.z - uav1_pose_.pose.position.z);
    double q1_norm = sqrt(pend1_q_.x*pend1_q_.x + pend1_q_.y*pend1_q_.y + pend1_q_.z*pend1_q_.z);
    pend1_q_.x = pend1_q_.x / q1_norm;
    pend1_q_.y = pend1_q_.y / q1_norm;
    pend1_q_.z = pend1_q_.z / q1_norm;

    double diff_time;
    diff_time = (this->get_clock()->now().seconds() - gazebo_last_called_); 
    gazebo_last_called_ = this->get_clock()->now().seconds();

    if(gazebo_omega_enabled_){
        pend0_omega_.x = (msg -> twist[pend0_link_index_]).angular.x;
        pend0_omega_.y = -(msg -> twist[pend0_link_index_]).angular.y;
        pend0_omega_.z = -(msg -> twist[pend0_link_index_]).angular.z;
        pend1_omega_.x = (msg -> twist[pend1_link_index_]).angular.x;
        pend1_omega_.y = -(msg -> twist[pend1_link_index_]).angular.y;
        pend1_omega_.z = -(msg -> twist[pend1_link_index_]).angular.z;
    } 

    if(dea_enabled_ && dea_enabled_ != dea_last_status_) {
        dea_start_time_ = this->get_clock()->now().seconds();
        RCLCPP_INFO(this->get_logger(),"[main] DEA controller enabled");
    }
    else if(!dea_enabled_ && dea_enabled_ != dea_last_status_){
        dea_end_time_ = this->get_clock()->now().seconds();
        RCLCPP_INFO(this->get_logger(),"[main] DEA controller disabled");
    } 
    dea_last_status_ = dea_enabled_;

    if(dea_enabled_){
        applyDSLSDEAController(state18_, dea_xi4_, this->get_clock()->now().seconds() - dea_start_time_);
        rate_dea_0_.timestamp = this->get_clock()->now().nanoseconds()/1000;
        rate_dea_1_.timestamp = this->get_clock()->now().nanoseconds()/1000;
        rate_setpoint_pub_0_->publish(rate_dea_0_); 
        rate_setpoint_pub_1_->publish(rate_dea_1_); 
        executeMission();
    }
    pubDebugData();
}

void DSLS_DEA::pubDebugData(){
    dsls_state_pub_->publish(state18_);
    dea_state_pub_->publish(dea_xi4_);    
    dea_force_0_pub_->publish(dea_force_0_);
    dea_force_1_pub_->publish(dea_force_1_);    
    ratesp_0_pub_->publish(rate_dea_0_);
    ratesp_1_pub_->publish(rate_dea_1_);
    //lpf_data_pub_->publish(lpf_data_); 
    mission_ref_pub_->publish(mission_ref_);  
}

void DSLS_DEA::executeMission(void) {
    if(mission_enabled_) {
        switch(mission_stage_) {
            case 0: // Set-point 0 (DEA Origin)
                if(!mission_initialized_){
                    RCLCPP_INFO(this->get_logger(),"[exeMission] Mission started at stage 0");
                    mission_initialized_ = true;
                }
                c_1_ = 0.0;
                c_2_ = 0.0;
                c_3_ = -2.0;
                q_1_3_r_ = 0.7406322196911044; 
                q_2_1_r_ = 0;
                q_2_2_r_ = -0.6717825272800765; 
                checkMissionStage(20);
                break;
            case 1: // Set-point 1
                c_1_ = 1.0;
                c_2_ = 0.0;
                c_3_ = -2.0;
                q_1_3_r_ = 0.7406322196911044; 
                q_2_1_r_ = 0;
                q_2_2_r_ = -0.6717825272800765; 
                checkMissionStage(10);
                break;
            case 2: // Set-point 2
                c_1_ = 0.0;
                c_2_ = 1.0;
                c_3_ = -2.0;
                q_1_3_r_ = 0.7406322196911044; 
                q_2_1_r_ = 0;
                q_2_2_r_ = -0.6717825272800765;
                checkMissionStage(10);
                break;  
            case 3: // Set-point 3
                c_1_ = 0.0;
                c_2_ = 0.0;
                c_3_ = -2.5;
                q_1_3_r_ = 0.7406322196911044; 
                q_2_1_r_ = 0;
                q_2_2_r_ = -0.6717825272800765;
                checkMissionStage(10);
                break;
            case 4: // Back to Set-point 0 (DEA Origin)
                c_1_ = 0.0;
                c_2_ = 0.0;
                c_3_ = -2.0;
                q_1_3_r_ = 0.7406322196911044; 
                q_2_1_r_ = 0;
                q_2_2_r_ = -0.6717825272800765;
                checkMissionStage(10);
                break;
            case 5: // Trajectory Tracking
                traj_tracking_enabled_ = true;
                checkMissionStage(40);
                break;
            case 6: // Back to Set-point 0 (DEA Origin)
                traj_tracking_enabled_ = false;
                c_1_ = 0.0;
                c_2_ = 0.0;
                c_3_ = -2.0;
                q_1_3_r_ = 0.7406322196911044; 
                q_2_1_r_ = 0;
                q_2_2_r_ = -0.6717825272800765;
                checkMissionStage(10);
                break;       
            default: // Reset Mission
                if(this->get_clock()->now().seconds() - mission_time_last_ >= 10){
                    RCLCPP_INFO(this->get_logger(), "[exeMission] Mission Accomplished");
                    mission_time_last_ = this->get_clock()->now().seconds();
                    mission_initialized_ = false;
                }
        }
    } else {
        mission_time_last_ = this->get_clock()->now().seconds();
        mission_stage_ = 0;
        mission_initialized_ = false;
        traj_tracking_enabled_ = false;
        radium_scalar_ = 0;
        c_1_ = 0.0;
        c_2_ = 0.0;
        c_3_ = -2.0;
        q_1_3_r_ = 0.7406322196911044; 
        q_2_1_r_ = 0;
        q_2_2_r_ = -0.6717825272800765; 
    }
}

void DSLS_DEA::checkMissionStage(double mission_time_span) {
    if(this->get_clock()->now().seconds() - mission_time_last_ >= mission_time_span) {
        mission_time_last_ = this->get_clock()->now().seconds();
        mission_stage_ += 1;
        RCLCPP_INFO(this->get_logger(),"[exeMission] Stage %d ended, switching to stage %d", mission_stage_ - 1, mission_stage_);
    }
}

int DSLS_DEA::applyDSLSDEAController(DSlsState state18, DEAState &dea_xi4, double t){
    /* Get Time Step */
    double diff_time;
    diff_time = (this->get_clock()->now().seconds() - controller_last_called_);
    controller_last_called_ = this->get_clock()->now().seconds(); 

    /* Finite Difference */
    if(diff_time > MIN_DELTA_TIME){    
        pend0_q_dot_ = applyFiniteDiffVector3(pend0_q_, pend0_q_last_, pend0_q_dot_last_, diff_time);
        pend1_q_dot_ = applyFiniteDiffVector3(pend1_q_, pend1_q_last_, pend0_q_dot_last_, diff_time);
    }    

    pend0_omega_ = crossProduct(pend0_q_, pend0_q_dot_);
    pend1_omega_ = crossProduct(pend1_q_, pend1_q_dot_);
    pend0_q_last_ = pend0_q_;
    pend1_q_last_ = pend1_q_;
  
    // system state msg
    // state vector x = [xp,q1,q2,vp,w1,w2]
    state18_.header.stamp = this->get_clock()->now();
    state18_.state18[0] = load_pose_.pose.position.x;
    state18_.state18[1] = load_pose_.pose.position.y;
    state18_.state18[2] = load_pose_.pose.position.z;
    state18_.state18[3] = pend0_q_.x;
    state18_.state18[4] = pend0_q_.y;
    state18_.state18[5] = pend0_q_.z;
    state18_.state18[6] = pend1_q_.x;
    state18_.state18[7] = pend1_q_.y;
    state18_.state18[8] = pend1_q_.z;
    state18_.state18[9] = load_twist_.twist.linear.x;
    state18_.state18[10] = load_twist_.twist.linear.y;
    state18_.state18[11] = load_twist_.twist.linear.z; 
    state18_.state18[12] = pend0_omega_.x;
    state18_.state18[13] = pend0_omega_.y;
    state18_.state18[14] = pend0_omega_.z;
    state18_.state18[15] = pend1_omega_.x;
    state18_.state18[16] = pend1_omega_.y;
    state18_.state18[17] = pend1_omega_.z;   

    /* Getting Full State */
    double state22[22] = {};
    for(int i = 0; i < 22; i++){
        if(i < 18) state22[i] = state18.state18[i]; //state18.state18[i]
        else if(i < 22) state22[i] = dea_xi4.dea_xi4[i-18]; //dea_xi4.dea_xi4[i-18]
    }

    /* Apply Controller */
    double F1[3];
    double F2[3];
    double xi_dot[4];
    bool reset_controller_state = false;

    // get dea_ref
    std::array<double,15> dea_ref{};
    if (!traj_tracking_enabled_) {
        dea_ref = { r_1_*radium_scalar_, fr_1_*freq_scalar_, c_1_, ph_1_,
                    r_2_*radium_scalar_, fr_2_*freq_scalar_, c_2_, ph_2_,
                    r_3_*radium_scalar_, fr_3_*freq_scalar_, c_3_, ph_3_,
                    q_1_3_r_, q_2_1_r_, q_2_2_r_ };
    } else {
        dea_ref = { 1.0, 0.5, 0.0, 0.0,
                    1.0, 0.5, 0.0, PI/2,
                    0.0, 0.0, -2.0, 0.0,
                    q_1_3_r_, q_2_1_r_, q_2_2_r_ };
    }
    // mission reference
    // format: ref = r * sin(fr * t + ph) + c
    mission_ref_.header.stamp = this->get_clock()->now();
    mission_ref_.dea_ref[0] = dea_ref[0]*sin(dea_ref[1]*t + dea_ref[3]) + dea_ref[2];
    mission_ref_.dea_ref[1] = dea_ref[4]*sin(dea_ref[5]*t + dea_ref[7]) + dea_ref[6];
    mission_ref_.dea_ref[2] = dea_ref[8]*sin(dea_ref[9]*t + dea_ref[11]) + dea_ref[10];

    // apply controller
    DSLSDEAController(state22, dea_k_, dea_param_, dea_ref.data(), t, F1, F2, xi_dot);
    dea_xi4.header.stamp = this->get_clock()->now();
    for(int i = 0; i < 4; i ++){
        dea_xi4.dea_xi_dot4[i] = xi_dot[i];
    }

    /* Infill DEA Force Msg */
    dea_force_0_.header.stamp = this->get_clock()->now();
    dea_force_0_.vector.x = F1[0];
    dea_force_0_.vector.y = F1[1];
    dea_force_0_.vector.z = F1[2];

    dea_force_1_.header.stamp = this->get_clock()->now();
    dea_force_1_.vector.x = F2[0];
    dea_force_1_.vector.y = F2[1];
    dea_force_1_.vector.z = F2[2];

    force_rate_convert(F1, attitude_dea_0_, rate_dea_0_, 0);
    force_rate_convert(F2, attitude_dea_1_, rate_dea_1_, 1);

    /* Controller State Integration */
    if(dea_enabled_ || dea_preintegrate_enabled_){
        for(int j = 0; j < 4; j ++){
            dea_xi4_.dea_xi4[j] += xi_dot[j]*diff_time; // Euler
            if(dea_xi4.dea_xi4[j] > max_xi_value_ || std::isnan(dea_xi4.dea_xi4[j])){ // dea_xi4.dea_xi4[j]
                reset_controller_state = true;
                break;
            } 
        }         
    }
 
    if(reset_controller_state){
        for(int i = 0; i<4; i++) dea_xi4.dea_xi4[i] = dea_xi4_ic_[i]; //dea_xi4.dea_xi4
        RCLCPP_INFO(this->get_logger(),"[applyDSLSDEA] xi reset detected");
    }      

    return 0;
}

void DSLS_DEA::force_rate_convert(double controller_output[3], VehicleAttitudeSetpoint &attitude, VehicleRatesSetpoint &rate,int uav){
    double thrust_norm_hover = 0.538;
    double thrust_coeff = 100;  
    double thrust_0 = 1.55*9.81;
    double max_force = (uav_mass_ + 0.5 * load_mass_) * max_fb_acc_ * 2; //
    double force_norm = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);

    /* Attitude */
    // Getting Current Attitude
    Quaternion quat_msg;
    double curr_roll, curr_pitch, curr_yaw;    
    if(uav == 0) quat_msg = uav0_pose_.pose.orientation;
    else if(uav == 1) quat_msg = uav1_pose_.pose.orientation;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(quat_msg, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(curr_roll, curr_pitch, curr_yaw);
    // bool curr_rpy_debug_enabled = false;
    // if(curr_rpy_debug_enabled) ROS_INFO_STREAM("Current RPY:" << curr_roll << " " << curr_pitch << " " << curr_yaw);

    // Clip reference force
    if(force_norm > max_force && force_clip_enabled_){
        for(int i=0; i<3; i++) controller_output[i] *= max_force / force_norm;
    }


    // Getting ENU Attitude Target (mavros will do the conversion to NED)
    attitude.timestamp = this->get_clock()->now().nanoseconds()/1000;
    double ref_roll, ref_pitch, ref_yaw, ref_thrust;
    ref_thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    ref_roll = std::asin((std::cos(curr_yaw) * controller_output[1] - std::sin(curr_yaw) * controller_output[0])/ref_thrust);
    ref_pitch = std::atan2(std::cos(curr_yaw) * controller_output[0] + std::sin(curr_yaw) * controller_output[1], -controller_output[2]);
    ref_yaw = 0;

    // Restrict Maximum Tilt
    if(std::abs(ref_roll) > max_tilt_angle_) ref_roll = std::copysign(ref_roll, max_tilt_angle_);
    if(std::abs(ref_pitch) > max_tilt_angle_) ref_pitch = std::copysign(ref_pitch, max_tilt_angle_);

    tf2::Quaternion attitude_target_q;
    //bool ref_rpy_debug_enabled = false;
    //if(ref_rpy_debug_enabled) ROS_INFO_STREAM("Target RPY:" << ref_roll << " " << ref_pitch << " " << ref_yaw);
    attitude_target_q.setRPY(ref_roll, ref_pitch, ref_yaw);
    // attitude.orientation.x = attitude_target_q.getX();
    // attitude.orientation.y = attitude_target_q.getY();
    // attitude.orientation.z = attitude_target_q.getZ();
    // attitude.orientation.w = attitude_target_q.getW(); // these are for ros1 
    // for ros2:
    Eigen::Quaterniond target_att_enu(attitude_target_q.getW(), attitude_target_q.getX(),attitude_target_q.getY(),attitude_target_q.getZ());
    Eigen::Quaterniond target_att_ned = ros_to_px4_orientation(target_att_enu);
    attitude.q_d[0] = static_cast<float>(target_att_ned.w());
    attitude.q_d[1] = static_cast<float>(target_att_ned.x());
    attitude.q_d[2] = static_cast<float>(target_att_ned.y());
    attitude.q_d[3] = static_cast<float>(target_att_ned.z());

    /* Rate */
    Eigen::Vector4d curr_att;
    Eigen::Vector4d ref_att;

    if (uav == 0){
        curr_att(0) = uav0_pose_.pose.orientation.w; 
        curr_att(1) = uav0_pose_.pose.orientation.x; 
        curr_att(2) = uav0_pose_.pose.orientation.y; 
        curr_att(3) = uav0_pose_.pose.orientation.z; 
    }
    else if (uav == 1){
        curr_att(0) = uav1_pose_.pose.orientation.w; 
        curr_att(1) = uav1_pose_.pose.orientation.x; 
        curr_att(2) = uav1_pose_.pose.orientation.y; 
        curr_att(3) = uav1_pose_.pose.orientation.z; 
    }
    ref_att(0) = attitude_target_q.getW();
    ref_att(1) = attitude_target_q.getX();
    ref_att(2) = attitude_target_q.getY();
    ref_att(3) = attitude_target_q.getZ();

    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att); 
    // attitude.body_rate.x = (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    // attitude.body_rate.y = (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    // attitude.body_rate.z = (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3); // for ros1
    // for ros2
    rate.timestamp = this->get_clock()->now().nanoseconds()/1000;
    Eigen::Vector3d body_rate_flu((2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1), (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2), (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3));  
    Eigen::Vector3d body_rate_frd = baselink_to_aircraft_body_frame(body_rate_flu);
    rate.roll  = static_cast<float>(body_rate_frd(0));
    rate.pitch = static_cast<float>(body_rate_frd(1));
    rate.yaw   = static_cast<float>(body_rate_frd(2));
    
    /* Thrust */ 
    double thrust = std::max(0.0, std::min(1.0, ref_thrust / (max_thrust_force_) + norm_thrust_offset_));
    attitude.thrust_body[0] = 0.0f;
    attitude.thrust_body[1] = 0.0f;
    attitude.thrust_body[2] = static_cast<float>(-thrust); 

    rate.thrust_body[0] = 0.0f;
    rate.thrust_body[1] = 0.0f;
    rate.thrust_body[2] = static_cast<float>(-thrust); 
}

Vector3 DSLS_DEA::crossProduct(const Vector3 v1, const Vector3 v2) {
    Vector3 result;
    result.x = v1.y * v2.z - v1.z * v2.y; 
    result.y = v1.z * v2.x - v1.x * v2.z; 
    result.z = v1.x * v2.y - v1.y * v2.x; 
    return result; 
}

double DSLS_DEA::applyLPF(double u, double u_last, double u_last_2, double y_last, double y_last_2, double Ts, double tau, double xi, double omega, int order){
    double y; // y(k)
    if(order == 1){ // 1st Order LPF
        double a = 2 / (Ts + 2 * tau);
        double b = (2 * tau - Ts) / (Ts + 2 * tau);
        y = a * (u - u_last) + b * y_last;
        if(lpf_debug_enabled_) printf("[LPF] 1st Order LPF Complete\n");
        return y;
    }
    else if(order == 2){ // 2nd Order LPF
        double k1 = 4 + 4 * xi * omega * Ts + omega * omega * Ts * Ts;
        double k2 = -8 + 2 * omega * omega * Ts * Ts;
        double k3 = 4 - 4 * xi * omega * Ts + omega * omega * Ts * Ts;
        double k4 = 2 * omega * omega * Ts;
        y = k4 / k1 * (u - u_last_2) - k2 / k1 * y_last - k3 / k1 * y_last_2;
        if(lpf_debug_enabled_) printf("[LPF] 2nd Order LPF Complete\n");
        return y;
    }
    else {
        printf("[LPF][Error] Order Error Detected\n");
    }
    return 1;
}

Vector3 DSLS_DEA::applyLPFVector3(Vector3 v, Vector3 v_last, Vector3 v_last_2, Vector3 v_dot_last, Vector3 v_dot_last_2, double diff_time, double tau, double xi, double omega, int order){
    Vector3 v_dot;
    v_dot.x = applyLPF(v.x, v_last.x, v_last_2.x, v_dot_last.x, v_dot_last_2.x, diff_time, tau, xi, omega, order);
    v_dot.y = applyLPF(v.x, v_last.y, v_last_2.y, v_dot_last.y, v_dot_last_2.y, diff_time, tau, xi, omega, order);
    v_dot.z = applyLPF(v.x, v_last.z, v_last_2.z, v_dot_last.z, v_dot_last_2.z, diff_time, tau, xi, omega, order);
    return v_dot;
}

Vector3 DSLS_DEA::applyFiniteDiffVector3(Vector3 v, Vector3 v_last, Vector3 v_dot_last, double diff_time){
    Vector3 v_dot;
    v_dot.x = applyFiniteDiff(v.x, v_last.x, v_dot_last.x, diff_time);
    v_dot.y = applyFiniteDiff(v.y, v_last.y, v_dot_last.y, diff_time);
    v_dot.z = applyFiniteDiff(v.z, v_last.z, v_dot_last.z, diff_time);
    return v_dot;
}

double DSLS_DEA::applyFiniteDiff(double x, double x_last, double x_dot_last, double diff_time){
    bool fd_debug_enabled = false;
    double x_dot;
    if(diff_time < DBL_MIN) {
        printf("[FiniteDiff][Fatal]: Time Step Too Small\n");
        return x_dot_last;
    }

    else if(x - x_last == 0){
        printf("f\n");
        return x_dot_last;
    }
    else{
        x_dot = (x - x_last) / diff_time;
        if(fd_debug_enabled) printf("[FiniteDiff] Finite Difference Complete\n");
        return x_dot;
    }
}

int DSLS_DEA::applyOpenLoopController(void){
    
    return 0;
}

int DSLS_DEA::enu2ned(void){
    uav0_pose_.pose.position.x = uav0_pose_.pose.position.y;
    uav0_pose_.pose.position.y = uav0_pose_.pose.position.x; 
    uav0_pose_.pose.position.z = -uav0_pose_.pose.position.z; 

    uav1_pose_.pose.position.x = uav1_pose_.pose.position.y; 
    uav1_pose_.pose.position.y = uav1_pose_.pose.position.x; 
    uav1_pose_.pose.position.z = -uav1_pose_.pose.position.z; 

    uav0_twist_.twist.linear.x = uav0_twist_.twist.linear.y;
    uav0_twist_.twist.linear.y = uav0_twist_.twist.linear.x;
    uav0_twist_.twist.linear.z = -uav0_twist_.twist.linear.z;

    uav1_twist_.twist.linear.x = uav1_twist_.twist.linear.y;
    uav1_twist_.twist.linear.y = uav1_twist_.twist.linear.x;
    uav1_twist_.twist.linear.z = -uav1_twist_.twist.linear.z;

    load_pose_.pose.position.x = load_pose_.pose.position.y; 
    load_pose_.pose.position.y = load_pose_.pose.position.x; 
    load_pose_.pose.position.z = -load_pose_.pose.position.z;    

    load_twist_.twist.linear.x = load_twist_.twist.linear.y;
    load_twist_.twist.linear.y = load_twist_.twist.linear.x;
    load_twist_.twist.linear.z = -load_twist_.twist.linear.z;

    return 0;
}

int DSLS_DEA::enu2esd(void){
    uav0_pose_.pose.position.y = -uav0_pose_.pose.position.y; 
    uav0_pose_.pose.position.z = -uav0_pose_.pose.position.z; 

    uav1_pose_.pose.position.y = -uav1_pose_.pose.position.y; 
    uav1_pose_.pose.position.z = -uav1_pose_.pose.position.z; 

    uav0_twist_.twist.linear.y = -uav0_twist_.twist.linear.y;
    uav0_twist_.twist.linear.z = -uav0_twist_.twist.linear.z;

    uav1_twist_.twist.linear.y = -uav1_twist_.twist.linear.y;
    uav1_twist_.twist.linear.z = -uav1_twist_.twist.linear.z;

    load_pose_.pose.position.y = -load_pose_.pose.position.y; 
    load_pose_.pose.position.z = -load_pose_.pose.position.z;    

    load_twist_.twist.linear.y = -load_twist_.twist.linear.y;
    load_twist_.twist.linear.z = -load_twist_.twist.linear.z;

    return 0;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DSLS_DEA>());

	rclcpp::shutdown();
	return 0;
}
