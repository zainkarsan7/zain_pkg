#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_opts;
    node_opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("rob_loader",node_opts);
    const auto& LOGGER = node->get_logger();



    // load robot given params from the launch.py
    robot_model_loader::RobotModelLoader robot_loader(node);
    const moveit::core::RobotModelPtr& k_model = robot_loader.getModel();
    std::vector<std::string> names = k_model->getJointModelGroupNames();
    for (std::size_t i=0; i<names.size(); ++i){
        RCLCPP_INFO(LOGGER,"model has these groups  %s",names[i].c_str());
    }
    
    const moveit::core::JointModelGroup* joint_model_group = k_model->getJointModelGroup("panda_arm");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    
    moveit::core::RobotStatePtr rob_state(new moveit::core::RobotState(k_model));
    rob_state->setToDefaultValues();
    rob_state->setToRandomPositions(joint_model_group);
    std::vector<double> j_vals;
    rob_state->copyJointGroupPositions("panda_arm",j_vals);
    for (std::size_t i=0;i<joint_names.size();++i){
        RCLCPP_INFO(LOGGER, "these are the j values for %s: %f",joint_names[i].c_str(),j_vals[i]);
    }

    const Eigen::Isometry3d& eef_state = rob_state->getGlobalLinkTransform("panda_link8");
    RCLCPP_INFO_STREAM(LOGGER,"Translation: \n"<<eef_state.translation()<<"\n");
    RCLCPP_INFO_STREAM(LOGGER,"Rotation: \n"<<eef_state.rotation()<<"\n");

    double timeout = 0.1;
    rob_state->setToDefaultValues();
    bool found_ik = rob_state->setFromIK(joint_model_group,eef_state,timeout);

    if (found_ik){
        rob_state->copyJointGroupPositions(joint_model_group,j_vals);
        for (std::size_t i=0;i<joint_names.size();++i){
            RCLCPP_INFO(LOGGER, "joint %s, %f",joint_names[i].c_str(),j_vals[i]);
        }
    }else{
        RCLCPP_INFO(LOGGER,"couldnt find soln");
    }


    rclcpp::shutdown();
    return 0;
}