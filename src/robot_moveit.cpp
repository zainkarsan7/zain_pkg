#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main (int argc, char *argv []){
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    auto node = rclcpp::Node::make_shared("zains_robot",node_options);

    const auto& LOGGER = node->get_logger();


    robot_model_loader::RobotModelLoader rml(node);
    const moveit::core::RobotModelPtr& kin_model = rml.getModel();
    RCLCPP_INFO(LOGGER,"Model frame : %s", kin_model->getModelFrame().c_str());
    
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kin_model));
    robot_state->setToDefaultValues();

}

