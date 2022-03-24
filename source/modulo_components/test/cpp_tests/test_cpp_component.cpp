#include <gtest/gtest.h>
#include <rclcpp_components/component_manager.hpp>


class TestCPPComponent : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    if (rcutils_logging_set_logger_level("CPPComponent", RCUTILS_LOG_SEVERITY_DEBUG) != RCL_RET_OK) {
      RCLCPP_WARN(rclcpp::get_logger("CPPComponent"), "Could not set log level for CPPComponent");
    }
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    manager_ = std::make_shared<rclcpp_components::ComponentManager>(exec_);
    exec_->add_node(manager_);
  }
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<rclcpp_components::ComponentManager> manager_;
};


TEST_F(TestCPPComponent, test_component_load) {
  auto resources = manager_->get_component_resources("template_component_package");
  bool component_exists = false;
  for (auto& resource : resources) {
    if (resource.first == "template_component_package::CPPComponent") {
      component_exists = true;
      EXPECT_TRUE(rcpputils::fs::exists(rcpputils::fs::path(resource.second)));
      auto factory = manager_->create_component_factory(resource);
      rclcpp_components::NodeInstanceWrapper instance;
      EXPECT_NO_THROW(instance = factory->create_node_instance(rclcpp::NodeOptions()));
    }
  }
  EXPECT_TRUE(component_exists);
}