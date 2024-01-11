
#include "rclcpp/rclcpp.hpp"
#include <map>
#include <perception_kit_msgs/msg/objects.hpp>
#include <random>
#include <chrono>

using namespace std;
using namespace perception_kit_msgs::msg;
using namespace std::chrono_literals;

struct SimulatedObject
{
  SimulatedObject(std::string const& obj_class, float width, float length, float height, float x_offset)
    : obj_class(obj_class), width(width), length(length), height(height), x_offset(x_offset){};

  std::string obj_class;
  float width{ 0.0 };
  float length{ 0.0 };
  float height{ 0.0 };
  float x_offset{ 0.0 };
  Object object;
};

class ObjectSimulation : public rclcpp::Node
{
public:
  ObjectSimulation();

private:
  void createObjects();
  void publishObjects(size_t counter);
  rclcpp::Publisher<Objects>::SharedPtr publisher_;
  vector<Object> objects_;
};

ObjectSimulation::ObjectSimulation() : Node("ObjectSimulation")
{
  publisher_ = this->create_publisher<Objects>("/perception/simulation/tracks", 10);

  rclcpp::WallRate rate(100ms);
  createObjects();
  size_t counter{ 0 };
  while (rclcpp::ok())
  {
    publishObjects(counter);
    ++counter;
    rate.sleep();
  }
}

void ObjectSimulation::publishObjects(size_t counter)
{
  auto const ros_time_now = this->now();

  Objects objects;
  //objects.header.seq = counter; //seq is removed from header in ROS2
  objects.header.frame_id = "layered_map_enu";
  objects.header.stamp = ros_time_now;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(0.09, 0.11);
  std::uniform_real_distribution<> dis_y(-0.01, 0.01);

  size_t const iterations = 400;
  for (auto& object : objects_)
  {
    //object.header.seq = counter;
    object.header.stamp = ros_time_now;
    object.position.x += dis_x(gen);
    object.position.y += dis_y(gen);
    if (counter % iterations == 0)
    {
      object.id += objects_.size();
      object.position.x -= 0.1 * iterations;
    }
    objects.objects.push_back(object);
  }
  publisher_->publish(objects);
}

void ObjectSimulation::createObjects()
{
  map<size_t, SimulatedObject> const classes = { { 0, { string("car"), 2.0, 4.5, 1.5, -1.5 } },
                                                 { 1, { string("pedestrian"), 0.4, 0.4, 1.8, 0.0 } },
                                                 { 2, { string("bicycle"), 0.5, 1.75, 1.6, 0.0 } },
                                                 { 3, { string("unknown"), 0.7, 0.5, 1.1, 0.0 } } };

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis_x(-30.0, 30.0);
  std::uniform_real_distribution<> dis_y(-0.25, 0.25);
  std::uniform_int_distribution<> dis_lane(-2, 2);
  std::uniform_real_distribution<> dis_rainbow(0.0, 1.0);

  size_t k = 0;
  for (size_t j = 0; j < 3; ++j)
  {
    for (size_t i = 0; i < classes.size(); ++i)
    {
      ++k;
      SimulatedObject sim = classes.at(i);

      Object& object = sim.object;
      object.header.frame_id = "layered_map_enu";

      object.id = k;
      object.existence_probability = 0.95;

      object.width = sim.width;
      object.length = sim.length;
      object.height = sim.height;
      object.x_offset = sim.x_offset;

      Classification classification;
      classification.obj_class = sim.obj_class;
      classification.confidence = 0.85;
      object.classification.push_back(classification);

      Attribute moving;
      moving.name = "is_moving";
      moving.value.push_back(0.85);
      object.attributes.push_back(moving);

      Attribute rainbow;
      rainbow.name = "rainbow";
      rainbow.value.push_back(dis_rainbow(gen));
      object.attributes.push_back(rainbow);

      object.position.x = dis_x(gen);
      object.position.y = dis_lane(gen) * 4.0 + dis_y(gen);

      object.velocity.x = 3.5;

      object.covariance[Object::ELEMENT_POSITION_X * Object::NUM_COVARIANCE_ELEMENTS + Object::ELEMENT_POSITION_X] =
          1.6;
      object.covariance[Object::ELEMENT_POSITION_Y * Object::NUM_COVARIANCE_ELEMENTS + Object::ELEMENT_POSITION_Y] =
          1.2;

      objects_.push_back(object);
    }
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectSimulation>());
  rclcpp::shutdown();
  return 0;
}
