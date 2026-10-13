// Design diagnostic: installed NavFn and stock inflation, no controller tuning.
#include <fstream>
#include <iostream>
#include <vector>
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/lifecycle_node.hpp"

class InputLayer : public nav2_costmap_2d::Layer {
public:
  std::vector<unsigned char> data;
  void onInitialize() override {current_ = true;}
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double *a, double *b, double *c, double *d) override
  {*a=-6.5; *b=-6.5; *c=6.5; *d=6.5;}
  void updateCosts(nav2_costmap_2d::Costmap2D &m, int, int, int, int) override
  {std::copy(data.begin(), data.end(), m.getCharMap());}
};
int main(int argc, char **argv) {
  if (argc != 3) {std::cerr << "probe_routes grid.bin path.csv\n"; return 2;}
  rclcpp::init(argc, argv);
  auto node=std::make_shared<nav2_util::LifecycleNode>("route_design_probe");
  node->declare_parameter("inflation_layer.inflation_radius", 0.7);
  node->declare_parameter("inflation_layer.cost_scaling_factor", 3.0);
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(260,260,.05,-6.5,-6.5);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (int i=0;i<32;++i) {geometry_msgs::msg::Point p; p.x=.22*cos(i*2*M_PI/32); p.y=.22*sin(i*2*M_PI/32); footprint.push_back(p);}
  map.setFootprint(footprint);
  tf2_ros::Buffer tf(node->get_clock());
  auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto input=std::make_shared<InputLayer>();
  std::ifstream stream(argv[1],std::ios::binary);
  input->data=std::vector<unsigned char>(std::istreambuf_iterator<char>(stream), {});
  if (input->data.size()!=260*260) {std::cerr << "bad grid size\n"; return 2;}
  input->initialize(&map,"input",&tf,node,group); map.addPlugin(input);
  auto inflation=std::make_shared<nav2_costmap_2d::InflationLayer>();
  inflation->initialize(&map,"inflation_layer",&tf,node,group); map.addPlugin(inflation);
  map.setFootprint(footprint);
  map.updateMap(-4,-4,0);
  nav2_navfn_planner::NavFn planner(260,260);
  planner.setCostmap(map.getCostmap()->getCharMap(),true,true);
  int start[2]={50,50}, goal[2]={210,50};
  planner.setStart(start); planner.setGoal(goal);
  planner.calcNavFnDijkstra([]{return false;},true);
  int count=planner.calcPath(4*260);
  std::ofstream out(argv[2]); out << "x,y\n";
  for(int i=0;i<count;++i) out << -6.5+(planner.getPathX()[i]+.5)*.05 << ',' << -6.5+(planner.getPathY()[i]+.5)*.05 << '\n';
  std::cout << "path_points=" << count << std::endl;
  rclcpp::shutdown(); return count>0 ? 0:1;
}
