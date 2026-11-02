// Pure in-process MoveIt check: no rclcpp::init, Node, DDS, or motion clients.
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using Json=nlohmann::json;
geometry_msgs::msg::Pose pose(const Json& matrix) {
  Eigen::Matrix3d r;
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)r(i,j)=matrix[i][j];
  Eigen::Quaterniond q(r);
  geometry_msgs::msg::Pose p;
  p.position.x=matrix[0][3];p.position.y=matrix[1][3];p.position.z=matrix[2][3];
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
  return p;
}
moveit_msgs::msg::CollisionObject object(const Json& entry) {
  moveit_msgs::msg::CollisionObject o;
  o.id=entry.at("id");o.header.frame_id=entry.value("frame","base_link");
  o.operation=o.ADD;o.pose.orientation.w=1.;
  if(entry.contains("size")) {
    shape_msgs::msg::SolidPrimitive box;box.type=box.BOX;
    for(const auto& v:entry.at("size"))box.dimensions.push_back(v.get<double>());
    o.primitives.push_back(box);o.primitive_poses.push_back(pose(entry.at("matrix")));
  } else {
    shape_msgs::msg::Mesh mesh;
    for(const auto& v:entry.at("vertices")) { geometry_msgs::msg::Point p;p.x=v[0];p.y=v[1];p.z=v[2];mesh.vertices.push_back(p); }
    for(const auto& f:entry.at("faces")) { shape_msgs::msg::MeshTriangle t;for(int i=0;i<3;i++)t.vertex_indices[i]=f[i];mesh.triangles.push_back(t); }
    o.meshes.push_back(mesh);o.mesh_poses.push_back(pose(entry.at("matrix")));
  }
  return o;
}
int main(int argc,char** argv) {
  if(argc!=2){std::cerr<<"usage: grasp01_moveit_offline input.json\n";return 2;}
  try {
    Json data;std::ifstream file(argv[1]);file>>data;
    auto urdf=urdf::parseURDFFile(data.at("urdf").get<std::string>());
    if(!urdf)throw std::runtime_error("URDF failed");
    auto srdf=std::make_shared<srdf::Model>();
    if(!srdf->initFile(*urdf,data.at("srdf").get<std::string>()))throw std::runtime_error("SRDF failed");
    auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    planning_scene::PlanningScene scene(model);
    for(const auto& e:data.at("world"))if(!scene.processCollisionObjectMsg(object(e)))throw std::runtime_error("world object failed");
    for(const auto& e:data.at("attached")) {
      moveit_msgs::msg::AttachedCollisionObject a;a.link_name=e.at("frame");a.object=object(e);a.touch_links=e.at("touch_links").get<std::vector<std::string>>();
      if(!scene.processAttachedCollisionObjectMsg(a))throw std::runtime_error("attachment failed");
    }
    Json results=Json::array();
    for(const auto& sample:data.at("states")) {
      auto& state=scene.getCurrentStateNonConst();
      for(auto it=sample.at("joints").begin();it!=sample.at("joints").end();++it)state.setVariablePosition(it.key(),it.value().get<double>());
      state.update();
      collision_detection::CollisionRequest req;req.contacts=true;req.max_contacts=1000;req.max_contacts_per_pair=8;req.distance=true;
      collision_detection::CollisionResult res;scene.checkCollision(req,res,state);
      Json contacts=Json::array();
      for(const auto& pair:res.contacts)for(const auto& c:pair.second)contacts.push_back({{"pair",{pair.first.first,pair.first.second}},{"position_m",{c.pos.x(),c.pos.y(),c.pos.z()}}});
      results.push_back({{"label",sample.at("label")},{"collision",res.collision},{"bounds_ok",state.satisfiesBounds()},{"library_reported_distance_m",res.distance},{"contacts",contacts}});
    }
    std::cout<<Json({{"results",results},{"no_ros_node",true}}).dump(2)<<std::endl;
    return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 2;}
}
