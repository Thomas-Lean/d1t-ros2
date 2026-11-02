// Local, offline triangle-mesh proximity. JSON-lines stdin/stdout; no ROS node.
// FCL reports surface intersections, NOT a reliable mesh penetration depth.
// A parity containment check additionally catches a closed component fully inside
// another solid. Representatives are supplied for every connected component.
#include <fcl/fcl.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

using Json = nlohmann::json;
using Vec = fcl::Vector3d;
using Transform = fcl::Transform3d;
using Model = fcl::BVHModel<fcl::OBBRSSd>;

struct Geometry {
  std::shared_ptr<Model> model;
  std::vector<Vec> vertices;
  std::vector<fcl::Triangle> faces;
  std::vector<int> representatives;
};

Transform transform(const Json& j) {
  Transform t = Transform::Identity();
  for (int i = 0; i < 4; ++i)
    for (int k = 0; k < 4; ++k) t.matrix()(i,k) = j.at(i).at(k).get<double>();
  if (!t.matrix().allFinite() || std::abs(t.linear().determinant()-1) > 1e-6 ||
      (t.linear().transpose()*t.linear()-Eigen::Matrix3d::Identity()).norm() > 1e-6)
    throw std::runtime_error("invalid rigid transform");
  return t;
}

Json vector_json(const Vec& p) { return Json::array({p.x(),p.y(),p.z()}); }

bool inside(const Vec& p, const Geometry& mesh) {
  // Containment is impossible outside the local vertex bounds. Vendor STL
  // surfaces can be non-watertight: a single odd ray count alone is insufficient.
  if (mesh.vertices.empty()) return false;
  Vec lo = mesh.vertices.front(), hi = lo;
  for (const auto& v : mesh.vertices) { lo = lo.cwiseMin(v); hi = hi.cwiseMax(v); }
  if ((p.array() < lo.array()-1e-10).any() || (p.array() > hi.array()+1e-10).any())
    return false;
  const Vec dir = Vec(1,0.371390676,0.529128483).normalized();
  std::vector<double> hits;
  for (const auto& f : mesh.faces) {
    const Vec& a = mesh.vertices[f[0]];
    const Vec e1 = mesh.vertices[f[1]]-a, e2 = mesh.vertices[f[2]]-a;
    const Vec h = dir.cross(e2);
    const double det = e1.dot(h);
    if (std::abs(det) < 1e-14) continue;
    const Vec s = p-a;
    const double u = s.dot(h)/det;
    if (u < -1e-10 || u > 1+1e-10) continue;
    const Vec q = s.cross(e1);
    const double v = dir.dot(q)/det;
    if (v < -1e-10 || u+v > 1+1e-10) continue;
    const double t = e2.dot(q)/det;
    if (t > 1e-10) hits.push_back(t);
  }
  std::sort(hits.begin(),hits.end());
  auto end = std::unique(hits.begin(),hits.end(),[](double a,double b){return std::abs(a-b)<1e-9;});
  return std::distance(hits.begin(),end)%2 == 1;
}

Json query(const Geometry& a, const Geometry& b, const Transform& ta, const Transform& tb) {
  fcl::CollisionObjectd oa(a.model,ta), ob(b.model,tb);
  fcl::CollisionRequestd cr(64,true);
  fcl::CollisionResultd collision;
  fcl::collide(&oa,&ob,cr,collision);
  Json out = {{"collision",collision.isCollision()},{"contained",false},
              {"contacts_m",Json::array()}};
  if (collision.isCollision()) {
    for (size_t i=0;i<collision.numContacts();++i)
      out["contacts_m"].push_back(vector_json(collision.getContact(i).pos));
    out["distance_m"] = 0.0;
    // Penetration depth intentionally omitted: triangle contacts do not give a
    // trustworthy solid penetration measure for this mesh pair.
    return out;
  }
  fcl::DistanceRequestd dr(true,false,0,0);
  fcl::DistanceResultd result;
  fcl::distance(&oa,&ob,dr,result);
  if (!std::isfinite(result.min_distance) || result.min_distance < -1e-10)
    throw std::runtime_error("invalid FCL separation result");
  out["distance_m"] = std::max(0.0,result.min_distance);
  out["nearest_m"] = {vector_json(result.nearest_points[0]),vector_json(result.nearest_points[1])};
  if (result.min_distance > 1e-9 && oa.getAABB().overlap(ob.getAABB())) {
    for (int index : a.representatives) {
      const Vec p = ta*a.vertices.at(index);
      if (inside(tb.inverse()*p,b)) {
        out["collision"]=true; out["contained"]=true;
        out["distance_m"]=0.0; out["contacts_m"].push_back(vector_json(p));
        break;
      }
    }
    if (!out["contained"].get<bool>()) for (int index : b.representatives) {
      const Vec p = tb*b.vertices.at(index);
      if (inside(ta.inverse()*p,a)) {
        out["collision"]=true; out["contained"]=true;
        out["distance_m"]=0.0; out["contacts_m"].push_back(vector_json(p));
        break;
      }
    }
  }
  return out;
}

int main() {
  std::map<std::string,Geometry> meshes;
  std::string line;
  while (std::getline(std::cin,line)) {
    try {
      const Json request = Json::parse(line);
      if (request.contains("load")) {
        meshes.clear();
        for (const auto& item : request.at("load").items()) {
          Geometry g;
          for (const auto& v : item.value().at("vertices"))
            g.vertices.emplace_back(v.at(0).get<double>(),v.at(1).get<double>(),v.at(2).get<double>());
          for (const auto& f : item.value().at("faces"))
            g.faces.emplace_back(f.at(0).get<int>(),f.at(1).get<int>(),f.at(2).get<int>());
          g.representatives = item.value().at("representatives").get<std::vector<int>>();
          g.model = std::make_shared<Model>();
          g.model->beginModel(); g.model->addSubModel(g.vertices,g.faces); g.model->endModel();
          meshes.emplace(item.key(),std::move(g));
        }
        std::cout << Json({{"loaded",meshes.size()}}).dump() << std::endl;
      } else {
        Json results = Json::array();
        for (const auto& q : request.at("queries"))
          results.push_back(query(meshes.at(q.at("a").get<std::string>()),
                                  meshes.at(q.at("b").get<std::string>()),
                                  transform(q.at("ta")),transform(q.at("tb"))));
        std::cout << results.dump() << std::endl;
      }
    } catch (const std::exception& e) {
      std::cout << Json({{"error",e.what()}}).dump() << std::endl;
      return 1;
    }
  }
  return 0;
}
