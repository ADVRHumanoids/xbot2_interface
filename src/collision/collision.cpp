#include "collision.hxx"
#include "../impl/utils.h"

#include <geometric_shapes/mesh_operations.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <fmt/format.h>

using namespace XBot::Collision;

fcl::Transform3f tofcl(const Eigen::Affine3d& T)
{
    fcl::Transform3f ret;
    ret.rotation() = T.linear();
    ret.translation() = T.translation();
    return ret;
}

fcl::Transform3f tofcl(const urdf::Pose& T)
{
    return fcl::Transform3f(fcl::Quaternion3f(T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z),
                            fcl::Vec3f(T.position.x, T.position.y, T.position.z));
}

Eigen::Affine3d toeigen(const urdf::Pose& T)
{
    Eigen::Affine3d ret;
    ret.translation() << T.position.x, T.position.y, T.position.z;
    ret.linear() = Eigen::Quaterniond(T.rotation.w, T.rotation.x, T.rotation.y, T.rotation.z).toRotationMatrix();
    return ret;
}

CollisionModel::Impl::Impl(ModelInterface::ConstPtr model):
    _model(model)
{
    parseCollisionObjects();

    generateAllPairs();

    removeDisabledPairs();

    updateCollisionPairData();
}

/**
 * @brief capsule_from_collision checks if the link collision is
 * formed by a cylinder and two spheres, and it returns the cylinder
 * collision shared pointer if that is true (nullptr otherwise)
 */
urdf::CollisionConstSharedPtr capsule_from_collision(urdf::Link& l)
{
    if(l.collision_array.size() != 3)
    {
        return nullptr;
    }

    int num_spheres = 0;
    urdf::CollisionConstSharedPtr cylinder;

    for(auto c : l.collision_array)
    {
        if(c->geometry->type == urdf::Geometry::CYLINDER)
        {
            cylinder = c;
        }
        else if(c->geometry->type == urdf::Geometry::SPHERE)
        {
            num_spheres++;
        }
    }

    if(cylinder && num_spheres == 2)
    {
        return cylinder;
    }

    return nullptr;
}

bool CollisionModel::Impl::parseCollisionObjects()
{
    // get urdf links
    std::vector<urdf::LinkSharedPtr> links;
    _model->getUrdf()->getLinks(links);

    // loop over links
    for(auto link : links)
    {
        // no collision defined, skip
        if(!link->collision)
        {
            std::cout << "collision not defined for link " << link->name << std::endl;
            _link_collision_map[link->name];
            continue;
        }

        // convert urdf collision to fcl shape
        std::shared_ptr<fcl::CollisionGeometry> shape;
        Eigen::Affine3d shape_origin;

        if(auto cylinder = capsule_from_collision(*link))
        {
            std::cout << "adding capsule for " << link->name << std::endl;

            auto collisionGeometry =
                std::dynamic_pointer_cast<urdf::Cylinder>(cylinder->geometry);

            shape = std::make_shared<fcl::Capsule>(collisionGeometry->radius,
                                                   collisionGeometry->length);

            shape_origin = toeigen(cylinder->origin);

        }
        else if(link->collision->geometry->type == urdf::Geometry::CYLINDER)
        {
            std::cout << "adding cylinder for " << link->name << std::endl;

            auto collisionGeometry =
                std::dynamic_pointer_cast<urdf::Cylinder>(link->collision->geometry);

            shape = std::make_shared<fcl::Cylinder>(collisionGeometry->radius,
                                                    collisionGeometry->length);

            shape_origin = toeigen(link->collision->origin);

            // note: check following line (for capsules it looks to
            // generate wrong results)
            // shape_origin.p -= collisionGeometry->length/2.0 * shape_origin.M.UnitZ();

        }
        else if(link->collision->geometry->type == urdf::Geometry::SPHERE)
        {
            std::cout << "adding sphere for " << link->name << std::endl;

            auto collisionGeometry =
                std::dynamic_pointer_cast<urdf::Sphere>(link->collision->geometry);

            shape = std::make_shared<fcl::Sphere>(collisionGeometry->radius);
            shape_origin = toeigen(link->collision->origin);
        }
        else if ( link->collision->geometry->type == urdf::Geometry::BOX )
        {
            std::cout << "adding box for " << link->name << std::endl;

            auto collisionGeometry =
                std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);

            shape = std::make_shared<fcl::Box>(collisionGeometry->dim.x,
                                               collisionGeometry->dim.y,
                                               collisionGeometry->dim.z);

            shape_origin = toeigen(link->collision->origin);

        }
        else if(link->collision->geometry->type == urdf::Geometry::MESH)
        {
            std::cout << "adding mesh for " << link->name << std::endl;

            auto collisionGeometry =
                std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);

            auto mesh = shapes::createMeshFromResource(collisionGeometry->filename);

            if(!mesh)
            {
                std::cout << "Error loading mesh for link " << link->name << std::endl;
                continue;
            }

            std::vector<fcl::Vec3f> vertices;
            std::vector<fcl::Triangle> triangles;

            for(unsigned int i = 0; i < mesh->vertex_count; ++i)
            {
                fcl::Vec3f v(mesh->vertices[3*i]*collisionGeometry->scale.x,
                             mesh->vertices[3*i + 1]*collisionGeometry->scale.y,
                             mesh->vertices[3*i + 2]*collisionGeometry->scale.z);

                vertices.push_back(v);
            }

            for(unsigned int i = 0; i< mesh->triangle_count; ++i)
            {
                fcl::Triangle t(mesh->triangles[3*i],
                                mesh->triangles[3*i + 1],
                                mesh->triangles[3*i + 2]);

                triangles.push_back(t);
            }

            // add the mesh data into the BVHModel structure
            auto bvhModel = std::make_shared<fcl::BVHModel<fcl::OBBRSS>>();
            shape = bvhModel;
            bvhModel->beginModel();
            bvhModel->addSubModel(vertices, triangles);
            bvhModel->endModel();

            shape_origin = toeigen(link->collision->origin);
        }

        if(!shape)
        {
            std::cerr << "collision type unknown for link " << link->name << std::endl;
            continue;
        }

        // compute local axis aligned bounding box (used to discard far apart shapes)
        shape->computeLocalAABB();

        // save parsed shapes for this link (TBD support multiple shapes)
        _link_collision_map[link->name] = LinkCollision{.geom{shape}, .l_T_shape{shape_origin}};
    }



    return true;
}

void CollisionModel::Impl::generateAllPairs()
{
    // get urdf links
    std::vector<urdf::LinkSharedPtr> links;
    _model->getUrdf()->getLinks(links);

    _active_link_pairs.clear();

    for(int i = 0; i < links.size(); i++)
    {
        if(!links[i]->collision)
        {
            continue;
        }

        for(int j = i+1; j < links.size(); j++)
        {
            if(!links[j]->collision)
            {
                continue;
            }

            _active_link_pairs.emplace_back(links[i]->name, links[j]->name);
        }
    }
}

void CollisionModel::Impl::removeDisabledPairs()
{
    auto is_disabled = [this](const LinkPair &pair) {

        if(!_model->getSrdf())
        {
            return false;
        }

        auto is_same_pair_srdf = [pair](const srdf::Model::CollisionPair &p) {
            return (p.link1_ == pair.first && p.link2_ == pair.second) ||
                   (p.link1_ == pair.second && p.link2_ == pair.first);
        };

        auto srdf_disabled_pairs = _model->getSrdf()->getDisabledCollisionPairs();

        bool srdf_disabled = std::find_if(srdf_disabled_pairs.begin(),
                                          srdf_disabled_pairs.end(),
                                          is_same_pair_srdf) != srdf_disabled_pairs.end();

        return srdf_disabled;
    };

    std::erase_if(_active_link_pairs, is_disabled);
}

void CollisionModel::Impl::updateCollisionPairData()
{
    _collision_pair_data.clear();

    int pidx = 0;

    for(auto [l1, l2] : _active_link_pairs)
    {
        // check link exists
        if(_model->getLinkId(l1) < 0)
        {
            throw std::out_of_range(
                fmt::format("link '{}' does not exist within model", l1));
        }

        if(_model->getLinkId(l2) < 0)
        {
            throw std::out_of_range(
                fmt::format("link '{}' does not exist within model", l2));
        }

        auto c1 = _link_collision_map.at(l1);
        auto c2 = _link_collision_map.at(l2);

        for(int i = 0; i < c1.geom.size(); i++)
        {
            for(int j = 0; j < c2.geom.size(); j++)
            {

                CollisionPairData cpd(c1.geom[i], c2.geom[j]);
                cpd.link1 = l1;
                cpd.link2 = l2;
                cpd.id1 = _model->getLinkId(l1);
                cpd.id2 = _model->getLinkId(l2);
                cpd.l_T_shape1 = c1.l_T_shape[i];
                cpd.l_T_shape2 = c2.l_T_shape[j];
                cpd.request.enable_nearest_points = true;

                _collision_pair_data.emplace_back(std::move(cpd));



                fmt::print("added pair i = {}: {} vs {} \n",
                           pidx++, l1, l2);

            }
        }
    }
}

CollisionModel::CollisionModel(ModelInterface::ConstPtr model):
    impl(std::make_unique<Impl>(model))
{}

int CollisionModel::getNumCollisionPairs() const
{
    return impl->_collision_pair_data.size();
}

std::vector<std::pair<std::string, std::string>> CollisionModel::getCollisionPairs() const
{
    std::vector<std::pair<std::string, std::string>> ret;
    ret.reserve(impl->_collision_pair_data.size());

    for(const auto& item : impl->_collision_pair_data)
    {
        ret.emplace_back(item.link1, item.link2);
    }

    return ret;

}

std::vector<std::pair<std::string, std::string>> CollisionModel::getLinkPairs() const
{
    return impl->_active_link_pairs;
}

void CollisionModel::setLinkPairs(std::vector<std::pair<std::string, std::string>> pairs)
{
    impl->_active_link_pairs = pairs;

    impl->updateCollisionPairData();
}

void CollisionModel::update(double threshold)
{
    for(auto& item : impl->_collision_pair_data)
    {
        item.compute(*impl->_model, threshold);
    }
}

std::vector<Eigen::Vector3d> CollisionModel::getNormals() const
{
    std::vector<Eigen::Vector3d> n;

    getNormals(n);

    return n;
}

void CollisionModel::getNormals(std::vector<Eigen::Vector3d> &n) const
{
    n.resize(getNumCollisionPairs());

    for(int i = 0; i < n.size(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        n[i] = cpd.result.normal;
    }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> CollisionModel::getWitnessPoints() const
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ret;

    getWitnessPoints(ret);

    return ret;

}

void CollisionModel::getWitnessPoints(
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &wp) const
{
    wp.resize(getNumCollisionPairs());

    for(int i = 0; i < wp.size(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        wp[i].first = cpd.result.nearest_points[0];
        wp[i].second = cpd.result.nearest_points[1];
    }
}

Eigen::VectorXd CollisionModel::getDistance() const
{
    Eigen::VectorXd ret;

    getDistance(ret);

    return ret;
}

void CollisionModel::getDistance(Eigen::VectorXd& d) const
{
    d.resize(getNumCollisionPairs());

    for(int i = 0; i < d.size(); i++)
    {
        d[i] = impl->_collision_pair_data[i].result.min_distance;
    }
}

Eigen::MatrixXd CollisionModel::getDistanceJacobian() const
{
    Eigen::MatrixXd J(getNumCollisionPairs(), impl->_model->getNv());

    getDistanceJacobian(J);

    return J;
}

void CollisionModel::getDistanceJacobian(MatRef J) const
{
    check_mat_size(J, getNumCollisionPairs(), impl->_model->getNv(), __func__);

    auto& model = *impl->_model;

    impl->_Jtmp.resize(6, model.getNv());

    for(int i = 0; i < J.rows(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        model.getJacobian(cpd.id1, cpd.result.nearest_points[0], impl->_Jtmp);
        J.row(i).noalias() = -cpd.result.normal.transpose() * impl->_Jtmp.topRows<3>();

        model.getJacobian(cpd.id2, cpd.result.nearest_points[1], impl->_Jtmp);
        J.row(i).noalias() += cpd.result.normal.transpose() * impl->_Jtmp.topRows<3>();

    }

}

CollisionModel::~CollisionModel()
{

}

CollisionModel::Impl::CollisionPairData::CollisionPairData(
    std::shared_ptr<fcl::CollisionGeometry> _g1, std::shared_ptr<fcl::CollisionGeometry> _g2):
    dist(_g1.get(), _g2.get()),
    g1(_g1), g2(_g2)
{

}

void CollisionModel::Impl::CollisionPairData::compute(const ModelInterface &model,
                                                      double threshold)
{
    auto w_T_l1 = model.getPose(id1);

    auto w_T_l2 = model.getPose(id2);

    auto w_T_shape1 = w_T_l1 * l_T_shape1;

    auto w_T_shape2 = w_T_l2 * l_T_shape2;

    Eigen::Vector3d aabb_1 = w_T_shape1*g1->aabb_center;
    Eigen::Vector3d aabb_2 = w_T_shape2*g2->aabb_center;
    Eigen::Vector3d aabb_21 = aabb_2 - aabb_1;
    double aabb_dist = aabb_21.norm() - g1->aabb_radius - g2->aabb_radius;

    result.clear();

    if(threshold >0 && aabb_dist > threshold)
    {
        result.min_distance = aabb_dist;
        result.normal = aabb_21.normalized();
        result.nearest_points[0] = aabb_1 + result.normal;
        result.nearest_points[1] = aabb_2 - result.normal;
        return;
    }

    dist(tofcl(w_T_shape1),
         tofcl(w_T_shape2),
         request,
         result);


    // hack to fix wrong distance when in deep collision
    if(result.min_distance < 0)
    {
        result.min_distance = -(result.nearest_points[0] - result.nearest_points[1]).norm();
    }

    // convert witness points to local link frame
    result.nearest_points[0] = w_T_l1.inverse() * result.nearest_points[0];
    result.nearest_points[1] = w_T_l2.inverse() * result.nearest_points[1];



}
