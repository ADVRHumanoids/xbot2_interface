#include "collision.hxx"
#include "../impl/utils.h"

#include <xbot2_interface/common/utils.h>
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

    makeCollisionManager();

    _Jtmp.setZero(6, _model->getNv());
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

            _link_collision_map[link->name] =
                std::make_shared<LinkCollision>(*_model, link->name);

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
        _link_collision_map[link->name]
            = std::make_shared<LinkCollision>(*_model,
                                              link->name,
                                              std::vector{shape},
                                              std::vector{shape_origin});
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

            _active_link_pairs.insert({links[i]->name, links[j]->name});
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

        for(int i = 0; i < c1->coll_obj.size(); i++)
        {
            for(int j = 0; j < c2->coll_obj.size(); j++)
            {

                CollisionPairData cpd(c1->coll_obj[i], c2->coll_obj[j]);

                cpd.link1 = c1;
                cpd.link2 = c2;
                cpd.id1 = _model->getLinkId(l1);
                cpd.id2 = _model->getLinkId(l2);

                cpd.drequest.enable_nearest_points = true;
                // cpd.crequest.....

                _collision_pair_data.emplace_back(std::move(cpd));

                fmt::print("added pair i = {}: {} vs {} \n",
                           pidx++, l1, l2);

            }
        }
    }
}

void CollisionModel::Impl::makeCollisionManager()
{
    // _manager = std::make_unique<fcl::DynamicAABBTreeCollisionManager>();

    // for(auto& [lname, lc] : _link_collision_map)
    // {
    //     for(auto& obj : lc.coll_obj)
    //     {
    //         _manager->registerObject(&obj);
    //         obj.setUserData((void*)(&lname));
    //     }
    // }

    // _manager->setup();
}

bool CollisionModel::Impl::checkSelfCollision(std::vector<int>* coll_pair_ids)
{
    bool ret = false;

    if(coll_pair_ids)
    {
        coll_pair_ids->clear();
    }

    int id = 0;

    for(auto& cpd : _collision_pair_data)
    {
        cpd.compute_collision(*_model);

        if(cpd.cresult.isCollision())
        {
            if(coll_pair_ids)
            {
                coll_pair_ids->push_back(id);
                ret = true;
            }
            else
            {
                return true;
            }
        }

        ++id;
    }

    return ret;
}

void CollisionModel::Impl::check_distance_called_throw(const char * func)
{
    if(!(_cached_computation & Distance))
    {
        throw std::runtime_error(
            fmt::format("{} requires calling getDistance() first",
                        func));
    }
}

void CollisionModel::Impl::set_distance_called()
{
    _cached_computation |= Distance;
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
        ret.emplace_back(item.link1->link_name, item.link2->link_name);
    }

    return ret;

}

std::set<std::pair<std::string, std::string>> CollisionModel::getLinkPairs() const
{
    return impl->_active_link_pairs;
}

void CollisionModel::setLinkPairs(std::set<std::pair<std::string, std::string>> pairs)
{
    impl->_active_link_pairs = pairs;

    impl->updateCollisionPairData();
}

bool CollisionModel::checkSelfCollision(std::vector<int>& coll_pair_ids)
{
    return impl->checkSelfCollision(&coll_pair_ids);
}

bool CollisionModel::checkSelfCollision()
{
    return impl->checkSelfCollision(nullptr);
}

void CollisionModel::update()
{
    for(auto& lc : impl->_link_collision_map)
    {
        lc.second->update(*impl->_model);
    }

    impl->_cached_computation = 0;
}

std::vector<Eigen::Vector3d> CollisionModel::getNormals() const
{
    std::vector<Eigen::Vector3d> n;

    getNormals(n);

    return n;
}

void CollisionModel::getNormals(std::vector<Eigen::Vector3d> &n) const
{
    impl->check_distance_called_throw(__func__);

    n.resize(getNumCollisionPairs());

    for(int i = 0; i < n.size(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        n[i] = cpd.dresult.normal;
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
    impl->check_distance_called_throw(__func__);

    wp.resize(getNumCollisionPairs());

    for(int i = 0; i < wp.size(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        wp[i].first = cpd.dresult.nearest_points[0];
        wp[i].second = cpd.dresult.nearest_points[1];
    }
}

Eigen::VectorXd CollisionModel::computeDistance(double threshold) const
{
    Eigen::VectorXd ret;

    computeDistance(ret, threshold);

    return ret;
}

void CollisionModel::computeDistance(Eigen::VectorXd& d, double threshold) const
{
    for(auto& item : impl->_collision_pair_data)
    {
        item.compute_distance(*impl->_model, threshold);
    }

    impl->set_distance_called();

    d.resize(getNumCollisionPairs());

    for(int i = 0; i < d.size(); i++)
    {
        d[i] = impl->_collision_pair_data[i].dresult.min_distance;
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

    impl->check_distance_called_throw(__func__);

    auto& model = *impl->_model;

    auto& Jtmp = impl->_Jtmp;

    Jtmp.resize(6, model.getNv());

    for(int i = 0; i < J.rows(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        // translate J1 to witness point
        Eigen::Vector3d r = cpd.dresult.nearest_points[0] - cpd.link1->w_T_l.translation();
        Jtmp = cpd.link1->J;
        XBot::Utils::changeRefPoint(Jtmp, r);

        // update distance jacobian
        J.row(i).noalias() = -cpd.dresult.normal.transpose() * Jtmp.topRows<3>();

        // translate J2 to witness point
        r = cpd.dresult.nearest_points[1] - cpd.link2->w_T_l.translation();
        Jtmp = cpd.link2->J;
        XBot::Utils::changeRefPoint(Jtmp, r);

        // update distance jacobian
        J.row(i).noalias() += cpd.dresult.normal.transpose() * Jtmp.topRows<3>();

    }

}

CollisionModel::~CollisionModel()
{

}

CollisionModel::Impl::CollisionPairData::CollisionPairData(
    std::shared_ptr<fcl::CollisionObject> _o1, std::shared_ptr<fcl::CollisionObject> _o2):
    dist(_o1->collisionGeometryPtr(), _o2->collisionGeometryPtr()),
    coll(_o1->collisionGeometryPtr(), _o2->collisionGeometryPtr()),
    o1(_o1), o2(_o2)
{

}

void CollisionModel::Impl::CollisionPairData::compute_distance(const ModelInterface &model,
                                                               double threshold)
{
    Eigen::Vector3d aabb_1 = o1->getAABB().center();
    Eigen::Vector3d aabb_2 = o2->getAABB().center();
    Eigen::Vector3d aabb_21 = aabb_2 - aabb_1;
    double aabb_dist = o1->getAABB().distance(o2->getAABB());

    dresult.clear();

    if(threshold > 0 && aabb_dist > threshold)
    {
        dresult.min_distance = aabb_dist;
        dresult.normal = aabb_21.normalized();
        dresult.nearest_points[0] = aabb_1 + dresult.normal;
        dresult.nearest_points[1] = aabb_2 - dresult.normal;
        return;
    }

    /* Crash to be investigated:
     * D435_head_camera_link vs arm1_4
     * 0.2893013111227844  0.6620163446598711 -0.8569489983182211   0.3255094250551769 -0.04220620018335156  -0.7014379749636802   0.6326507868841883
     * 0.3030546655518612  0.6643892127068073 -0.8234177007679618  0.05331201346441317    0.988241826570244  -0.1400101570343576 -0.03054631507551606
     * test_collision: /home/iit.local/alaurenzi/code/core_ws/src/hpp-fcl/src/narrowphase/gjk.cpp:339: void hpp::fcl::details::getSupportFuncTpl(const MinkowskiDiff&, const hpp::fcl::Vec3f&, bool, hpp::fcl::Vec3f&, hpp::fcl::Vec3f&, hpp::fcl::support_func_guess_t&, MinkowskiDiff::ShapeData*) [with Shape0 = hpp::fcl::Box; Shape1 = hpp::fcl::Capsule; bool TransformIsIdentity = false; hpp::fcl::Vec3f = Eigen::Matrix<double, 3, 1>; hpp::fcl::support_func_guess_t = Eigen::Matrix<int, 2, 1>]: Assertion `NeedNormalizedDir || dir.cwiseAbs().maxCoeff() >= 1e-6' failed.
     * Aborted (core dumped)
     */


    // std::cout << link1->link_name << " vs " << link2->link_name << "\n" <<
    //     o1->getTransform().getTranslation().transpose().format(16) << " " << o1->getTransform().getQuatRotation().coeffs().transpose().format(16) << "\n" <<
    //     o2->getTransform().getTranslation().transpose().format(16) << " " << o2->getTransform().getQuatRotation().coeffs().transpose().format(16) << "\n";

    dist(o1->getTransform(),
         o2->getTransform(),
         drequest,
         dresult);


    // hack to fix wrong distance when in deep collision
    if(dresult.min_distance < 0)
    {
        dresult.min_distance = -(dresult.nearest_points[0] - dresult.nearest_points[1]).norm();
    }
}

void CollisionModel::Impl::CollisionPairData::compute_collision(const ModelInterface &model,
                                                                double threshold)
{
    Eigen::Vector3d aabb_1 = o1->getAABB().center();
    Eigen::Vector3d aabb_2 = o2->getAABB().center();
    Eigen::Vector3d aabb_21 = aabb_2 - aabb_1;
    double aabb_dist = o1->getAABB().distance(o2->getAABB());

    cresult.clear();

    if(threshold > 0 && aabb_dist > threshold)
    {
        return;
    }

    crequest.num_max_contacts = 1;

    coll(o1->getTransform(),
         o2->getTransform(),
         crequest,
         cresult);
}

CollisionModel::Impl::LinkCollision::LinkCollision(const ModelInterface &model,
                                                   string_const_ref link_name,
                                                   std::vector<CollisionGeometryPtr> geoms,
                                                   std::vector<Eigen::Affine3d> l_T_shape)
{
    this->link_name = link_name;

    link_id = model.getLinkId(link_name);

    if(link_id < 0)
    {
        throw std::runtime_error("link '" + link_name + "' undefined");
    }

    if(geoms.size() != l_T_shape.size())
    {
        auto what = fmt::format("internal error: geoms.size() != l_T_shape.size() [{} != {}] "
                                "while processing link '{}'",
                                geoms.size(), l_T_shape.size(), link_name);

        throw std::runtime_error(what);
    }

    this->l_T_shape = l_T_shape;

    for(int i = 0; i < geoms.size(); i++)
    {
        auto obj = std::make_shared<fcl::CollisionObject>(geoms[i]);
        coll_obj.push_back(obj);
    }

    J = model.getJacobian(link_name);
}

void CollisionModel::Impl::LinkCollision::update(const ModelInterface &model)
{
    w_T_l = model.getPose(link_id);

    model.getJacobian(link_id, J);

    for(int i = 0; i < coll_obj.size(); i++)
    {
        auto w_T_shape = w_T_l * l_T_shape[i];

        coll_obj[i]->setTransform(tofcl(w_T_shape));

        coll_obj[i]->computeAABB();
    }

}
