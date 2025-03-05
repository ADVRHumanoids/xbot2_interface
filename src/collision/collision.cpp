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

CollisionModel::Impl::Impl(ModelInterface::ConstPtr model,
                           Collision::CollisionModel& api):
    _api(api),
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

    // construct env collision model
    // TODO put collision elements of link 'world' there?
    if(_model->getLinkId("world") >= 0)
    {
        _env_collision = std::make_shared<LinkCollision>(*_model, "world");
    }

    return true;
}

void CollisionModel::Impl::generateAllPairs()
{
    // get urdf links
    std::vector<urdf::LinkSharedPtr> links;
    _model->getUrdf()->getLinks(links);

    _active_link_pairs.clear();
    _env_active_links.clear();

    for(int i = 0; i < links.size(); i++)
    {
        // no collision objects for this link
        if(!_link_collision_map.contains(links[i]->name))
        {
            continue;
        }

        // add to env links
        _env_active_links.insert(links[i]->name);

        for(int j = i+1; j < links.size(); j++)
        {
            // no collision objects for this link
            if(!_link_collision_map.contains(links[j]->name))
            {
                continue;
            }

            // add pair
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

    _ordered_idx.clear();

    _collision_pairs_no_env.clear();

    _collision_pairs.clear();

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
                if(c1->disabled_collisions[i].contains(l2) ||
                    c2->disabled_collisions[j].contains(l1))
                {
                    continue;
                }

                CollisionPairData cpd(c1->coll_obj[i], c2->coll_obj[j]);

                cpd.link1 = c1;
                cpd.link2 = c2;
                cpd.co_idx1 = c1->getIndex(c1->coll_obj[i]);
                cpd.co_idx2 = c2->getIndex(c2->coll_obj[j]);
                cpd.id1 = _model->getLinkId(l1);
                cpd.id2 = _model->getLinkId(l2);

                cpd.drequest.enable_nearest_points = true;
                // cpd.crequest.....

                _collision_pair_data.emplace_back(std::move(cpd));

                // fmt::print("added pair i = {}: {} vs {} \n",
                //            pidx, l1, l2);

                _ordered_idx.push_back(pidx);

                _collision_pairs.emplace_back(l1, l2);

                _collision_pairs_no_env.emplace_back(l1, l2);

                pidx++;

            }
        }
    }

    _n_self_collision_pairs = _collision_pair_data.size();

    if(!_env_collision || _env_collision->coll_obj.size() == 0)
    {
        return;
    }

    // add robot-env collisions

    for(auto l : _env_active_links)
    {
        // check link exists
        if(_model->getLinkId(l) < 0)
        {
            throw std::out_of_range(
                fmt::format("link '{}' does not exist within model", l));
        }

        auto c = _link_collision_map.at(l);
        auto env = _env_collision;

        

        for(int i = 0; i < c->coll_obj.size(); i++)
        {
            for(int j = 0; j < env->coll_obj.size(); j++)
            {
                if(env->disabled_collisions[j].contains(l) || c->disabled_collisions[i].contains("world"))
                {
                    continue;
                }

                CollisionPairData cpd(c->coll_obj[i], env->coll_obj[j]);

                cpd.link1 = c;
                cpd.link2 = env;
                cpd.co_idx1 = c->getIndex(c->coll_obj[i]);
                cpd.co_idx2 = env->getIndex(env->coll_obj[j]);
                cpd.id1 = _model->getLinkId(l);
                cpd.id2 = -1;

                cpd.drequest.enable_nearest_points = true;
                // cpd.crequest.....

                _collision_pair_data.emplace_back(std::move(cpd));

                // fmt::print("added pair i = {}: {} vs {} \n",
                //            pidx, l, "env");

                _ordered_idx.push_back(pidx);

                _collision_pairs.emplace_back(l, "world");

                pidx++;

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

bool CollisionModel::Impl::checkSelfCollision(std::vector<int>* coll_pair_ids,
                                              bool include_env,
                                              double threshold)
{
    bool ret = false;

    if(coll_pair_ids)
    {
        coll_pair_ids->clear();
    }

    int id = 0;

    for(auto& cpd : _collision_pair_data)
    {
        if(!include_env && cpd.link2->is_world)
        {
            return ret;
        }

        cpd.compute_collision(*_model, threshold);

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

// define helper struct to generate a visitor from a set of lambdas
template<typename ... Ts>
struct Overload : Ts ... {
    using Ts::operator() ...;
};
template<class... Ts> Overload(Ts...) -> Overload<Ts...>;

bool CollisionModel::Impl::addCollisionShape(string_const_ref name,
                                             string_const_ref link,
                                             Shape::Variant shape,
                                             Eigen::Affine3d link_T_shape,
                                             std::vector<std::string> disabled_collisions)
{
    // check already exists
    // if so, check if type matches and move it
    if(_user_object_map.contains(name))
    {
        auto uo = _user_object_map.at(name);

        if(uo.shape.index() != shape.index())
        {
            fmt::print("type mismatch for already existing shape '{}'", name);

            return false;
        }

        uo.link_collision->updatePose(uo.collision_object, link_T_shape);

        return true;
    }

    std::shared_ptr<hpp::fcl::CollisionGeometry> fcl_geom;

    std::cout << "adding shape with name " << name << ", type ";

    auto ShapeVisitor = Overload {
        [&](const Shape::Box& box)
        {
            fcl_geom = std::make_shared<hpp::fcl::Box>(
                box.size.x(),
                box.size.y(),
                box.size.z()
                );

            std::cout << "box";

            return true;
        },
        [&](const Shape::Capsule& caps)
        {
            fcl_geom = std::make_shared<hpp::fcl::Capsule>(
                caps.radius,
                caps.length
                );

            std::cout << "capsule";

            return true;
        },
        [&](const Shape::Cylinder& cyl)
        {
            fcl_geom = std::make_shared<hpp::fcl::Cylinder>(
                cyl.radius,
                cyl.length
                );

            std::cout << "cylinder";

            return true;
        },
        [&](const Shape::Halfspace& hs)
        {
            fcl_geom = std::make_shared<hpp::fcl::Halfspace>(
                hs.normal,
                hs.d
                );

            std::cout << "halfspace";

            return true;
        },
        [&](const Shape::HeightMap& caps)
        {
            throw std::runtime_error("heightmap not implemented");

            return false;
        },
        [&](const Shape::Mesh& m)
        {
            // read mesh file
            auto mesh = shapes::createMeshFromResource(m.filepath);

            if(!mesh)
            {
                std::cout << "Error loading mesh for collision " << name << std::endl;
                return false;
            }

            // fill vertices and triangles
            std::vector<fcl::Vec3f> vertices;
            std::vector<fcl::Triangle> triangles;

            for(unsigned int i = 0; i < mesh->vertex_count; ++i)
            {
                fcl::Vec3f v(mesh->vertices[3*i]*m.scale.x(),
                             mesh->vertices[3*i + 1]*m.scale.y(),
                             mesh->vertices[3*i + 2]*m.scale.z());

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
            fcl_geom = bvhModel;
            bvhModel->beginModel();
            bvhModel->addSubModel(vertices, triangles);
            bvhModel->endModel();

            std::cout << "mesh";

            return true;
        },
        [&](const Shape::Octree& oct)
        {
            // fcl_geom = std::any_cast<std::shared_ptr<hpp::fcl::OcTree>>(oct.data);
            return false;
        },
        [&](const Shape::Sphere& sp)
        {
            fcl_geom = std::make_shared<hpp::fcl::Sphere>(sp.radius);
            std::cout << "sphere";
            return true;
        }
    };

    if(!std::visit(ShapeVisitor, shape))
    {
        std::cout << "..failed \n";
        return false;
    }

    if(!fcl_geom)
    {
        std::cout << "..failed \n";
        throw std::runtime_error("fcl geometry is null");
    }

    std::cout << "..ok \n";

    fcl_geom->computeLocalAABB();

    // search the correct link collision object
    LinkCollision::Ptr link_collision;

    if(link == "world" && !_env_collision)
    {
        throw std::invalid_argument("no world link was defined");
    }

    if(link == "world")
    {
        link_collision = _env_collision;
    }
    else if(!_link_collision_map.contains(link))
    {
        _link_collision_map[link]
            = link_collision =
            std::make_shared<LinkCollision>(*_model,
                                            link);
    }
    else
    {
        link_collision = _link_collision_map.at(link);
    }

    // add geometry to collision
    auto fcl_obj = std::make_shared<fcl::CollisionObject>(fcl_geom);
    link_collision->addCollisionObject(fcl_obj, link_T_shape);

    // check disabled collisions are valid link names
    for(auto& dc : disabled_collisions)
    {
        if(!_link_collision_map.contains(dc))
        {
            throw std::invalid_argument(
                fmt::format("disabled collision '{}' is not a valid collision link name",
                            dc));
        }
    }

    // add disabled collisions
    link_collision->disabled_collisions.back().insert(disabled_collisions.begin(),
                                                      disabled_collisions.end());

    // add to user map
    _user_object_map[name] = {link_collision, fcl_obj, shape};

    // re-generate pairs
    updateCollisionPairData();


    return true;
}

XBot::Collision::CollisionModel::ComputeCollisionFreeOptions::ComputeCollisionFreeOptions()
{
    max_iter = std::numeric_limits<int>::max();
    include_env = true;
    min_distance = 0.0;
}

bool CollisionModel::Impl::computeCollisionFree(VecRef q,
                                                ComputeCollisionFreeOptions opt)
{

    if(opt.w_norm.size() > 0 &&
        opt.w_norm.size() != _model->getNv())
    {
        throw std::invalid_argument(
            fmt::format("w_norm has wrong size {} != {}",
                        opt.w_norm.size(), _model->getNv()));
    }

    if(opt.w_norm.minCoeff() <= 0)
    {
        throw std::invalid_argument(
            fmt::format("w_norm has negative coefficients"));
    }

    double d_th = opt.min_distance + 0.10;

    double min_d = opt.min_distance;

    double err_th = 1e-2;

    double lam = 1.0;

    bool include_env = opt.include_env;

    const auto& coll_pairs = _api.getCollisionPairs(include_env);

    Eigen::MatrixXd Jn(coll_pairs.size(), _model->getNv());
    Eigen::VectorXd en(coll_pairs.size());  // des - actual

    auto model = std::const_pointer_cast<ModelInterface>(_model);

    model->setJointPosition(q);

    for(int k = 0; k < opt.max_iter; k++)
    {
        model->update();

        _api.update();

        Eigen::VectorXd d = _api.computeDistance(include_env, d_th);

        if(d.minCoeff() > min_d - err_th)
        {
            return true;
        }

        Eigen::MatrixXd J = _api.getDistanceJacobian(include_env);

        int nn = 0;

        for(int i = 0; i < d.size(); i++)
        {
            if(d[i] >= min_d)
            {
                continue;
            }

            en[nn] = min_d - d[i];
            Jn.row(nn) = J.row(i);
            nn++;
        }

        auto Jn_block = Jn.topRows(nn);
        auto en_block = en.head(nn);

        auto svd = Jn_block.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
        svd.setThreshold(svd.singularValues()[0] / 20.0);

        Eigen::VectorXd dq = lam * svd.solve(en_block);

        model->integrateJointPosition(dq);
    }

    return false;
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
    impl(std::make_unique<Impl>(model, *this))
{

}

int CollisionModel::getNumCollisionPairs(bool include_env) const
{
    if(include_env)
    {
        return impl->_collision_pair_data.size();
    }
    else
    {
        return impl->_n_self_collision_pairs;
    }
}

bool CollisionModel::addCollisionShape(string_const_ref name,
                                       string_const_ref link,
                                       Shape::Variant shape,
                                       Eigen::Affine3d link_T_shape,
                                       std::vector<std::string> disabled_collisions)
{
    if(impl->addCollisionShape(name, link, shape, link_T_shape, disabled_collisions))
    {
        update();
        return true;
    }

    return false;
}

bool CollisionModel::setCollisionShapeActive(string_const_ref name, bool flag)
{
    auto user_obj_it = impl->_user_object_map.find(name);

    if(user_obj_it == impl->_user_object_map.end())
    {
        fmt::print("shape {} not found", name);
        return false;
    }

    auto co = user_obj_it->second.collision_object;
    auto lc = user_obj_it->second.link_collision;

    lc->setEnabled(co, flag);

    return true;
}

CollisionModel::CollisionShapeData CollisionModel::getCollisionShapeData(string_const_ref name) const
{
    auto user_obj_it = impl->_user_object_map.find(name);

    if(user_obj_it == impl->_user_object_map.end())
    {
        throw std::out_of_range(name + " not found");
    }

    auto co = user_obj_it->second.collision_object;
    auto lc = user_obj_it->second.link_collision;
    auto sh = user_obj_it->second.shape;

    return {lc->link_name, sh, lc->getPose(co)};
}

bool CollisionModel::moveCollisionShape(string_const_ref name, Eigen::Affine3d link_T_shape)
{
    auto user_obj_it = impl->_user_object_map.find(name);

    if(user_obj_it == impl->_user_object_map.end())
    {
        fmt::print("user collision shape '{}' not found", name);
        return false;
    }

    auto co = user_obj_it->second.collision_object;

    auto lc = user_obj_it->second.link_collision;

    try
    {
        lc->updatePose(co, link_T_shape);
    }
    catch(std::out_of_range& e)
    {
        return false;
    }

    return true;
}

const std::vector<std::pair<std::string, std::string>>&
    CollisionModel::getCollisionPairs(bool include_env) const
{
    if(include_env)
    {
        return impl->_collision_pairs;
    }
    else
    {
        return impl->_collision_pairs_no_env;
    }
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

std::set<std::string> CollisionModel::getLinksVsEnvironment() const
{
    return impl->_env_active_links;
}

void CollisionModel::setLinksVsEnvironment(std::set<std::string> links)
{
    impl->_env_active_links = links;

    impl->updateCollisionPairData();
}

void CollisionModel::resetLinkPairs()
{
    // note: restore old env active links
    auto env_pairs = impl->_env_active_links;
    impl->generateAllPairs();
    impl->_env_active_links = env_pairs;

    impl->removeDisabledPairs();
    impl->updateCollisionPairData();
}

void CollisionModel::resetLinksVsEnvironment()
{
    impl->generateAllPairs();
    impl->removeDisabledPairs();
    impl->updateCollisionPairData();
}

bool CollisionModel::checkSelfCollision(std::vector<int>& coll_pair_ids, double threshold)
{
    return impl->checkSelfCollision(&coll_pair_ids, false, threshold);
}

bool CollisionModel::checkSelfCollision(double threshold)
{
    return impl->checkSelfCollision(nullptr, false, threshold);
}

bool CollisionModel::checkCollision(std::vector<int> &coll_pair_ids, bool include_env, double threshold)
{
    return impl->checkSelfCollision(&coll_pair_ids, include_env, threshold);
}

bool CollisionModel::checkCollision(bool include_env, double threshold)
{
    return impl->checkSelfCollision(nullptr, include_env, threshold);
}

void CollisionModel::update()
{
    for(auto& lc : impl->_link_collision_map)
    {
        lc.second->update(*impl->_model);
    }

    impl->_cached_computation = 0;
}

std::vector<Eigen::Vector3d> CollisionModel::getNormals(bool include_env) const
{
    std::vector<Eigen::Vector3d> n;

    getNormals(n, include_env);

    return n;
}

void CollisionModel::getNormals(std::vector<Eigen::Vector3d> &n, bool include_env) const
{
    impl->check_distance_called_throw(__func__);

    n.resize(getNumCollisionPairs(include_env));

    for(int i = 0; i < n.size(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        n[i] = cpd.dresult.normal;
    }
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> CollisionModel::getWitnessPoints(bool include_env) const
{
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ret;

    getWitnessPoints(ret, include_env);

    return ret;

}

void CollisionModel::getWitnessPoints(
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &wp,
    bool include_env) const
{
    impl->check_distance_called_throw(__func__);

    wp.resize(getNumCollisionPairs(include_env));

    for(int i = 0; i < wp.size(); i++)
    {
        const auto& cpd = impl->_collision_pair_data[i];

        wp[i].first = cpd.dresult.nearest_points[0];
        wp[i].second = cpd.dresult.nearest_points[1];
    }
}

Eigen::VectorXd CollisionModel::computeDistance(bool include_env, double threshold) const
{
    Eigen::VectorXd ret;

    computeDistance(ret, include_env, threshold);

    return ret;
}

void CollisionModel::computeDistance(Eigen::VectorXd& d,
                                     bool include_env,
                                     double threshold) const
{
    for(auto& item : impl->_collision_pair_data)
    {
        // if id2 == -1 we have reached a robot-env collision pair,
        // break the loop if include_env is false
        if(item.id2 < 0 && !include_env)
        {
            break;
        }

        item.compute_distance(*impl->_model, threshold);
    }

    impl->set_distance_called();

    d.resize(getNumCollisionPairs(include_env));

    for(int i = 0; i < d.size(); i++)
    {
        d[i] = impl->_collision_pair_data[i].dresult.min_distance;
    }
}

Eigen::MatrixXd CollisionModel::getDistanceJacobian(bool include_env) const
{
    Eigen::MatrixXd J(getNumCollisionPairs(include_env), impl->_model->getNv());

    getDistanceJacobian(J, include_env);

    return J;
}

void CollisionModel::getDistanceJacobian(MatRef J, bool include_env) const
{
    check_mat_size(J, getNumCollisionPairs(include_env), impl->_model->getNv(), __func__);

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

        // if we're processing a robot-env collision pair,
        // we don't need to add the contribution from link2 (i.e., env),
        // as the world jacobian is zero
        if(i >= impl->_n_self_collision_pairs)
        {
            continue;
        }

        // translate J2 to witness point
        r = cpd.dresult.nearest_points[1] - cpd.link2->w_T_l.translation();
        Jtmp = cpd.link2->J;
        XBot::Utils::changeRefPoint(Jtmp, r);

        // update distance jacobian
        J.row(i).noalias() += cpd.dresult.normal.transpose() * Jtmp.topRows<3>();

    }
}

const std::vector<int> &CollisionModel::getOrderedCollisionPairIndices() const
{
    std::sort(impl->_ordered_idx.begin(), impl->_ordered_idx.end(), [this](int a, int b) {
        return impl->_collision_pair_data[a].dresult.min_distance <
               impl->_collision_pair_data[b].dresult.min_distance;
    });

    return impl->_ordered_idx;
}

bool CollisionModel::computeCollisionFree(VecRef q, ComputeCollisionFreeOptions opt)
{
    return impl->computeCollisionFree(q, opt);
}

CollisionModel::~CollisionModel()
{

}

CollisionModel::Impl::CollisionPairData::CollisionPairData(
    std::shared_ptr<fcl::CollisionObject> _o1, std::shared_ptr<fcl::CollisionObject> _o2):
    dist(_o1->collisionGeometry().get(), _o2->collisionGeometry().get()),
    coll(_o1->collisionGeometry().get(), _o2->collisionGeometry().get()),
    o1(_o1), o2(_o2)
{

}

void CollisionModel::Impl::CollisionPairData::compute_distance(const ModelInterface &model,
                                                               double threshold)
{
    dresult.clear();

    // one of the two collisions is disabled, return
    if(!link1->enabled[co_idx1] || !link2->enabled[co_idx2])
    {
        dresult.min_distance = std::numeric_limits<double>::infinity();
        dresult.normal.setZero();  // note: this makes jacobian be zero as we want
        dresult.nearest_points[0].setZero();
        dresult.nearest_points[1].setZero();
        return;
    }

    // inexpensive aabb test
    Eigen::Vector3d aabb_1 = o1->getAABB().center();
    Eigen::Vector3d aabb_2 = o2->getAABB().center();
    Eigen::Vector3d aabb_21 = aabb_2 - aabb_1;
    double aabb_dist = o1->getAABB().distance(o2->getAABB(),
                                              &dresult.nearest_points[0],
                                              &dresult.nearest_points[1]);

    if(threshold > 0 && aabb_dist > threshold)
    {
        dresult.min_distance = aabb_dist;
        dresult.normal = (dresult.nearest_points[1]-dresult.nearest_points[0]).normalized();
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

    // distance computation
    dresult.normal.setZero();

    dist(o1->getTransform(),
         o2->getTransform(),
         drequest,
         dresult);

    // hack to fix wrong distance when in deep collision
    if(dresult.min_distance < 0)
    {
        dresult.min_distance = -(dresult.nearest_points[0] - dresult.nearest_points[1]).norm();
    }

    // hack fix normal bug gjk
    if(dresult.normal.norm() < 1e-6)
    {
        dresult.normal = (dresult.nearest_points[1]-dresult.nearest_points[0]).normalized();
    }
}

void CollisionModel::Impl::CollisionPairData::compute_collision(const ModelInterface &model,
                                                                double threshold)
{
    cresult.clear();

    // one of the two collisions is disabled, return
    if(!link1->enabled[co_idx1] || !link2->enabled[co_idx2])
    {
        return;
    }

    // simple aabb distance
    Eigen::Vector3d aabb_1 = o1->getAABB().center();
    Eigen::Vector3d aabb_2 = o2->getAABB().center();
    Eigen::Vector3d aabb_21 = aabb_2 - aabb_1;
    double aabb_dist = o1->getAABB().distance(o2->getAABB());

    // aabb distance larger than given threshold -> cannot collide !
    if(threshold > 0 && aabb_dist > threshold)
    {
        return;
    }

    crequest.num_max_contacts = 1;
    crequest.security_margin = threshold;
    crequest.enable_contact = false;

    coll(o1->getTransform(),
         o2->getTransform(),
         crequest,
         cresult);
}


CollisionModel::Impl::LinkCollision::LinkCollision(const ModelInterface &model,
                                                   string_const_ref link_name,
                                                   std::vector<CollisionGeometryPtr> geoms,
                                                   std::vector<Eigen::Affine3d> l_T_shape):
    enabled(geoms.size(), true),
    disabled_collisions(geoms.size())
{
    this->link_name = link_name;


    if(link_name == "world") // world
    {
        is_world = true;

        link_id = -1;

        w_T_l.setIdentity();

        J.setZero(6, model.getNv());
    }
    else // robot link
    {
        is_world = false;

        link_id = model.getLinkId(link_name);

        if(link_id < 0)
        {
            throw std::runtime_error("link '" + link_name + "' undefined");
        }
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

        // note: initialize transform assuming w_T_l = eye
        // this is ok for environment
        obj->setTransform(tofcl(l_T_shape[i]));
        obj->computeAABB();
    }

    J = model.getJacobian(link_name);
}

void CollisionModel::Impl::LinkCollision::update(const ModelInterface &model)
{
    if(is_world)
    {
        // nothing to do as we already set all object transforms
        // in the constructor

        return;
    }

    w_T_l = model.getPose(link_id);

    model.getJacobian(link_id, J);

    for(int i = 0; i < coll_obj.size(); i++)
    {
        auto w_T_shape = w_T_l * l_T_shape[i];

        coll_obj[i]->setTransform(tofcl(w_T_shape));

        coll_obj[i]->computeAABB();
    }
}

void CollisionModel::Impl::LinkCollision::updatePose(CollisionObjectPtr co,
                                                     const Eigen::Affine3d &pose)
{
    int idx = getIndex(co);

    if(idx < 0)
    {
        throw std::out_of_range(link_name + ": could not find collision object");
    }

    l_T_shape[idx] = pose;

    if(is_world)
    {
        coll_obj[idx]->setTransform(tofcl(pose));
        coll_obj[idx]->computeAABB();
    }
}

Eigen::Affine3d CollisionModel::Impl::LinkCollision::getPose(CollisionObjectPtr co) const
{
    int idx = getIndex(co);

    if(idx < 0)
    {
        throw std::out_of_range(link_name + ": could not find collision object");
    }

    return l_T_shape[idx];
}

int CollisionModel::Impl::LinkCollision::getIndex(CollisionObjectPtr co) const
{
    auto co_it = std::find(coll_obj.begin(), coll_obj.end(), co);

    if(co_it == coll_obj.end())
    {
        return -1;
    }

    return co_it - coll_obj.begin();
}

void CollisionModel::Impl::LinkCollision::addCollisionObject(CollisionObjectPtr co,
                                                             const Eigen::Affine3d &link_T_shape)
{
    if(getIndex(co) >= 0)
    {
        throw std::invalid_argument("collision object cannot be added: exists");
    }

    coll_obj.push_back(co);
    l_T_shape.push_back(link_T_shape);
    enabled.push_back(true);
    disabled_collisions.push_back({});

    updatePose(co, link_T_shape);
}

void CollisionModel::Impl::LinkCollision::setEnabled(CollisionObjectPtr co, bool flag)
{
    int idx = getIndex(co);

    if(idx < 0)
    {
        throw std::invalid_argument("[LinkCollision::setActive] collision object not found");
    }

    enabled[idx] = flag;

}
