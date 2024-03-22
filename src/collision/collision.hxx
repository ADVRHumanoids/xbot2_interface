#ifndef COLLISION_HXX
#define COLLISION_HXX

#include <xbot2_interface/collision.h>

#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/broadphase/broadphase.h>


namespace fcl = hpp::fcl;

namespace XBot {

class Collision::CollisionModel::Impl
{

public:

    friend class CollisionModel;

    Impl(ModelInterface::ConstPtr model);

    bool parseCollisionObjects();

    void generateAllPairs();

    void removeDisabledPairs();

    void updateCollisionPairData();

    void makeCollisionManager();

    //

    bool checkSelfCollision(std::vector<int> * coll_pair_ids, bool include_env, double threshold);

    bool addCollisionShape(string_const_ref name,
                           string_const_ref link,
                           Shape::Variant shape,
                           Eigen::Affine3d link_T_shape,
                           std::vector<std::string> disabled_collisions);

    void check_distance_called_throw(const char * func);

    void set_distance_called();

private:

    Eigen::MatrixXd _Jtmp;

    ModelInterface::ConstPtr _model;

    typedef std::pair<std::string, std::string> LinkPair;
    typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;
    typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr;

    struct LinkCollision
    {
        typedef std::shared_ptr<LinkCollision> Ptr;

        LinkCollision(const ModelInterface& model,
                      string_const_ref link_name,
                      std::vector<CollisionGeometryPtr> geoms = std::vector<CollisionGeometryPtr>{},
                      std::vector<Eigen::Affine3d> l_T_shape = std::vector<Eigen::Affine3d>{});

        void update(const ModelInterface& model);

        void updatePose(CollisionObjectPtr co, const Eigen::Affine3d& link_T_shape);

        Eigen::Affine3d getPose(CollisionObjectPtr co) const;

        int getIndex(CollisionObjectPtr co) const;

        void addCollisionObject(CollisionObjectPtr co,  const Eigen::Affine3d& link_T_shape);

        void setEnabled(CollisionObjectPtr co, bool flag);

        bool is_world;
        int link_id;
        std::string link_name;
        Eigen::Affine3d w_T_l;
        Eigen::MatrixXd J;

        std::vector<Eigen::Affine3d> l_T_shape;
        std::vector<CollisionObjectPtr> coll_obj;
        std::vector<bool> enabled;
        std::vector<std::set<std::string>> disabled_collisions;
    };

    struct CollisionPairData
    {
        fcl::ComputeDistance dist;
        fcl::DistanceRequest drequest;
        fcl::DistanceResult dresult;

        fcl::ComputeCollision coll;
        fcl::CollisionRequest crequest;
        fcl::CollisionResult cresult;

        CollisionObjectPtr o1, o2;
        LinkCollision::Ptr link1, link2;
        int co_idx1, co_idx2;
        int id1, id2;

        CollisionPairData(CollisionObjectPtr o1,
                          CollisionObjectPtr o2);

        void compute_distance(const ModelInterface &model, double threshold = -1);

        void compute_collision(const ModelInterface &model, double threshold = -1);
    };

    enum ComputationType
    {
        None = 0,
        Distance = 1,
        DistanceJac = 2
    };

    mutable uint16_t _cached_computation = 0;

    // distances are computed for each item in this vector
    std::vector<CollisionPairData> _collision_pair_data;
    int _n_self_collision_pairs;

    // map link -> collisions
    std::map<std::string, LinkCollision::Ptr> _link_collision_map;

    // initialized from ACM, can be set by user, it is used for
    // filling _collision_pair_data
    std::set<LinkPair> _active_link_pairs;

    // internal storage for the vector of ordered collision pair
    // indices (ascending distance)
    std::vector<int> _ordered_idx;

    // user object
    struct UserObject {

        LinkCollision::Ptr link_collision;

        CollisionObjectPtr collision_object;

        Shape::Variant shape;
    };

    // environment
    LinkCollision::Ptr _env_collision;
    std::set<std::string> _env_active_links;
    std::map<std::string, UserObject> _user_object_map;

};

}


#endif // COLLISION_HXX
