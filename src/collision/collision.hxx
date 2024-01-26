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

    bool checkSelfCollision(std::vector<int> * coll_pair_ids);

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

        int link_id;
        std::string link_name;
        Eigen::Affine3d w_T_l;
        Eigen::MatrixXd J;

        std::vector<Eigen::Affine3d> l_T_shape;
        std::vector<CollisionObjectPtr> coll_obj;
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

    std::vector<CollisionPairData> _collision_pair_data;

    std::map<std::string, LinkCollision::Ptr> _link_collision_map;

    std::set<LinkPair> _active_link_pairs;

};

}


#endif // COLLISION_HXX
