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

private:

    Eigen::MatrixXd _Jtmp;

    ModelInterface::ConstPtr _model;

    typedef std::pair<std::string, std::string> LinkPair;

    struct CollisionPairData
    {
        fcl::ComputeDistance dist;
        fcl::DistanceRequest request;
        fcl::DistanceResult result;
        std::shared_ptr<fcl::CollisionGeometry> g1, g2;
        std::string link1, link2;
        int id1, id2;
        Eigen::Affine3d l_T_shape1, l_T_shape2;

        CollisionPairData(std::shared_ptr<fcl::CollisionGeometry> g1,
                          std::shared_ptr<fcl::CollisionGeometry>g2);

        void compute(const ModelInterface& model,
                     double threshold = -1);
    };

    struct LinkCollision
    {
        std::vector<std::shared_ptr<fcl::CollisionGeometry>> geom;
        std::vector<Eigen::Affine3d> l_T_shape;
    };

    std::vector<CollisionPairData> _collision_pair_data;

    std::map<std::string, LinkCollision> _link_collision_map;

    std::vector<LinkPair> _active_link_pairs;

};

}


#endif // COLLISION_HXX
