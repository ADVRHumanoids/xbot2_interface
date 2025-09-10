#ifndef XBOT2IFC_COLLISION_H
#define XBOT2IFC_COLLISION_H

#ifndef XBOT2IFC_COLLISION_SUPPORT
#error "collision support not included: did you link your library against the xbot2_interface::collision cmake target?"
#endif

#include <variant>
#include <set>

#include "xbotinterface2.h"

namespace XBot::Collision
{

inline namespace v2 {

struct XBOT2IFC_API Shape
{
    struct Sphere
    {
        double radius;
    };

    struct Capsule
    {
        double radius;
        double length;
    };

    struct Halfspace 
    {
        Eigen::Vector3d normal;
        double d;
    };

    struct Box
    {
        Eigen::Vector3d size;
    };

    struct Cylinder
    {
        double radius;
        double length;
    };

    struct Mesh
    {
        std::string filepath;
        Eigen::Vector3d scale;
        bool convex = false;
    };

    struct Octree
    {
        // data has to be a std::shared_ptr<hpp::fcl::OcTree>
        std::any data;
    };

    struct HeightMap
    {
        double dim_x;
        double dim_y;
        std::shared_ptr<Eigen::MatrixXf> height;
    };

    struct MeshRaw
    {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> triangles;
        bool convex = false;
    };

    using Variant = std::variant<
        Sphere,
        Capsule,
        Box,
        Cylinder,
        Halfspace,
        Mesh,
        Octree,
        HeightMap,
        MeshRaw
        >;
};

class XBOT2IFC_API CollisionModel
{

public:

    XBOT_DECLARE_SMART_PTR(CollisionModel);

    typedef std::vector<std::pair<std::string, std::string>> LinkPairVector;

    typedef std::set<std::pair<std::string, std::string>> LinkPairSet;

    typedef std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> WitnessPointVector;

    struct XBOT2IFC_API Options
    {
        // if true, the collision model will assume that all meshes are convex
        // this is *very* beneficial for performance
        bool assume_convex_meshes;

        Options();
    };

    /**
     * @brief CollisionModel constructor
     * @param model shared pointer
     */
    CollisionModel(ModelInterface::ConstPtr model, Options opt = Options());

    /**
     * @brief returns the number of collision pairs that the collision model will
     * compute the distance for
     * @param include_env: if true, also counts robot-environment collision pairs
     * @note this is not the same as getLinkPairs().size(), as one link could
     * have an arbitrary number of collision objects associated with it
     */
    int getNumCollisionPairs(bool include_env = false) const;

    /**
     * @brief addCollisionShape
     * @param name human-readable id for the collision shape
     * @param link to attach to
     * @param shape
     * @param link_T_shape: transform of the collision shape w.r.t. its parent link
     * @return true if success
     */
    bool addCollisionShape(string_const_ref name,
                           string_const_ref link,
                           Shape::Variant shape,
                           Eigen::Affine3d link_T_shape,
                           std::vector<std::string> disabled_collisions = {});

    /**
     * @brief disableCollisionShape
     * @param name
     * @return
     */
    bool setCollisionShapeActive(string_const_ref name, bool flag);

    /**
     * @brief Return type for the getCollisionShapeData method
     */
    struct CollisionShapeData
    {
        string_const_ref link;
        Shape::Variant shape;
        Eigen::Affine3d link_T_shape;
    };

    /**
     * @brief getCollisionShapeData
     * @param name
     * @return
     */
    CollisionShapeData getCollisionShapeData(string_const_ref name) const;

    /**
     * @brief moveCollisionShape
     * @param name
     * @param link_T_shape
     * @return
     */
    bool moveCollisionShape(string_const_ref name,
                            Eigen::Affine3d link_T_shape);

    /**
     * @brief returns the vector of link pairs corresponding to the model's collision
     * pairs (size = getNumCollisionPairs())
     * @param include_env: if true, also includes robot-environment collision pairs
     * @note link pairs could appear multiple times if they have more than one
     * collision object associated with them
     * @note the first getNumCollisionPairs(false) elements contain robot self collision
     * pairs, whereas the remaining ones are robot-environment pairs
     */
    const LinkPairVector& getCollisionPairs(bool include_env = false) const;

    /**
     * @brief returns the set of links that are being taken into account by this
     * collision model for self-collision and self-distance computations
     */
    LinkPairSet getLinkPairs() const;

    /**
     * @brief set the set of links that are being taken into accouny by this
     * collision model for self-collision and self-distance computations
     * @param pairs
     */
    void setLinkPairs(LinkPairSet pairs);

    /**
     * @brief returns the set of links that can collide with the environment
     */
    std::set<std::string> getLinksVsEnvironment() const;

    /**
     * @brief set the vector of robot links that can collide with the environment
     * @param links
     */
    void setLinksVsEnvironment(std::set<std::string> links);

    /**
     * @brief recompute self collision pairs from given urdf and srdf
     */
    void resetLinkPairs();

    /**
     * @brief recompute robot-environment collision pairs from given
     * urdf
     */
    void resetLinksVsEnvironment();

    /**
     * @brief checkSelfCollision
     * @return
     */
    bool checkSelfCollision(std::vector<int>& coll_pair_ids, double threshold = 0.0);

    /**
     * @brief checkSelfCollision
     * @return
     */
    bool checkSelfCollision(double threshold = 0.0);

    /**
     * @brief checkCollision
     * @return
     */
    bool checkCollision(std::vector<int>& coll_pair_ids,
                        bool include_env = true,
                        double threshold = 0.0);

    /**
     * @brief checkCollision
     * @return
     */
    bool checkCollision(bool include_env = true, double threshold = 0.0);

    /**
     * @brief update the collision model with the underlying ModelInterface's state
     */
    void update();

    /**
     * @brief performs distance computation for all active collision pairs; if the threshold
     * parameter is greater than zero, this function will only compute cheaper approximate
     * distance between collision pairs whose distance can be proved to be above the given
     * thresold, by means of an inexpensive AABB overlap test.
     * @param threshold: min distance below which exact distance computation is performed
     * @return vector of distances, one for each collision pair
     */
    Eigen::VectorXd computeDistance(bool include_env = false,
                                    double threshold = -1) const;

    /**
     * @brief performs distance computation for all active collision pairs; if the threshold
     * parameter is greater than zero, this function will only compute cheaper approximate
     * distance between collision pairs whose distance can be proved to be above the given
     * thresold, by means of an inexpensive AABB overlap test. Witness points and normals
     * for the simplified AABB overlap test are available after this call.
     * @param threshold: min distance below which exact distance computation is performed
     * @param d (output) vector of distances, one for each collision pair
     */
    void computeDistance(Eigen::VectorXd& d,
                         bool include_env = false,
                         double threshold = -1) const;

    /**
     * @brief returns the vector of all normals, one for each collision pair;
     * the i-th normal points from the witness point on object #1 to the witness
     * point on object #2
     */
    std::vector<Eigen::Vector3d> getNormals(bool include_env = false) const;

    void getNormals(std::vector<Eigen::Vector3d>& n, bool include_env = false) const;

    /**
     * @brief return the vector of witness points, one for each collision pair,
     * expressed in world coordinates
     */
    WitnessPointVector getWitnessPoints(bool include_env = false) const;

    void getWitnessPoints(WitnessPointVector& wp, bool include_env = false) const;



    /**
     * @brief return the approximate distance Jacobian; this assumes witness points do not
     * change with configuration
     * @note it requires calling update() and computeDistance() first
     */
    Eigen::MatrixXd getDistanceJacobian(bool include_env = false) const;

    /**
     * @brief return the approximate distance Jacobian; this assumes witness points do not
     * change with configuration
     * @note it requires calling update() and computeDistance() first
     * @param (output) the distance Jacobian; size must be getNumCollisionPairs() x model->getNv()
     */
    void getDistanceJacobian(MatRef J, bool include_env = false) const;

    /**
     * @brief returned the vector of collision pair indices, in ascending distance order
     */
    const std::vector<int>& getOrderedCollisionPairIndices() const;

    /**
     * @brief The ComputeCollisionFreeOptions class
     */
    struct ComputeCollisionFreeOptions
    {
        bool include_env;
        Eigen::VectorXd w_norm;
        int max_iter;
        double min_distance;

        ComputeCollisionFreeOptions();
    };

    /**
     * @brief computeNearestCollisionFree
     * @param q
     * @param w_norm
     * @return
     */
    bool computeCollisionFree(VecRef q,
                              ComputeCollisionFreeOptions opt = ComputeCollisionFreeOptions());

    virtual ~CollisionModel();

private:

    class Impl;

    std::unique_ptr<Impl> impl;

};


}

}

#endif // COLLISION_H
