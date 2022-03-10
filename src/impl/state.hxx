#ifndef STATE_HXX
#define STATE_HXX

#include <Eigen/Dense>

namespace XBot { namespace detail {

template <typename Vec>
struct StateTemplate
{
    typedef Eigen::Ref<Vec> ViewType;

    Vec qmot, qlink,
        vmot, vlink,
        a,
        tau,
        k, d,
        qref, vref, tauref;

    const std::vector<Vec*> qs = {&qmot, &qlink, &qref, &vref, &tauref};
    const std::vector<Vec*> vs = {&vlink, &a, &tau, &k, &d, &vref, &tauref};

};

template <typename Vec>
struct CommandTemplate
{
    typedef Eigen::Ref<Vec> ViewType;

    Vec qcmd, vcmd, taucmd, kcmd, dcmd;

    const std::vector<Vec*> qs = {&qcmd};
    const std::vector<Vec*> vs = {&vcmd, &taucmd, &kcmd, &dcmd};
};


template <typename T>
void resize(T& cnt, int nq, int nv)
{
    for(int i = 0; i < cnt.qs.size(); i++)
    {
        cnt.qs[i]->setZero(nq);
    }

    for(int i = 0; i < cnt.vs.size(); i++)
    {
        cnt.vs[i]->setZero(nv);
    }
}

template <typename T>
typename T::ViewType createView(T& cnt, int iq, int nq, int iv, int nv)
{
    typename T::ViewType ret;

    for(int i = 0; i < cnt.qs.size(); i++)
    {
        *cnt.ret.qs[i] = cnt.qs[i]->segment(iq, nq);
    }

    for(int i = 0; i < cnt.vs.size(); i++)
    {
        *cnt.ret.vs[i] = cnt.vs[i]->segment(iv, nv);
    }

    return ret;
}


typedef StateTemplate<Eigen::VectorXd> State;
typedef CommandTemplate<Eigen::VectorXd> Command;

typedef Eigen::VectorBlock<Eigen::VectorXd> ViewType;

typedef StateTemplate<ViewType> StateView;
typedef CommandTemplate<ViewType> CommandView;

}
               }

#endif // STATE_HXX
