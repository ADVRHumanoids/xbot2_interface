#ifndef STATE_HXX
#define STATE_HXX

#include <Eigen/Dense>

namespace XBot { namespace detail {

template <typename Vec>
struct StateTemplate
{
    typedef Eigen::Ref<Vec> ViewType;
    typedef StateTemplate<ViewType> ViewContainer;

    Vec qmot, qlink,
    vmot, vlink,
    a,
    tau,
    k, d,
    qref, vref, tauref;

    const std::vector<Vec*> qs = {&qmot, &qlink, &qref};
    const std::vector<Vec*> vs = {&vmot, &vlink, &a, &tau, &k, &d, &vref, &tauref};

};

template <typename Vec>
struct CommandTemplate
{
    typedef Eigen::Ref<Vec> ViewType;
    typedef CommandTemplate<ViewType> ViewContainer;

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

template <typename Vec>
typename StateTemplate<Vec>::ViewContainer createView(StateTemplate<Vec>& cnt, int iq, int nq, int iv, int nv)
{
    typename StateTemplate<Vec>::ViewContainer ret {
        cnt.qmot.segment(iq, nq),
        cnt.qlink.segment(iq, nq),
        cnt.vmot.segment(iv, nv),
        cnt.vlink.segment(iv, nv),
        cnt.a.segment(iv, nv),
        cnt.tau.segment(iv, nv),
        cnt.k.segment(iv, nv),
        cnt.d.segment(iv, nv),
        cnt.qref.segment(iq, nq),
        cnt.vref.segment(iv, nv),
        cnt.tauref.segment(iv, nv)
    };

    return ret;
}

template <typename Vec>
typename CommandTemplate<Vec>::ViewContainer createView(CommandTemplate<Vec>& cnt, int iq, int nq, int iv, int nv)
{
    typename CommandTemplate<Vec>::ViewContainer ret {
        cnt.qcmd.segment(iq, nq),
        cnt.vcmd.segment(iv, nv),
        cnt.taucmd.segment(iv, nv),
        cnt.kcmd.segment(iv, nv),
        cnt.dcmd.segment(iv, nv)
    };

    return ret;
}

typedef StateTemplate<Eigen::VectorXd> State;
typedef CommandTemplate<Eigen::VectorXd> Command;

typedef StateTemplate<State::ViewType> StateView;
typedef CommandTemplate<Command::ViewType> CommandView;

} }

#endif // STATE_HXX
