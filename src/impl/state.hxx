#ifndef STATE_HXX
#define STATE_HXX

#include <xbot2_interface/common/types.h>

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
    const std::vector<Vec*> js = {};

};

template <typename Vec, typename VecInt>
struct CommandTemplate
{
    typedef Eigen::Ref<Vec> ViewType;
    typedef Eigen::Ref<VecInt> ViewTypeInt;
    typedef CommandTemplate<ViewType, ViewTypeInt> ViewContainer;

    Vec qcmd, vcmd, taucmd, kcmd, dcmd;
    VecInt ctrlmode;
    VecInt ctrlset;

    const std::vector<Vec*> qs = {&qcmd};
    const std::vector<Vec*> vs = {&vcmd, &taucmd, &kcmd, &dcmd};
    const std::vector<VecInt*> js = {&ctrlmode, &ctrlset};
};


template <typename T>
void resize(T& cnt, int nq, int nv, int nj)
{
    for(int i = 0; i < cnt.qs.size(); i++)
    {
        cnt.qs[i]->setZero(nq);
    }

    for(int i = 0; i < cnt.vs.size(); i++)
    {
        cnt.vs[i]->setZero(nv);
    }

    for(int i = 0; i < cnt.js.size(); i++)
    {
        cnt.js[i]->setZero(nj);
    }
}

template <typename Vec>
typename StateTemplate<Vec>::ViewContainer createView(StateTemplate<Vec>& cnt,
                                                      int iq, int nq,
                                                      int iv, int nv)
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

template <typename Vec, typename VecInt>
typename CommandTemplate<Vec, VecInt>::ViewContainer createView(CommandTemplate<Vec, VecInt>& cnt,
                                                                int iq, int nq,
                                                                int iv, int nv,
                                                                int ij, int nj)
{
    typename CommandTemplate<Vec, VecInt>::ViewContainer ret {
        cnt.qcmd.segment(iq, nq),
        cnt.vcmd.segment(iv, nv),
        cnt.taucmd.segment(iv, nv),
        cnt.kcmd.segment(iv, nv),
        cnt.dcmd.segment(iv, nv),
        cnt.ctrlmode.segment(ij, nj),
        cnt.ctrlset.segment(ij, nj)
    };

    return ret;
}

typedef StateTemplate<Eigen::VectorXd> State;
typedef CommandTemplate<Eigen::VectorXd, Eigen::CtrlModeVector> Command;

typedef StateTemplate<State::ViewType> StateView;
typedef CommandTemplate<Command::ViewType, Command::ViewTypeInt> CommandView;

} }

#endif // STATE_HXX
