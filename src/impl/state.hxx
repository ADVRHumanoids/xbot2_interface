#ifndef STATE_HXX
#define STATE_HXX

#include <xbot2_interface/common/types.h>

#include "utils.h"

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
        qref, vref, tauref,
        qmin, qmax, vmax, taumax,
        qneutral;

    std::vector<std::string> qnames, vnames, jnames;

    const std::vector<Vec*> qs = {&qmot, &qlink, &qref, &qneutral};
    const std::vector<Vec*> vs = {&vmot, &vlink, &a, &tau, &k, &d, &vref, &tauref, &qmin, &qmax, &vmax, &taumax};
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

    std::vector<std::string> qnames, vnames, jnames;

    const std::vector<Vec*> qs = {&qcmd};
    const std::vector<Vec*> vs = {&vcmd, &taucmd, &kcmd, &dcmd};
    const std::vector<VecInt*> js = {&ctrlmode, &ctrlset};
};


template<class Container>
void qToMap(const Container& cnt, VecConstRef q, JointNameMap &qmap)
{
    for(int i = 0; i < cnt.qnames.size(); i++)
    {
        qmap[cnt.qnames[i]] = q[i];
    }
}

template<class Container>
void mapToQ(const Container& cnt, const JointNameMap& qmap, VecRef q)
{
    check_mat_size(q, cnt.qnames.size(), 1, __func__);

    for(int i = 0; i < cnt.qnames.size(); i++)
    {
        try
        {
            q[i] = qmap.at(cnt.qnames[i]);
        }
        catch (std::out_of_range&)
        {

        }
    }
}

template<class Container>
void vToMap(const Container& cnt, VecConstRef v, JointNameMap &vmap)
{
    for(int i = 0; i < cnt.vnames.size(); i++)
    {
        vmap[cnt.vnames[i]] = v[i];
    }
}

template<class Container>
void mapToV(const Container& cnt, const JointNameMap& vmap, VecRef v)
{
    check_mat_size(v, cnt.vnames.size(), 1, __func__);

    for(int i = 0; i < cnt.vnames.size(); i++)
    {
        try
        {
            v[i] = vmap.at(cnt.vnames[i]);
        }
        catch (std::out_of_range&)
        {

        }
    }
}

template<class Container>
void jToMap(const Container& cnt, CtrlModeVectorConstRef ctrl, CtrlModeTypeMap &ctrlmap)
{
    for(int i = 0; i < cnt.jnames.size(); i++)
    {
        ctrlmap[cnt.jnames[i]] = ControlMode::Type(ctrl[i]);
    }
}

template<class Container>
void jToMap(const Container& cnt, CtrlModeVectorConstRef ctrl, CtrlModeMap &ctrlmap)
{
    for(int i = 0; i < cnt.jnames.size(); i++)
    {
        ctrlmap[cnt.jnames[i]] = ControlMode::Type(ctrl[i]);
    }
}

template<class Container>
void mapToJ(const Container& cnt, const CtrlModeMap& ctrlmap, CtrlModeVectorRef ctrl)
{
    check_mat_size(ctrl, cnt.jnames.size(), 1, __func__);

    for(int i = 0; i < cnt.vnames.size(); i++)
    {
        try
        {
            ctrl[i] = ctrlmap.at(cnt.vnames[i]);
        }
        catch (std::out_of_range&)
        {

        }
    }
}

template<class Container>
void mapToJ(const Container& cnt, const CtrlModeTypeMap& ctrlmap, CtrlModeVectorRef ctrl)
{
    check_mat_size(ctrl, cnt.jnames.size(), 1, __func__);

    for(int i = 0; i < cnt.vnames.size(); i++)
    {
        try
        {
            ctrl[i] = ctrlmap.at(cnt.vnames[i]);
        }
        catch (std::out_of_range&)
        {

        }
    }
}

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

    cnt.qnames.assign(nq, "");

    cnt.vnames.assign(nv, "");

    cnt.jnames.assign(nj, "");

}

template <typename Vec>
typename StateTemplate<Vec>::ViewContainer createView(StateTemplate<Vec>& cnt,
                                                      int iq, int nq,
                                                      int iv, int nv,
                                                      int ij, int nj)
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
        cnt.tauref.segment(iv, nv),
        cnt.qmin.segment(iv, nv),
        cnt.qmax.segment(iv, nv),
        cnt.vmax.segment(iv, nv),
        cnt.taumax.segment(iv, nv),
        cnt.qneutral.segment(iq, nq)
    };

    ret.qnames.assign(cnt.qnames.begin() + iq, cnt.qnames.begin() + iq + nq);

    ret.vnames.assign(cnt.vnames.begin() + iv, cnt.vnames.begin() + iv + nv);

    ret.jnames.assign(cnt.jnames.begin() + ij, cnt.jnames.begin() + ij + nj);

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

    ret.qnames.assign(cnt.qnames.begin() + iq, cnt.qnames.begin() + iq + nq);

    ret.vnames.assign(cnt.vnames.begin() + iv, cnt.vnames.begin() + iv + nv);

    ret.jnames.assign(cnt.jnames.begin() + ij, cnt.jnames.begin() + ij + nj);

    return ret;
}

typedef StateTemplate<Eigen::VectorXd> State;
typedef CommandTemplate<Eigen::VectorXd, Eigen::CtrlModeVector> Command;

typedef StateTemplate<State::ViewType> StateView;
typedef CommandTemplate<Command::ViewType, Command::ViewTypeInt> CommandView;

} }

#endif // STATE_HXX
