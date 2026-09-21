#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include "autopilot_core.hpp"

int main() {
  std::cout << std::setprecision(12);
  ar::PdController q;
  q.setDt(0.016);
  std::cout << "quant_rate dt=0.016 cmd=" << q.saturate(10) << " bound=0.16\n";
  q.reset(); q.setDt(0.004);
  double out = 0;
  for (int i=0;i<1000;i++) out=q.saturate(10);
  std::cout << "quant_stall dt=0.004 after1000=" << out << "\n";
  auto qp=q.params(); qp.dMax=34.96; qp.ddMax=100000; qp.dt=.02;
  q.setParams(qp);
  std::cout << "quant_amplitude dMax=34.96 cmd=" << q.saturate(35) << "\n";

  ar::PdController::Params dp;
  dp.Kp=0; dp.Kd=1; dp.tau=0; dp.deadzone=0;
  dp.ddMax=100000; dp.quant=.001;
  ar::PdController wrap(dp);
  wrap.update(0,-179.9);
  std::cout << "wrap_derivative cmd=" << wrap.update(0,179.9) << " shortestDiffExpected=10\n";

  dp.tau=.2; dp.deadzone=.3;
  ar::PdController dead(dp);
  dead.update(1,0); dead.update(5,0);
  for(int i=0;i<1000;i++) dead.update(0,0);
  std::cout << "deadzone_resume cmd=" << dead.update(.31,0) << " continuousFilterExpected=" << .31/.22 << "\n";

  ar::AutoTuner::Params tp; tp.wn=.5;
  ar::AutoTuner tuner(tp);
  ar::IdSnapshot id{true,true,.2,8};
  const bool established=tuner.establishBaseline(id);
  tuner.tick({0,0,0,5,.02,1,.5,id});
  auto g=tuner.tick({0,0,0,5,.02,1,.5,id});
  std::cout << "dither established="<<established<<" kp="<<g.Kp<<" kd="<<g.Kd<<" wn="<<std::sqrt(g.Kp*.2/8)<<" stability="<<tuner.stabilityCheck(.2,8,g.Kp,g.Kd,0)<<"\n";

  ar::AutoTuner osc;
  osc.establishBaseline(id);
  for(int i=0;i<1600;i++) osc.tick({0,0,0,5,.02,1,.5,id});
  for(double heading:{-10.0,10.0,-10.0}) osc.tick({0,heading,0,5,.02,1,.5,id});
  std::cout << "osc_before_suspend="<<osc.oscDetector().amplitude();
  osc.suspend();
  osc.tick({1,1,0,5,.02,1,.5,id});
  std::cout << " after_resume_changed_ref="<<osc.oscDetector().amplitude()<<"\n";

  ar::NomotoIdentifier rejected, elapsed;
  rejected.update(10,0,1); elapsed.update(10,0,1);
  rejected.update(10,std::numeric_limits<double>::quiet_NaN(),1);
  rejected.update(10,2,1);
  elapsed.update(10,2,2);
  std::cout << "rls_badgap k="<<rejected.K()<<" t="<<rejected.T()<<" elapsedCorrect k="<<elapsed.K()<<" t="<<elapsed.T()<<"\n";
}
