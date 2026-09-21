#include "param_store.hpp"
#include "autopilot_core.hpp"
#include <algorithm>
#include <iostream>
#include <limits>

ar::ControlParams sample() {
  ar::ControlParams p;
  p.K = .2; p.T = 8.;
  return p;
}
int main() {
  for (unsigned n : {111u, 112u}) {
    std::vector<uint8_t> disk;
    ar::ParamStore s(
      [&](uint8_t* b, size_t len) {
        std::copy_n(disk.data(), std::min(len,disk.size()), b); return true;
      },
      [&](const uint8_t* b, size_t len) { disk.assign(b,b+len); return true; });
    auto p = sample();
    p.schedTable.resize(n, ar::GainPoint{0., 1., 0, 1., .5});
    bool saved = s.save(p);
    ar::ControlParams out;
    bool loaded = s.load(out);
    std::cout << "rows=" << n << " bytes=" << disk.size() << " save=" << saved << " load=" << loaded << '\n';
  }
  {
    auto p = sample();
    p.schedTable = {{10.,0.,99,std::numeric_limits<double>::quiet_NaN(),-3.}};
    std::cout << "invalid_schedule_validate=" << p.validateRange() << '\n';
  }
  {
    auto p = sample(); p.version = 5; p.timestamp=123; p.updateCrc();
    auto before = p;
    ar::ParamStore s([](uint8_t*,size_t){return false;},
                    [](const uint8_t*,size_t){return false;}, []{return 456u;});
    bool saved = s.save(p);
    std::cout << "failed_save=" << saved << " version=" << before.version << "->" << p.version
              << " timestamp=" << before.timestamp << "->" << p.timestamp << " crc_changed=" << (before.crc != p.crc) << '\n';
  }
  std::cout << "default_config_validate=" << ar::ControlParams{}.validateRange() << '\n';
  std::cout << "host_sizes: ControlParams=" << sizeof(ar::ControlParams)
            << " GainPoint=" << sizeof(ar::GainPoint)
            << " AutoTuner=" << sizeof(ar::AutoTuner)
            << " AutopilotCore=" << sizeof(ar::AutopilotCore) << '\n';
}
