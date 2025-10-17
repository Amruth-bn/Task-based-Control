#include <simulator.h>
#include <feature_stack.h>

using std::string;

int main()
{
  Simulator sim;
  FeatureStack stack(sim);

  const auto config{stack.config()};

  const auto useXY     = config.read<bool>("useXY");
  const auto usePolar  = config.read<bool>("usePolar");
  const auto use2Half  = config.read<bool>("use2Half");

  const auto err_min   = config.read<double>("errMin");
  const auto lambda    = config.read<double>("lambda");
  const auto iter_max  = config.read<uint>("iterMax");

  if(useXY) {
    for(auto point: sim.observedPoints())
      stack.addFeaturePoint(point, PointDescriptor::XY);
  }

  if(usePolar) {
    for(auto point: sim.observedPoints())
      stack.addFeaturePoint(point, PointDescriptor::Polar);
  }

  // 2½D: CoG XY + CoG depth (+ rotation is handled below by config)
  if(use2Half) {
    stack.addFeaturePoint(sim.cog(), PointDescriptor::XY);
    stack.addFeaturePoint(sim.cog(), PointDescriptor::Depth);
    // If you want to force a rotation here instead of config, uncomment one:
    // stack.setRotation3D("cdRc");
    // stack.setRotation3D("cRcd");
  }

  // 3D features from config ("none" means no-op inside the stack)
  stack.setTranslation3D(config.read<string>("translation3D"));
  stack.setRotation3D(config.read<string>("rotation3D"));

  stack.summary();

  uint iter(0);
  const auto sd = stack.sd();
  vpColVector s(sd.size(), err_min);

  while(iter++ < iter_max && (s - sd).frobeniusNorm() > err_min && !sim.clicked())
  {
    stack.updateFeatures(sim.currentPose());

    // current features and interaction matrix
    s = stack.s();
    const vpMatrix L = stack.L();

    // control law: v = -lambda * L^+ * (s - sd)
    const vpColVector e = s - sd;
    const vpMatrix Lp = L.pseudoInverse();     // ViSP pinv
    const vpColVector v = -lambda * (Lp * e); // 6x1 twist

    sim.setVelocity(v);
  }

  sim.waitForClick();
  sim.plot();
}
