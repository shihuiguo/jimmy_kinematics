#ifndef Transform_h_DEFINED
#define Transform_h_DEFINED

#include <vector>
#include <stdio.h>

class Transform {
public:
  Transform();
  virtual ~Transform() {}
  double* getTranslation();
  void clear();
  Transform &translate(double x, double y, double z);
  Transform &translateOrigin(double x, double y, double z);
  Transform &translateX(double x = 0);
  Transform &translateY(double y = 0);
  Transform &translateZ(double z = 0);
  Transform &rotateX(double a = 0);
  Transform &rotateY(double a = 0);
  Transform &rotateZ(double a = 0);
  Transform &mDH(double alpha, double a, double theta, double d);
  //Transform &SetTransform(Vec3 pos, Vec3 rot);
  void apply(double x[3]);
  void print();
  double& operator() (int i, int j);
  const double operator() (int i, int j) const;

 private:
  double t[4][4];
};

Transform operator* (const Transform &t1, const Transform &t2);
Transform inv (const Transform &t1);
Transform transform6D(const double p[6]);
std::vector<double> position6D(const Transform &t1);

#endif
