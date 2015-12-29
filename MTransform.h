#ifndef MTransform_h_DEFINED
#define MTransform_h_DEFINED

#include <vector>
#include <stdio.h>

class MTransform {
public:
  MTransform();
  virtual ~MTransform() {}
  double* getTranslation();
  void clear();
  MTransform &translate(double x, double y, double z);
  MTransform &translateOrigin(double x, double y, double z);
  MTransform &translateX(double x = 0);
  MTransform &translateY(double y = 0);
  MTransform &translateZ(double z = 0);
  MTransform &rotateX(double a = 0);
  MTransform &rotateY(double a = 0);
  MTransform &rotateZ(double a = 0);
  MTransform &mDH(double alpha, double a, double theta, double d);
  void apply(double x[3]);
  void print();
  double& operator() (int i, int j);
  const double operator() (int i, int j) const;

 private:
  double t[4][4];
};

#endif
