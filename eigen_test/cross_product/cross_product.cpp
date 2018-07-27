#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
int main()
{
  Vector3d v(1,2,3);
  Vector3d w(0,1,2);
  cout << "Cross product:\n" << v.cross(w) << endl;
  Vector3d v4(1,2,3,0);
  Vector3d w4(0,1,2,0);
  cout << "4D Cross product:\n" << v4.cross(w4) << endl;
}
