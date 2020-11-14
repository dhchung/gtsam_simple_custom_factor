#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>


using namespace gtsam;
class CalculateTransformation{
public:
    Vector3 inGlobal(Vector3 in, Vector3 dstate);

};