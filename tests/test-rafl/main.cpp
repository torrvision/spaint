#include <rafl/base/Example.h>
using namespace rafl;

int main()
{
  Descriptor_Ptr d(new Descriptor(10, 23));
  Example<int> x1(d, 23);
  Example<int> x2(Descriptor_Ptr(), 23);
  return 0;
}
