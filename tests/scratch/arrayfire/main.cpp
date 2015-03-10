#include <stdio.h>
#include <arrayfire.h>
#include <af/util.h>

#define fooprint(bar) af::print(#bar, af::reorder(bar, 1, 0, 2, 3).as(u32))

int main()
{
  try
  {
    af::array A = floor(20 * af::randu(10, 5, 2, 2)).as(u32);
    A = A * (A > 10);
    fooprint(A);
    fooprint(af::where(A).as(u32));
  }
  catch(af::exception &ae)
  {
    std::cout << ae.what();
  }
  return 0;
}
