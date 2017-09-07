#include <iostream>

#include <alglib/optimization.h>

int main()
{
  double arr[] = { 23, 9, 84 };
  alglib::real_1d_array rarr;
  rarr.setcontent(sizeof(arr) / sizeof(double), arr);
  std::cout << rarr.tostring(1) << '\n';
  return 0;
}
