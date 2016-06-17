#include <iostream>
#include <vector>

#include <lodepng.h>

int main()
{
  std::vector<unsigned char> buffer;
  std::cout << lodepng::load_file(buffer, "dummy.png") << '\n';
  return 0;
}
