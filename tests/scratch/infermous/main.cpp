#include <algorithm>
#include <iostream>
#include <iterator>

#include <infermous/CRF2D.h>
using namespace infermous;

int main()
{
  /*CRF2D<int> crf(10, 10);
  for(CRF2D<int>::iterator it = crf.nodes_begin(), iend = crf.nodes_end(); it != iend; ++it)
  {
    std::cout << it.x() << ' ' << it.y() << '\n';
    CRF2D<int>::Node& n = *it;
  }*/
  CRF2D<int>::ProbabilitiesGrid_Ptr unariesGrid(new CRF2D<int>::ProbabilitiesGrid(5, 5));
  CRF2D<int> crf(unariesGrid, 3);

  crf.update();

  // TODO
  return 0;
}
