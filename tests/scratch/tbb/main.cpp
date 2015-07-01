#include <iostream>

#include "tbb/concurrent_unordered_map.h"

int main()
{
  const int N = 10;

  tbb::concurrent_unordered_map<int,int> m;

  for(int i = 0; i < N; ++i)
  {
    m[i] = i;
  }

  for(tbb::concurrent_unordered_map<int,int>::const_iterator it = m.begin(), iend = m.end(); it != iend; ++it)
  {
    std::cout << it->first << ' ' << it->second << '\n';
  }

  return 0;
}
