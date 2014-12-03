/**
 * infermous: CRF2D.h
 */

#ifndef H_INFERMOUS_CRF2D
#define H_INFERMOUS_CRF2D

#include <cassert>
#include <iterator>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace infermous {

template <typename Label>
class CRF2D
{
  //#################### NESTED TYPES ####################

public:
  class Node
  {
    //~~~~~~~~~~~~~~~~~~~~ PRIVATE VARIABLES ~~~~~~~~~~~~~~~~~~~~
  private:
    std::map<Label,float> m_marginals;
    std::map<Label,float> m_unaries;

    //~~~~~~~~~~~~~~~~~~~~ PUBLIC MEMBER FUNCTIONS ~~~~~~~~~~~~~~~~~~~~
  public:
    void set_unaries(const std::map<Label,float>& unaries)
    {
      m_unaries = unaries;
    }
  };

  //#################### TYPEDEFS ####################
private:
  typedef std::vector<Node> Grid;
  typedef boost::shared_ptr<Grid> Grid_Ptr;

  //#################### ITERATORS ####################
public:
  class iterator : public std::iterator<std::forward_iterator_tag,Node>
  {
  private:
    Grid_Ptr m_grid;
    size_t m_index;
    size_t m_width;
  private:
    iterator(const Grid_Ptr& grid, size_t width, size_t index)
    : m_grid(grid), m_index(index), m_width(width)
    {}
  public:
    Node& operator*()
    {
      assert(m_index < m_grid->size());
      return (*m_grid)[m_index];
    }

    Node *operator->()
    {
      return &operator*();
    }

    iterator& operator++()
    {
      assert(m_index < m_grid->size());
      ++m_index;
      return *this;
    }

    bool operator==(const iterator& rhs) const
    {
      return m_grid == rhs.m_grid && m_index == rhs.m_index;
    }

    bool operator!=(const iterator& rhs) const
    {
      return !(*this == rhs);
    }
  public:
    size_t index() const
    {
      return m_index;
    }

    size_t x() const
    {
      return m_index % m_width;
    }

    size_t y() const
    {
      return m_index / m_width;
    }

    friend CRF2D<Label>;
  };

  //#################### PRIVATE VARIABLES ####################
private:
  Grid_Ptr m_curGrid;
  Grid_Ptr m_prevGrid;

  size_t m_width;

  //#################### CONSTRUCTORS ####################
public:
  CRF2D(size_t width, size_t height)
  : m_width(width)
  {
    size_t size = width * height;
    m_curGrid.reset(new Grid(size));
    m_prevGrid.reset(new Grid(size));
  }

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  iterator nodes_begin()
  {
    return iterator(m_curGrid, m_width, 0);
  }

  iterator nodes_end()
  {
    return iterator(m_curGrid, m_width, m_curGrid->size());
  }
};

}

#endif
