/***
 * tvgutil: IDAllocator.h
 ***/

#ifndef H_TVGUTIL_IDALLOCATOR
#define H_TVGUTIL_IDALLOCATOR

#include <cstddef>
#include <set>

namespace tvgutil {

/**
 * \brief An instance of this class can be used to manage the allocation of integer IDs.
 *
 * (This was borrowed from my original implementation in hesperus.)
 */
class IDAllocator
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** A set of IDs that have previously been used but are now available for reallocation. */
  std::set<int> m_free;

  /** A set of IDs that are currently in use. */
  std::set<int> m_used;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Allocates a new ID.
   *
   * \return  The allocated ID.
   */
  int allocate();

  /**
   * \brief Deallocates the specified ID.
   *
   * \param[in] n The ID to deallocate.
   */
  void deallocate(int n);

  /**
   * \brief Resets the ID allocator.
   */
  void reset();

  /**
   * \brief Gets the set of IDs that are currently in use.
   *
   * \return  The set of IDs that are currently in use.
   */
  const std::set<int>& used() const;

  /**
   * \brief Gets the number of IDs that are currently in use.
   *
   * \return  The number of IDs that are currently in use.
   */
  size_t used_count() const;

  //#################### PRIVATE MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Gets the maximum ID currently in use.
   *
   * \return  The maximum ID currently in use.
   */
  int max_used() const;
};

}

#endif
