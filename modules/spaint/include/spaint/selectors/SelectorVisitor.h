/**
 * spaint: SelectorVisitor.h
 */

#ifndef H_SPAINT_SELECTORVISITOR
#define H_SPAINT_SELECTORVISITOR

namespace spaint {

//#################### FORWARD DECLARATIONS ####################

class NullSelector;
class PickingSelector;

/**
 * \brief An instance of a class deriving from this one can be used to visit selectors (e.g. for the purpose of rendering them).
 */
class SelectorVisitor
{
  //#################### DESTRUCTOR ####################
public:
  /**
   * \brief Destroys the visitor.
   */
  virtual ~SelectorVisitor() = 0;

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Visits a null selector.
   *
   * \param selector  The selector to visit.
   */
  virtual void visit(const NullSelector& selector) const;

  /**
   * \brief Visits a picking selector.
   *
   * \param selector  The selector to visit.
   */
  virtual void visit(const PickingSelector& selector) const;
};

}

#endif
