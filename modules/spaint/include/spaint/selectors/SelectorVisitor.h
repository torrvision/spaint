/**
 * spaint: SelectorVisitor.h
 */

#ifndef H_SPAINT_SELECTORVISITOR
#define H_SPAINT_SELECTORVISITOR

namespace spaint {

//#################### FORWARD DECLARATIONS ####################

#ifdef WITH_LEAP
class LeapSelector;
#endif
class NullSelector;
class PickingSelector;
#ifdef WITH_ARRAYFIRE
class TouchSelector;
#endif

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
#ifdef WITH_LEAP
  /**
   * \brief Visits a Leap selector.
   *
   * \param selector  The selector to visit.
   */
  virtual void visit(const LeapSelector& selector) const;
#endif

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

#ifdef WITH_ARRAYFIRE
  /**
   * \brief Visits a Touch selector.
   *
   * \param selector  The selector to visit.
   */
  virtual void visit(const TouchSelector& selector) const;
#endif
};

}

#endif
