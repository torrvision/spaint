/**
 * spaint: LabelManager.cpp
 */

#ifndef H_SPAINT_LABELMANAGER
#define H_SPAINT_LABELMANAGER

#include <map>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <tvgutil/IDAllocator.h>

#include "SpaintVoxel.h"

namespace spaint {

/**
 * \brief An instance of this class can be used to manage the labels that are used for labelling a scene.
 */
class LabelManager
{
  //#################### PRIVATE VARIABLES ####################
private:
  /** The ID allocator used to allocate labels. */
  tvgutil::IDAllocator m_labelAllocator;

  /** A map from label names to labels. */
  std::map<std::string,SpaintVoxel::Label> m_labelsByName;

  /** A map from labels to their properties (names and colours). */
  std::map<SpaintVoxel::Label,std::pair<std::string,Vector3u> > m_labelProperties;

  /** The maximum number of labels that the manager is allowed to allocate. */
  size_t m_maxLabelCount;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs a label manager.
   *
   * Note: The maximum number of labels that can be specified is limited by the number of available colours (currently 20).
   *
   * \param maxLabelCount The maximum number of labels that the manager is allowed to allocate.
   */
  explicit LabelManager(size_t maxLabelCount);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /**
   * \brief Attempts to add a label with the specified name.
   *
   * A label will only be added if a label with the specified name does not
   * already exist and we have not yet reached the maximum label count.
   *
   * \param name  The name of the label we want to add.
   * \return      true, if we successfully added the label, or false otherwise.
   */
  bool add_label(const std::string& name);

  /**
   * \brief Gets the label with the specified name.
   *
   * \param name                The name of the label we want to get.
   * \return                    The label with the specified name.
   * \throws std::runtime_error If the manager does not contain a label with the specified name.
   */
  SpaintVoxel::Label get_label(const std::string& name) const;

  /**
   * \brief Gets the colour of the specified label.
   *
   * \param label               The label whose colour we want to get.
   * \return                    The colour of the specified label.
   * \throws std::runtime_error If the manager does not contain the specified label.
   */
  Vector3u get_label_colour(SpaintVoxel::Label label) const;

  /**
   * \brief Gets all of the available label colours.
   *
   * \return  The available label colours.
   */
  const std::vector<Vector3u>& get_label_colours() const;

  /**
   * \brief Gets the number of labels that are currently allocated.
   *
   * \return  The number of labels that are currently allocated.
   */
  size_t get_label_count() const;

  /**
   * \brief Gets the name of the specified label.
   *
   * \param label               The label whose name we want to get.
   * \return                    The name of the specified label.
   * \throws std::runtime_error If the manager does not contain the specified label.
   */
  std::string get_label_name(SpaintVoxel::Label label) const;

  /**
   * \brief Gets the maximum number of labels that the manager is allowed to allocate.
   *
   * \return  The maximum number of labels that the manager is allowed to allocate.
   */
  size_t get_max_label_count() const;

  /**
   * \brief Gets the label directly succeeding the specified label in the label order (if any).
   *
   * \param label The label whose successor we want to get.
   * \return      The label directly succeeding the specified label in the label order (if any), or the specified label if it's the last label.
   */
  SpaintVoxel::Label get_next_label(SpaintVoxel::Label label) const;

  /**
   * \brief Gets the label directly preceding the specified label in the label order (if any).
   *
   * \param label The label whose predecessor we want to get.
   * \return      The label directly preceding the specified label in the label order (if any), or the specified label if it's the first label.
   */
  SpaintVoxel::Label get_previous_label(SpaintVoxel::Label label) const;

  /**
   * \brief Gets whether or not the manager contains the specified label.
   *
   * \param name  The label we want to check.
   * \return      true, if the manager contains the specified label, or false otherwise.
   */
  bool has_label(SpaintVoxel::Label label) const;

  /**
   * \brief Gets whether or not the manager contains a label with the specified name.
   *
   * \param name  The name of the label we want to check.
   * \return      true, if the manager contains a label with the specified name, or false otherwise.
   */
  bool has_label(const std::string& name) const;
};

//#################### TYPEDEFS ####################

typedef boost::shared_ptr<LabelManager> LabelManager_Ptr;
typedef boost::shared_ptr<const LabelManager> LabelManager_CPtr;

}

#endif
