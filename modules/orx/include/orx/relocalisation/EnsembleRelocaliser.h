/**
 * orx: EnsembleRelocaliser.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2019. All rights reserved.
 */

#ifndef H_ORX_ENSEMBLERELOCALISER
#define H_ORX_ENSEMBLERELOCALISER

#include "Relocaliser.h"

namespace orx {

/**
 * \brief An instance of this class represents an ensemble relocaliser that combines the results of several other relocalisers.
 */
class EnsembleRelocaliser : public Relocaliser
{
  //#################### PRIVATE MEMBER VARIABLES ####################
private:
  /** The individual relocalisers in the ensemble. */
  std::vector<Relocaliser_Ptr> m_innerRelocalisers;

  //#################### CONSTRUCTORS ####################
public:
  /**
   * \brief Constructs an ensemble relocaliser.
   *
   * \param innerRelocalisers The individual relocalisers in the ensemble.
   */
  explicit EnsembleRelocaliser(const std::vector<Relocaliser_Ptr>& innerRelocalisers);

  //#################### COPY CONSTRUCTOR & ASSIGNMENT OPERATOR ####################
private:
  // Deliberately private and unimplemented.
  EnsembleRelocaliser(const EnsembleRelocaliser&);
  EnsembleRelocaliser& operator=(const EnsembleRelocaliser&);

  //#################### PUBLIC MEMBER FUNCTIONS ####################
public:
  /** Override */
  virtual void finish_training();

  /** Override */
  virtual ORUChar4Image_CPtr get_visualisation_image(const std::string& key) const;

  /** Override */
  virtual void load_from_disk(const std::string& inputFolder);

  /** Override */
  virtual std::vector<Result> relocalise(const ORUChar4Image *colourImage, const ORFloatImage *depthImage, const Vector4f& depthIntrinsics) const;

  /** Override */
  virtual void reset();

  /** Override */
  virtual void save_to_disk(const std::string& outputFolder) const;

  /** Override */
  virtual void train(const ORUChar4Image *colourImage, const ORFloatImage *depthImage,
                     const Vector4f& depthIntrinsics, const ORUtils::SE3Pose& cameraPose);

  /** Override */
  virtual void update();

  //#################### PRIVATE STATIC MEMBER FUNCTIONS ####################
private:
  /**
   * \brief Compares two relocalisation results.
   *
   * \param lhs The left-hand result.
   * \param rhs The right-hand result.
   * \return    true, if the left-hand result should be ordered before the right-hand result, or false otherwise.
   */
  static bool compare_results(const Result& lhs, const Result& rhs);
};

}

#endif
