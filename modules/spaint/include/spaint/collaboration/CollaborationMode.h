/**
 * spaint: CollaborationMode.h
 * Copyright (c) Torr Vision Group, University of Oxford, 2017. All rights reserved.
 */

#ifndef H_SPAINT_COLLABORATIONMODE
#define H_SPAINT_COLLABORATIONMODE

namespace spaint {

/**
 * \brief The values of this enumeration denote the different modes in which we can run a collaborative reconstruction.
 */
enum CollaborationMode
{
  /**
   * In batch mode, no inter-agent relocalisation is performed until the scenes for all of the individual agents have been reconstructed.
   * Relocalisations are scheduled to run back-to-back, since all of the individual agents' relocalisers have finished training.
   */
  CM_BATCH,

  /**
   * In live mode, inter-agent relocalisation is performed whilst the scenes for the individual agents are still being reconstructed.
   * Relocalisations are scheduled periodically, to give the individual agents' relocalisers time to train.
   */
  CM_LIVE
};

}

#endif
