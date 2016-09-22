#########################
# UseScoreForests.cmake #
#########################

SET(ScoreForests_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../scoreforests/" CACHE PATH "The ScoreForests main directory")
SET(ScoreForests_BUILD_DIR "${PROJECT_SOURCE_DIR}/../scoreforests/build" CACHE PATH "The ScoreForests build directory")

FIND_LIBRARY(ScoreForests_Datasets_LIBRARY_DEBUG Datasets HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Datasets/Debug")
FIND_LIBRARY(ScoreForests_Features_LIBRARY_DEBUG Features HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Features/Debug")
FIND_LIBRARY(ScoreForests_Hashes_LIBRARY_DEBUG Hashes HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Hashes/Debug")
FIND_LIBRARY(ScoreForests_Helpers_LIBRARY_DEBUG Helpers HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Helpers/Debug")
FIND_LIBRARY(ScoreForests_Learners_LIBRARY_DEBUG Learners HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Learners/Debug")
FIND_LIBRARY(ScoreForests_StatisticalTools_LIBRARY_DEBUG StatisticalTools HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/StatisticalTools/Debug")

FIND_LIBRARY(ScoreForests_Datasets_LIBRARY_RELEASE Datasets HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Datasets/Release")
FIND_LIBRARY(ScoreForests_Features_LIBRARY_RELEASE Features HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Features/Release")
FIND_LIBRARY(ScoreForests_Hashes_LIBRARY_RELEASE Hashes HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Hashes/Release")
FIND_LIBRARY(ScoreForests_Helpers_LIBRARY_RELEASE Helpers HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Helpers/Release")
FIND_LIBRARY(ScoreForests_Learners_LIBRARY_RELEASE Learners HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/Learners/Release")
FIND_LIBRARY(ScoreForests_StatisticalTools_LIBRARY_RELEASE StatisticalTools HINTS "${ScoreForests_BUILD_DIR}/lib" "${ScoreForests_BUILD_DIR}/StatisticalTools/Release")

SET(ScoreForests_Datasets_LIBRARY debug ${ScoreForests_Datasets_LIBRARY_DEBUG} optimized ${ScoreForests_Datasets_LIBRARY_RELEASE})
SET(ScoreForests_Features_LIBRARY debug ${ScoreForests_Features_LIBRARY_DEBUG} optimized ${ScoreForests_Features_LIBRARY_RELEASE})
SET(ScoreForests_Hashes_LIBRARY debug ${ScoreForests_Hashes_LIBRARY_DEBUG} optimized ${ScoreForests_Hashes_LIBRARY_RELEASE})
SET(ScoreForests_Helpers_LIBRARY debug ${ScoreForests_Helpers_LIBRARY_DEBUG} optimized ${ScoreForests_Helpers_LIBRARY_RELEASE})
SET(ScoreForests_Learners_LIBRARY debug ${ScoreForests_Learners_LIBRARY_DEBUG} optimized ${ScoreForests_Learners_LIBRARY_RELEASE})
SET(ScoreForests_StatisticalTools_LIBRARY debug ${ScoreForests_StatisticalTools_LIBRARY_DEBUG} optimized ${ScoreForests_StatisticalTools_LIBRARY_RELEASE})

SET(ScoreForests_LIBRARIES ${ScoreForests_Datasets_LIBRARY} ${ScoreForests_Features_LIBRARY} ${ScoreForests_Hashes_LIBRARY} ${ScoreForests_Helpers_LIBRARY} ${ScoreForests_Learners_LIBRARY} ${ScoreForests_StatisticalTools_LIBRARY})

INCLUDE_DIRECTORIES("${ScoreForests_SOURCE_DIR}/modules/Datasets/include/Datasets")
INCLUDE_DIRECTORIES("${ScoreForests_SOURCE_DIR}/modules/Features/include/Features")
INCLUDE_DIRECTORIES("${ScoreForests_SOURCE_DIR}/modules/Hashes/include/Hashes")
INCLUDE_DIRECTORIES("${ScoreForests_SOURCE_DIR}/modules/Helpers/include/Helpers")
INCLUDE_DIRECTORIES("${ScoreForests_SOURCE_DIR}/modules/Learners/include/Learners")
INCLUDE_DIRECTORIES("${ScoreForests_SOURCE_DIR}/modules/StatisticalTools/include/StatisticalTools")

ADD_DEFINITIONS(-DUSING_CMAKE)

