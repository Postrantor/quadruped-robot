/**
 * @brief
 * @date 2024-02-25
 * @copyright Copyright (c) 2017, Alexander W. Winkler. All rights reserved.
 */

/**
 * @file endeffector_mappings.h
 *
 * Assigning some semantic information (e.g. name of the foot) to endeffector
 * indices.
 */
#ifndef XPP_STATES_ENDEFFECTOR_MAPPINGS_H_
#define XPP_STATES_ENDEFFECTOR_MAPPINGS_H_

#include <map>
#include <string>

namespace xpp {

namespace biped {
enum FootIDs { L=0, R };
static std::map<FootIDs, std::string> foot_to_name =
{
  {L, "Left" },
  {R, "Right"}
};
}

namespace quad {
enum FootIDs { LF=0, RF, LH, RH };
static std::map<FootIDs, std::string> foot_to_name =
{
  {LF, "Left-Front" },
  {RF, "Right-Front"},
  {LH, "Left-Hind"  },
  {RH, "Right-Hind" }
};
}

} // namespace xpp

#endif /* XPP_STATES_ENDEFFECTOR_MAPPINGS_H_ */
