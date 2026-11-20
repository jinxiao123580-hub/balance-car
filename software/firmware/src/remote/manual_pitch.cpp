#include "remote/manual_pitch.h"

namespace remote {

// Parse the entire frame before changing either axis: ^ turnHz,pitchDegrees.
bool parseManualMotion(const char * value, float & turn, float & forward) {
  char * end = nullptr;
  float t = std::strtof(value, & end);
  if (end == value || * end != ',' || !std::isfinite(t)) return false;
  const char * second = end + 1;
  float f = std::strtof(second, & end);
  if (end == second || * end != '\0' || !std::isfinite(f)) return false;
  turn = t;
  forward = f;
  return true;
}

}  // namespace remote
