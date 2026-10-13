#pragma once

#include <cstdint>

namespace ui::radar {

constexpr bool frameSpriteEnabledForBoard(bool) {
  return true;
}

constexpr uint8_t frameSpriteColorDepthForBoard(bool) {
  return 8;
}

}  // namespace ui::radar
