

#pragma once

#include <string>
#include <vector>

#include "uavia_riskmap/Riskmap.h"

namespace uavia_riskmap {

class BasicRiskmap : public Riskmap {
 public:
  // Riskmap Interface
  mutex_t getMutex() override { return _mutex; }

  // NavGrid Interface
  void reset() override;
  void setValue(unsigned int x, unsigned int y,
                const unsigned char& value) override;
  unsigned char getValue(unsigned int x, unsigned int y) const override;
  void setInfo(const uavia_nav_grid::NavGridInfo& info) override {
    _info = info;
    reset();
  }

  // Index Conversion
  unsigned int getIndex(unsigned int x, unsigned int y) const;

 protected:
  mutex_t _mutex;
  std::vector<unsigned char> _data;
};

}  // namespace uavia_riskmap
