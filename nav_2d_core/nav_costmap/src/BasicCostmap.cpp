#include "uavia_riskmap/BasicRiskmap.h"

namespace uavia_riskmap {

void BasicRiskmap::reset() {
  _data.assign(_info._width * _info._height, this->_defaultValue);
}

unsigned int BasicRiskmap::getIndex(unsigned int x, unsigned int y) const {
  return y * _info._width + x;
}

unsigned char BasicRiskmap::getValue(unsigned int x, unsigned int y) const {
  return _data[getIndex(x, y)];
}

void BasicRiskmap::setValue(unsigned int x, unsigned int y,
                            const unsigned char& value) {
  _data[getIndex(x, y)] = value;
}

}  // namespace uavia_riskmap
