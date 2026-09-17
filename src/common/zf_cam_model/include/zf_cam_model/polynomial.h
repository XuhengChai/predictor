#ifndef ZF_POLYNOMIAL_H
#define ZF_POLYNOMIAL_H

#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

#include "zf_global/in/zf_detect_global.h"

BEGIN_NS_ZF_DETECTION
// // f(x) = 1 + 2 * x^2 + 3 * x^3 + 5 * x^5
// Polynomial poly;
// poly[0] = 1.0;
// poly[2] = 2.0;
// poly[3] = 3.0;
// poly[5] = 5.0;

// EXPECT_NEAR(poly[0], 1.0, 1e-8);
// EXPECT_NEAR(poly[1], 0.0, 1e-8);
// EXPECT_NEAR(poly[2], 2.0, 1e-8);
// EXPECT_NEAR(poly[3], 3.0, 1e-8);
// EXPECT_NEAR(poly[4], 0.0, 1e-8);
// EXPECT_NEAR(poly[5], 5.0, 1e-8);

// EXPECT_NEAR(poly(0.0), 1.0, 1e-6);
// EXPECT_NEAR(poly(1.0), 11.0, 1e-6);
// // f(x) = 1 + 2 * x^2 + 3 * x^3 + 5 * x^5 + 6 * x^6
// poly[6] = 6.0;
// EXPECT_NEAR(poly(1.0), 17.0, 1e-6);
class Polynomial {
 public:
  Polynomial();
  ~Polynomial();
  double& operator[](const uint32_t& order);
  // sum(coeff_[i] * x^i)
  double operator()(const double& x);

  const std::map<uint32_t, double>& getCoeff() const;

 private:
  std::map<uint32_t, double> coeff_;
  std::vector<uint32_t> index_gap_;
  std::unordered_map<uint32_t, double> power_cache_;
  bool initialized_ = false;
};

std::ostream& operator<<(std::ostream& o, const Polynomial& p);

END_NS_ZF_DETECTION
#endif  // ZF_POLYNOMIAL_H