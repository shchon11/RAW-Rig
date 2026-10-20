// Preview rendering from a Bayer RAW frame, for image_rgb/compressed in RAW capture profiles.
//
// The camera's own 8-bit path (ImageProcessor -> RGB8) keeps only the top 8 bits of a 16-bit RAW
// value, so a low-light RAW frame previews as black (G median 1472 -> 5 on 3DRAW scene_020). This
// renders a display-only picture instead: one output pixel per 2x2 Bayer cell (averaged over
// `downscale` x `downscale` cells), black subtracted, gray-world white balance, auto (or fixed)
// digital gain and a power-law gamma. It is for framing and focusing the scene in the GUI; it is
// not a reference ISP and nothing downstream should treat it as one.
//
// Header-only and SDK-free: the same file is in FLIR_control (flir_spinnaker_camera/src/raw_preview.hpp).

#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace raw_preview
{

enum class Cfa { kRGGB, kBGGR, kGRBG, kGBRG };

inline bool ParseCfa(const std::string & name, Cfa * cfa)
{
  if (name == "RGGB") {*cfa = Cfa::kRGGB; return true;}
  if (name == "BGGR") {*cfa = Cfa::kBGGR; return true;}
  if (name == "GRBG") {*cfa = Cfa::kGRBG; return true;}
  if (name == "GBRG") {*cfa = Cfa::kGBRG; return true;}
  return false;
}

struct Settings
{
  double max_rate_hz{5.0};      // preview frames per second at most (0 = every frame)
  int downscale{1};             // 1: one pixel per 2x2 cell (half resolution), 2: quarter, ...
  double gain{0.0};             // fixed digital gain; 0 = auto (target_mean)
  double target_mean{0.18};     // auto gain: mean G after black, as a fraction of full scale
  double max_gain{64.0};        // auto gain ceiling (multiple of full scale)
  // Auto gain is also kept low enough that the dark-end noise (estimated from the spread between the
  // 0.5th and 5th percentile of G) stays below this fraction of the display range: a capped lens then
  // shows dark instead of gray noise, whatever the container's full scale (12-bit channel or 24-bit HDR).
  double noise_ceiling{0.02};
  double gamma{2.2};            // display gamma (output = linear^(1/gamma))
  bool gray_world{true};        // white balance by channel means; false = raw sensor colors
  double black_level_dn{-1.0};  // < 0: estimate (0.5th percentile of the G plane)
  int jpeg_quality{80};
};

// Carried between frames so the auto gain does not flicker.
struct State
{
  bool valid{false};
  double log_gain{0.0};
  double last_black{0.0};
  double last_gain{1.0};
};

// Renders `data` (row-major, `stride` elements per row) into a BGR8 image. `white_level` is the
// full-scale code of the container (e.g. 65520 for BayerRG16, 4095 for a 12-bit channel).
template<typename T>
cv::Mat Render(
  const T * data, int width, int height, std::size_t stride, Cfa cfa, double white_level,
  const Settings & settings, State & state)
{
  const int f = std::max(1, settings.downscale);
  const int block = 2 * f;
  const int out_w = width / block;
  const int out_h = height / block;
  if (data == nullptr || out_w <= 0 || out_h <= 0) {
    return cv::Mat();
  }

  // (row, col) of R, B and the two G sites inside a 2x2 cell
  int ry = 0, rx = 0, by = 1, bx = 1;
  switch (cfa) {
    case Cfa::kRGGB: ry = 0; rx = 0; by = 1; bx = 1; break;
    case Cfa::kBGGR: ry = 1; rx = 1; by = 0; bx = 0; break;
    case Cfa::kGRBG: ry = 0; rx = 1; by = 1; bx = 0; break;
    case Cfa::kGBRG: ry = 1; rx = 0; by = 0; bx = 1; break;
  }
  const int g1y = ry, g1x = 1 - rx;
  const int g2y = 1 - ry, g2x = rx;

  const std::size_t n = static_cast<std::size_t>(out_w) * static_cast<std::size_t>(out_h);
  std::vector<float> r(n), g(n), b(n);
  const float norm = 1.0f / static_cast<float>(f * f);
  for (int oy = 0; oy < out_h; ++oy) {
    for (int ox = 0; ox < out_w; ++ox) {
      double sr = 0.0, sg = 0.0, sb = 0.0;
      for (int cy = 0; cy < f; ++cy) {
        const std::size_t y0 = static_cast<std::size_t>(oy * block + 2 * cy);
        const T * row0 = data + y0 * stride;
        const T * row1 = row0 + stride;
        const T * rows[2] = {row0, row1};
        for (int cx = 0; cx < f; ++cx) {
          const std::size_t x0 = static_cast<std::size_t>(ox * block + 2 * cx);
          sr += static_cast<double>(rows[ry][x0 + rx]);
          sb += static_cast<double>(rows[by][x0 + bx]);
          sg += static_cast<double>(rows[g1y][x0 + g1x]) + static_cast<double>(rows[g2y][x0 + g2x]);
        }
      }
      const std::size_t i = static_cast<std::size_t>(oy) * out_w + ox;
      r[i] = static_cast<float>(sr) * norm;
      g[i] = static_cast<float>(sg) * norm * 0.5f;
      b[i] = static_cast<float>(sb) * norm;
    }
  }

  // Dark end of G over a subsample: the 0.5th percentile is the pedestal when the scene is dark (or
  // the darkest part of the scene), and its distance to the 5th percentile bounds the noise there.
  std::vector<float> sample;
  {
    const std::size_t step = std::max<std::size_t>(1, n / 20000);
    sample.reserve(n / step + 1);
    for (std::size_t i = 0; i < n; i += step) {
      sample.push_back(g[i]);
    }
  }
  const std::size_t k05 = sample.size() / 200;
  const std::size_t k5 = sample.size() / 20;
  std::nth_element(sample.begin(), sample.begin() + k05, sample.end());
  const double p05 = sample[k05];
  std::nth_element(sample.begin() + k05, sample.begin() + k5, sample.end());
  const double p5 = sample[k5];
  // Gaussian: p5 - p0.5 = (2.576 - 1.645) sigma (of the plane as displayed, after averaging).
  const double dark_sigma = std::max((p5 - p05) / 0.931, 1e-6);

  double black = settings.black_level_dn;
  if (black < 0.0) {
    black = p05;
  }
  const double range = std::max(1.0, white_level - black);

  double mr = 0.0, mg = 0.0, mb = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    mr += std::max(0.0, r[i] - black);
    mg += std::max(0.0, g[i] - black);
    mb += std::max(0.0, b[i] - black);
  }
  mr /= static_cast<double>(n);
  mg /= static_cast<double>(n);
  mb /= static_cast<double>(n);

  double wr = 1.0, wb = 1.0;
  if (settings.gray_world && mr > 1e-3 && mb > 1e-3 && mg > 1e-3) {
    wr = std::clamp(mg / mr, 0.25, 4.0);
    wb = std::clamp(mg / mb, 0.25, 4.0);
  }

  double gain = settings.gain;
  if (gain <= 0.0) {
    const double target = std::max(1e-3, settings.target_mean) / std::max(mg / range, 1e-7);
    double ceiling = std::max(0.05, settings.max_gain);
    if (settings.noise_ceiling > 0.0) {
      ceiling = std::min(ceiling, std::max(0.05, settings.noise_ceiling * range / dark_sigma));
    }
    const double log_target = std::log(std::clamp(target, 0.05, ceiling));
    state.log_gain = state.valid ? 0.6 * state.log_gain + 0.4 * log_target : log_target;
    state.valid = true;
    gain = std::exp(state.log_gain);
  }
  state.last_black = black;
  state.last_gain = gain;

  constexpr int kLutSize = 4096;
  std::array<std::uint8_t, kLutSize> lut{};
  const double inv_gamma = 1.0 / std::max(0.1, settings.gamma);
  for (int i = 0; i < kLutSize; ++i) {
    const double v = static_cast<double>(i) / (kLutSize - 1);
    lut[i] = static_cast<std::uint8_t>(std::lround(255.0 * std::pow(v, inv_gamma)));
  }
  const double scale = gain / range * (kLutSize - 1);
  const auto map = [&](double value, double wbal) {
      const double v = (value - black) * wbal * scale;
      const int index = v <= 0.0 ? 0 : (v >= kLutSize - 1 ? kLutSize - 1 : static_cast<int>(v));
      return lut[index];
    };

  cv::Mat bgr(out_h, out_w, CV_8UC3);
  for (int oy = 0; oy < out_h; ++oy) {
    std::uint8_t * out = bgr.ptr<std::uint8_t>(oy);
    for (int ox = 0; ox < out_w; ++ox) {
      const std::size_t i = static_cast<std::size_t>(oy) * out_w + ox;
      out[3 * ox + 0] = map(b[i], wb);
      out[3 * ox + 1] = map(g[i], 1.0);
      out[3 * ox + 2] = map(r[i], wr);
    }
  }
  return bgr;
}

}  // namespace raw_preview
