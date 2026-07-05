// small_gicp_align — offline one-shot GICP registration shim.
//
// Reads two raw point files (target = ideal model in map frame, source = LIO
// cloud in lidar frame), an initial transform, and a few knobs, runs
// small_gicp::align, and prints the refined transform + quality stats to
// stdout. The Python driver (auto_align_gicp.py) does all the PCD I/O with the
// pure-Python pcd_io.py, so this shim links ONLY small_gicp — no PCL — keeping
// with the tools/auto no-PCL policy.
//
// Raw point file format: little-endian, N * 3 * float64 (x,y,z, x,y,z, ...).
// The byte size divided by 24 gives N; no header.
//
// Usage:
//   small_gicp_align <target.bin> <source.bin> \
//       --init t00 t01 ... t33            (16 numbers, row-major 4x4) \
//       [--downsample 0.25] [--max-dist 1.0] [--max-iter 20] [--threads 4]
//
// stdout (one "key value" per line, easy to parse):
//   T 16 row-major numbers of T_target_source
//   converged 0|1
//   iterations N
//   num_inliers N
//   error F
//   source_points N   (downsampled source point count)

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <small_gicp/registration/registration_helper.hpp>

namespace {

std::vector<Eigen::Vector3d> load_raw_xyz(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) {
    std::fprintf(stderr, "small_gicp_align: cannot open %s\n", path.c_str());
    std::exit(2);
  }
  const std::streamsize bytes = f.tellg();
  if (bytes % (3 * sizeof(double)) != 0) {
    std::fprintf(stderr,
                 "small_gicp_align: %s size %lld not a multiple of 24 bytes\n",
                 path.c_str(), static_cast<long long>(bytes));
    std::exit(2);
  }
  const size_t n = static_cast<size_t>(bytes) / (3 * sizeof(double));
  f.seekg(0);
  std::vector<double> buf(n * 3);
  f.read(reinterpret_cast<char*>(buf.data()), bytes);
  std::vector<Eigen::Vector3d> pts(n);
  for (size_t i = 0; i < n; ++i) {
    pts[i] = Eigen::Vector3d(buf[3 * i], buf[3 * i + 1], buf[3 * i + 2]);
  }
  return pts;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr,
                 "usage: %s target.bin source.bin --init t00..t33 "
                 "[--downsample r] [--max-dist d] [--max-iter n] [--threads k]\n",
                 argv[0]);
    return 2;
  }

  const std::string target_path = argv[1];
  const std::string source_path = argv[2];

  Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
  double downsample = 0.25;
  double max_dist = 1.0;
  int max_iter = 20;
  int threads = 4;

  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--init") {
      for (int k = 0; k < 16; ++k) {
        if (++i >= argc) {
          std::fprintf(stderr, "small_gicp_align: --init needs 16 numbers\n");
          return 2;
        }
        init(k / 4, k % 4) = std::atof(argv[i]);
      }
    } else if (a == "--downsample") {
      downsample = std::atof(argv[++i]);
    } else if (a == "--max-dist") {
      max_dist = std::atof(argv[++i]);
    } else if (a == "--max-iter") {
      max_iter = std::atoi(argv[++i]);
    } else if (a == "--threads") {
      threads = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr, "small_gicp_align: unknown arg %s\n", a.c_str());
      return 2;
    }
  }

  const std::vector<Eigen::Vector3d> target = load_raw_xyz(target_path);
  const std::vector<Eigen::Vector3d> source = load_raw_xyz(source_path);
  if (target.empty() || source.empty()) {
    std::fprintf(stderr, "small_gicp_align: empty cloud (target=%zu source=%zu)\n",
                 target.size(), source.size());
    return 2;
  }

  Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
  init_T.matrix() = init;

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.downsampling_resolution = downsample;
  setting.max_correspondence_distance = max_dist;
  setting.max_iterations = max_iter;
  setting.num_threads = threads;

  const small_gicp::RegistrationResult result =
      small_gicp::align(target, source, init_T, setting);

  // Report the downsampled source size so the driver can compute an inlier
  // ratio consistent with what GICP actually matched against.
  const auto down = small_gicp::preprocess_points(source, downsample, 10, threads);
  const size_t source_points = down.first ? down.first->size() : source.size();

  const Eigen::Matrix4d T = result.T_target_source.matrix();
  std::printf("T");
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      std::printf(" %.10f", T(r, c));
    }
  }
  std::printf("\n");
  std::printf("converged %d\n", result.converged ? 1 : 0);
  std::printf("iterations %zu\n", result.iterations);
  std::printf("num_inliers %zu\n", result.num_inliers);
  std::printf("error %.10f\n", result.error);
  std::printf("source_points %zu\n", source_points);
  return 0;
}
