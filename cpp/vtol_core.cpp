// vtol_core: the hot loops of the LiDAR processing in C++ (pybind11, OpenMP). Built by cpp/build.sh into
// scripts/vtol_core*.so; scripts/fastcore.py uses it when present and falls back to NumPy otherwise.
//
//   world_points(t_lines, ranges, traj, d_b, t_bl, pulse_dt) -> (xyz float32 (N, 3), t float64 (N,))
//       every finite range of `ranges` (lines x rays) as a world point, p_W = T_WB(t) (t_BL + r d_B), ray k of
//       line i at t_lines[i] + k * pulse_dt with the pose interpolated in `traj` (t, x, y, z, qx, qy, qz, qw:
//       linear position, normalised-linear quaternion, as flight_report.interp_poses); pulse_dt = 0: one pose
//       per line. Hits in row-major order (the order of ranges[np.isfinite(ranges)]).
//   cell_reduce(index, values, n_cells, op) -> float64 (n_cells,)
//       per cell the max ("max"), min ("min") or count ("count") of the values whose index is in [0, n_cells);
//       empty cells -inf / +inf / 0 (np.maximum.at / np.minimum.at / np.bincount, ~10x faster).
//   VoxelMap(voxel, column): add(xyz), voxels() -> (keys int64, hits int64), columns() -> (keys, zmin, zmax),
//       points; keys as scripts/lidar_map.LidarMap._key. Replaces the np.unique merges of LidarMap.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace py = pybind11;
using f64 = py::array_t<double, py::array::c_style | py::array::forcecast>;
using f32 = py::array_t<float, py::array::c_style | py::array::forcecast>;
using i64 = py::array_t<int64_t, py::array::c_style | py::array::forcecast>;

namespace {

struct Pose {
    double p[3];
    double r[9];  // row-major R_WB
};

Pose interp(const double *traj, ssize_t n, double t) {
    // i = clip(searchsorted(traj_t, t), 1, n - 1) as in NumPy (side "left").
    ssize_t lo = 0, hi = n;
    while (lo < hi) {
        ssize_t mid = (lo + hi) / 2;
        if (traj[mid * 8] < t) lo = mid + 1; else hi = mid;
    }
    ssize_t i = std::min(std::max<ssize_t>(lo, 1), n - 1);
    const double *a = traj + (i - 1) * 8, *b = traj + i * 8;
    double w = (t - a[0]) / std::max(b[0] - a[0], 1e-9);
    Pose o;
    for (int k = 0; k < 3; ++k) o.p[k] = (1 - w) * a[1 + k] + w * b[1 + k];
    double q0[4] = {a[4], a[5], a[6], a[7]}, q1[4] = {b[4], b[5], b[6], b[7]};
    double dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3];
    double s = dot < 0 ? -1.0 : 1.0, q[4], nn = 0;
    for (int k = 0; k < 4; ++k) { q[k] = (1 - w) * q0[k] + w * s * q1[k]; nn += q[k] * q[k]; }
    nn = std::sqrt(nn);
    double x = q[0] / nn, y = q[1] / nn, z = q[2] / nn, ww = q[3] / nn;
    double *r = o.r;
    r[0] = 1 - 2 * (y * y + z * z); r[1] = 2 * (x * y - ww * z); r[2] = 2 * (x * z + ww * y);
    r[3] = 2 * (x * y + ww * z); r[4] = 1 - 2 * (x * x + z * z); r[5] = 2 * (y * z - ww * x);
    r[6] = 2 * (x * z - ww * y); r[7] = 2 * (y * z + ww * x); r[8] = 1 - 2 * (x * x + y * y);
    return o;
}

py::tuple world_points(f64 t_lines, f32 ranges, f64 traj, f64 d_b, f64 t_bl, double pulse_dt) {
    auto R = ranges.unchecked<2>();
    const ssize_t n = R.shape(0), m = R.shape(1);
    if (t_lines.size() != n || d_b.shape(0) != m || d_b.shape(1) != 3 || traj.shape(1) != 8 || t_bl.size() != 3)
        throw std::invalid_argument("world_points: shapes do not match");
    const double *T = t_lines.data(), *TR = traj.data(), *D = d_b.data(), *TB = t_bl.data();
    const ssize_t nt = traj.shape(0);
    const float *rp = ranges.data();
    std::vector<ssize_t> start(n + 1, 0);
    for (ssize_t i = 0; i < n; ++i) {
        ssize_t c = 0;
        for (ssize_t k = 0; k < m; ++k) c += std::isfinite(rp[i * m + k]);
        start[i + 1] = start[i] + c;
    }
    const ssize_t N = start[n];
    py::array_t<float> xyz({N, (ssize_t)3});
    py::array_t<double> tt(N);
    float *X = xyz.mutable_data();
    double *TT = tt.mutable_data();
    {
        py::gil_scoped_release nogil;
#pragma omp parallel for schedule(dynamic, 64)
        for (ssize_t i = 0; i < n; ++i) {
            ssize_t o = start[i];
            Pose pl = interp(TR, nt, T[i]);
            for (ssize_t k = 0; k < m; ++k) {
                float rr = rp[i * m + k];
                if (!std::isfinite(rr)) continue;
                double t = T[i] + k * pulse_dt;
                Pose ps = pulse_dt != 0.0 ? interp(TR, nt, t) : pl;
                const double *d = D + k * 3;
                double v[3] = {TB[0] + rr * d[0], TB[1] + rr * d[1], TB[2] + rr * d[2]};
                for (int a = 0; a < 3; ++a)
                    X[o * 3 + a] = (float)(ps.p[a] + ps.r[a * 3] * v[0] + ps.r[a * 3 + 1] * v[1] + ps.r[a * 3 + 2] * v[2]);
                TT[o] = t;
                ++o;
            }
        }
    }
    return py::make_tuple(xyz, tt);
}

py::array_t<double> cell_reduce(i64 index, f64 values, ssize_t n_cells, const std::string &op) {
    const int64_t *I = index.data();
    const double *V = values.data();
    const ssize_t n = index.size();
    if (values.size() != n && op != "count") throw std::invalid_argument("cell_reduce: sizes differ");
    double init = op == "max" ? -std::numeric_limits<double>::infinity()
                : op == "min" ? std::numeric_limits<double>::infinity() : 0.0;
    if (op != "max" && op != "min" && op != "count") throw std::invalid_argument("cell_reduce: op max|min|count");
    py::array_t<double> out(n_cells);
    double *O = out.mutable_data();
    std::fill(O, O + n_cells, init);
    py::gil_scoped_release nogil;
    if (op == "count") {
        for (ssize_t j = 0; j < n; ++j) if (I[j] >= 0 && I[j] < n_cells) O[I[j]] += 1;
    } else if (op == "max") {
        for (ssize_t j = 0; j < n; ++j) if (I[j] >= 0 && I[j] < n_cells && V[j] > O[I[j]]) O[I[j]] = V[j];
    } else {
        for (ssize_t j = 0; j < n; ++j) if (I[j] >= 0 && I[j] < n_cells && V[j] < O[I[j]]) O[I[j]] = V[j];
    }
    return out;
}

constexpr int64_t OFF = 1 << 20, SH = 1 << 21;
inline int64_t key2(int64_t i, int64_t j) { return (i + OFF) * SH + (j + OFF); }
inline int64_t key3(int64_t i, int64_t j, int64_t k) { return key2(i, j) * SH + (k + OFF); }

class VoxelMap {
  public:
    VoxelMap(double voxel, double column) : v_(voxel), c_(column) {}

    void add(f64 xyz) {
        auto P = xyz.unchecked<2>();
        py::gil_scoped_release nogil;
        for (ssize_t n = 0; n < P.shape(0); ++n) {
            double x = P(n, 0), y = P(n, 1), z = P(n, 2);
            ++vox_[key3((int64_t)std::floor(x / v_), (int64_t)std::floor(y / v_), (int64_t)std::floor(z / v_))];
            auto it = col_.find(key2((int64_t)std::floor(x / c_), (int64_t)std::floor(y / c_)));
            if (it == col_.end()) col_.emplace(key2((int64_t)std::floor(x / c_), (int64_t)std::floor(y / c_)),
                                               std::make_pair(z, z));
            else { it->second.first = std::min(it->second.first, z); it->second.second = std::max(it->second.second, z); }
        }
        points_ += P.shape(0);
    }

    py::tuple voxels() const {
        py::array_t<int64_t> k(vox_.size());
        py::array_t<int64_t> h(vox_.size());
        int64_t *K = k.mutable_data(), *H = h.mutable_data();
        size_t a = 0;
        for (auto &e : vox_) { K[a] = e.first; H[a] = e.second; ++a; }
        return py::make_tuple(k, h);
    }

    py::tuple columns() const {
        py::array_t<int64_t> k(col_.size());
        py::array_t<double> lo(col_.size()), hi(col_.size());
        int64_t *K = k.mutable_data();
        double *L = lo.mutable_data(), *U = hi.mutable_data();
        size_t a = 0;
        for (auto &e : col_) { K[a] = e.first; L[a] = e.second.first; U[a] = e.second.second; ++a; }
        return py::make_tuple(k, lo, hi);
    }

    int64_t points() const { return points_; }
    size_t n_voxels() const { return vox_.size(); }
    size_t n_columns() const { return col_.size(); }

  private:
    double v_, c_;
    int64_t points_ = 0;
    std::unordered_map<int64_t, int64_t> vox_;
    std::unordered_map<int64_t, std::pair<double, double>> col_;
};

}  // namespace

PYBIND11_MODULE(vtol_core, m) {
    m.doc() = "VTOL-mapping LiDAR hot loops (see cpp/vtol_core.cpp)";
    m.def("world_points", &world_points, py::arg("t_lines"), py::arg("ranges"), py::arg("traj"), py::arg("d_b"),
          py::arg("t_bl"), py::arg("pulse_dt"));
    m.def("cell_reduce", &cell_reduce, py::arg("index"), py::arg("values"), py::arg("n_cells"), py::arg("op"));
    py::class_<VoxelMap>(m, "VoxelMap")
        .def(py::init<double, double>(), py::arg("voxel"), py::arg("column"))
        .def("add", &VoxelMap::add)
        .def("voxels", &VoxelMap::voxels)
        .def("columns", &VoxelMap::columns)
        .def_property_readonly("points", &VoxelMap::points)
        .def_property_readonly("n_voxels", &VoxelMap::n_voxels)
        .def_property_readonly("n_columns", &VoxelMap::n_columns);
}
