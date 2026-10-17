#include <aims_pp/runtime.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <openssl/sha.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace aims_pp {
namespace {
constexpr double pi = 3.14159265358979323846;
using V = std::array<double, 2>;
double wrap(double x, double length) {
  double r = std::fmod(x, length);
  return r < 0 ? r + length : r;
}
double angle(double x) { return wrap(x + pi, 2 * pi) - pi; }
void need(bool ok, const std::string &reason) {
  if (!ok)
    throw std::invalid_argument(reason);
}
void finite(double v) { need(std::isfinite(v), "finite inputs required"); }
double number(const json &v) {
  need(v.is_number() && !v.is_boolean(), "numeric input required");
  double x = v.get<double>();
  finite(x);
  return x;
}
int64_t integer_string(const json &v) {
  auto text = v.get<std::string>();
  size_t used = 0;
  auto result = std::stoll(text, &used);
  for (size_t i = used; i < text.size(); i++)
    need(std::isspace(static_cast<unsigned char>(text[i])), "invalid integer");
  return result;
}
double float_string(const json &v) {
  auto text = v.get<std::string>();
  size_t used = 0;
  auto result = std::stod(text, &used);
  for (size_t i = used; i < text.size(); i++)
    need(std::isspace(static_cast<unsigned char>(text[i])), "invalid float");
  finite(result);
  return result;
}
json defaults() {
  return {{"wheelbase", .36},
          {"steer_limit", .45},
          {"rear_offset", 0.},
          {"half_length", nullptr},
          {"front_extent", .52},
          {"rear_extent", .1},
          {"half_width", .16},
          {"max_speed", 4.},
          {"geometry_verified", true},
          {"enforce_corridor", false},
          {"lookahead_gain", 1.},
          {"min_lookahead", .3},
          {"max_lookahead", 4.5},
          {"lateral_error_gain", 1.},
          {"heading_error_gain", .1},
          {"curvature_ff_gain", .1},
          {"steering_limit_start_speed", 4.},
          {"steering_limit_full_speed", 6.},
          {"high_speed_max_steering", .18}};
}
json checked_config(const json &input) {
  json c = defaults();
  need(input.is_object(), "PP config mapping required");
  for (auto it = input.begin(); it != input.end(); ++it) {
    need(c.contains(it.key()), "unknown PP config field");
    c[it.key()] = it.value();
  }
  for (auto it = c.begin(); it != c.end(); ++it) {
    if (it.key() == "geometry_verified")
      need(it.value().is_boolean() && it.value() == true,
           "verified vehicle geometry required");
    else if (it.key() == "enforce_corridor")
      need(it.value().is_boolean(), "boolean enforce_corridor required");
    else if (it.key() == "half_length" && it.value().is_null())
      continue;
    else
      need(number(it.value()) >= 0, "finite nonnegative config required");
  }
  for (auto k : {"wheelbase", "steer_limit", "half_width", "front_extent",
                 "max_speed", "min_lookahead"})
    need(number(c[k]) > 0, "positive vehicle and controller limits required");
  need(number(c["steer_limit"]) < pi / 2 &&
           number(c["max_lookahead"]) >= number(c["min_lookahead"]),
       "invalid steering or lookahead bounds");
  need(number(c["steering_limit_full_speed"]) >=
           number(c["steering_limit_start_speed"]),
       "invalid speed steering transition");
  return c;
}
json yaml_mapping(const std::string &file, bool ignore_unknown = false) {
  auto y = YAML::LoadFile(file);
  need(y.IsMap(), "configuration mapping required");
  json j = json::object();
  auto d = defaults();
  for (auto it : y) {
    auto k = it.first.as<std::string>();
    static const std::set<std::string> vehicle_keys = {
        "wheelbase",         "steer_limit",     "rear_offset", "half_length",
        "front_extent",      "rear_extent",     "half_width",  "max_speed",
        "geometry_verified", "enforce_corridor"};
    if (ignore_unknown && !vehicle_keys.count(k))
      continue;
    if (!d.contains(k)) {
      need(ignore_unknown, "unknown configuration field");
      continue;
    }
    auto v = it.second;
    if (v.IsNull())
      j[k] = nullptr;
    else if (k == "geometry_verified" || k == "enforce_corridor")
      j[k] = v.as<bool>();
    else
      j[k] = v.as<double>();
  }
  return j;
}
std::array<double, 4> pose_values(const json &p) {
  std::array<double, 4> a;
  if (p.is_array()) {
    need(p.size() == 4, "planar pose size required");
    for (int i = 0; i < 4; i++)
      a[i] = number(p[i]);
  } else {
    for (int i = 0; i < 4; i++)
      a[i] = number(
          p.at(std::array<const char *, 4>{"x", "y", "yaw", "velocity"}[i]));
  }
  return a;
}
double dot(V a, V b) { return a[0] * b[0] + a[1] * b[1]; }
struct Reference {
  json data, metadata;
  std::string hash, checksum;
  double length, left, right;
  std::vector<double> knots, positions, speeds, samples;
  std::vector<V> sample_xy;
  std::array<std::vector<V>, 6> coeff;
  Reference(const std::string &directory, const json &c) {
    auto file = std::filesystem::path(directory) / "reference.json";
    need(std::filesystem::file_size(file) <= 64 * 1024 * 1024,
         "reference exceeds 64 MiB");
    std::ifstream in(file);
    std::string contents((std::istreambuf_iterator<char>(in)), {});
    need(in.good() || in.eof(), "cannot read reference");
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(contents.data()),
           contents.size(), digest);
    std::ostringstream sum;
    sum << std::hex << std::setfill('0');
    for (auto b : digest)
      sum << std::setw(2) << int(b);
    checksum = sum.str();
    data = json::parse(contents);
    need(data.value("schema_version", 1) == 1, "unsupported reference schema");
    length = number(data.at("length"));
    left = number(data.at("left_width"));
    right = number(data.at("right_width"));
    metadata = data.at("metadata");
    need(data.at("frame_id") == "map" && metadata.is_object() &&
             metadata.at("closed_lap") == true,
         "closed forward map reference required");
    need(metadata.at("source_frame") == "base_link" &&
             metadata.value("coordinate_reference", std::string("rear_axle")) ==
                 "rear_axle",
         "base_link source and rear_axle coordinate reference required");
    hash = metadata.at("map_sha256").get<std::string>();
    need(hash.size() == 64 &&
             hash.find_first_not_of("0123456789abcdef") == std::string::npos,
         "map SHA-256 required");
    need(length > 0 && left > 0 && right > 0,
         "positive finite length and widths required");
    for (auto x : data.at("knots"))
      knots.push_back(number(x));
    need(knots.size() >= 3 && knots.size() <= 20001 && knots[0] == 0 &&
             std::abs(knots.back() - length) <= 1e-9,
         "strict increasing knots spanning the length required");
    for (size_t i = 1; i < knots.size(); i++)
      need(knots[i] > knots[i - 1], "strict increasing knots required");
    auto co = data.at("coefficients");
    need(co.is_array() && co.size() == 6, "degree-5 coefficients required");
    for (int k = 0; k < 6; k++) {
      need(co[k].size() == knots.size() - 1, "coefficient interval count");
      for (auto xy : co[k]) {
        need(xy.is_array() && xy.size() == 2, "planar coefficients required");
        coeff[k].push_back({number(xy[0]), number(xy[1])});
      }
    }
    auto speed = data.at("speed_profile");
    need(speed.at("schema_version") == 1, "invalid frozen speed profile");
    for (auto x : speed.at("positions"))
      positions.push_back(number(x));
    for (auto x : speed.at("speeds"))
      speeds.push_back(number(x));
    need(!positions.empty() && positions.size() == speeds.size() &&
             positions[0] == 0 && positions.back() < length,
         "invalid frozen speed profile");
    for (size_t i = 0; i < speeds.size(); i++)
      need(speeds[i] >= 0 && (i == 0 || positions[i] > positions[i - 1]),
           "invalid frozen speed profile");
    for (size_t i = 0; i < knots.size() - 1; i++)
      for (int d = 0; d < 3; d++) {
        auto a = eval(i, knots[i + 1] - knots[i], d),
             b = eval((i + 1) % (knots.size() - 1), 0, d);
        need(std::abs(a[0] - b[0]) <= 1e-5 && std::abs(a[1] - b[1]) <= 1e-5,
             "quintic reference has a discontinuous seam");
      }
    auto geom = metadata.at("vehicle_geometry");
    need(geom.is_object(), "complete vehicle geometry required");
    for (auto k : {"wheelbase", "rear_offset", "half_length", "front_extent",
                   "rear_extent", "half_width"}) {
      need(geom.contains(k), std::string("vehicle geometry mismatch: ") + k);
      if (geom[k].is_null() && c[k].is_null())
        continue;
      need(!c[k].is_null() && std::abs(number(geom[k]) - number(c[k])) <= 1e-9,
           std::string("vehicle geometry mismatch: ") + k);
    }
    need(std::min(left, right) > number(c["half_width"]) &&
             *std::max_element(speeds.begin(), speeds.end()) <=
                 number(c["max_speed"]),
         "reference width or speed incompatible with vehicle");
    need(length / .05 <= 20000, "reference exceeds bounded projection length");
    int count = std::max(64, int(std::ceil(length / .05)));
    need(count <= 20000, "reference exceeds bounded projection length");
    for (int i = 0; i < count; i++) {
      double s = length * i / count;
      samples.push_back(s);
      auto d = derivatives(s);
      need(dot(d[1], d[1]) >= 1e-12, "reference contains zero tangent");
      sample_xy.push_back(d[0]);
    }
  }
  V eval(size_t i, double h, int derivative) const {
    V r{0, 0};
    for (int k = 0; k < 6 - derivative; k++) {
      double factor = 1;
      for (int f = 5 - k - derivative + 1; f <= 5 - k; f++)
        factor *= f;
      for (int axis = 0; axis < 2; axis++)
        r[axis] = r[axis] * h + factor * coeff[k][i][axis];
    }
    return r;
  }
  std::array<V, 3> derivatives(double s) const {
    finite(s);
    double sw = wrap(s, length);
    size_t i =
        std::min(size_t(std::upper_bound(knots.begin(), knots.end(), sw) -
                        knots.begin() - 1),
                 knots.size() - 2);
    return {eval(i, sw - knots[i], 0), eval(i, sw - knots[i], 1),
            eval(i, sw - knots[i], 2)};
  }
  double speed(double s) const {
    finite(s);
    s = wrap(s, length);
    auto i = size_t(std::upper_bound(positions.begin(), positions.end(), s) -
                    positions.begin() - 1);
    double end = i + 1 < positions.size() ? positions[i + 1] : length;
    double next = i + 1 < speeds.size() ? speeds[i + 1] : speeds[0];
    return speeds[i] +
           (s - positions[i]) / (end - positions[i]) * (next - speeds[i]);
  }
  json at(double s) const {
    auto d = derivatives(s);
    double norm = dot(d[1], d[1]);
    need(norm >= 1e-12, "zero reference tangent");
    return {{"x", d[0][0]},
            {"y", d[0][1]},
            {"yaw", std::atan2(d[1][1], d[1][0])},
            {"curvature",
             (d[1][0] * d[2][1] - d[1][1] * d[2][0]) / std::pow(norm, 1.5)},
            {"speed", speed(s)}};
  }
  json project(V xy) const {
    finite(xy[0]);
    finite(xy[1]);
    size_t index = 0;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < samples.size(); i++) {
      V e{sample_xy[i][0] - xy[0], sample_xy[i][1] - xy[1]};
      double err = dot(e, e);
      if (err < best) {
        best = err;
        index = i;
      }
    }
    best = std::numeric_limits<double>::infinity();
    double out = 0, lateral = 0;
    for (int offset : {-1, 0, 1}) {
      double s = samples[(index + samples.size() + offset) % samples.size()];
      for (int k = 0; k < 8; k++) {
        auto d = derivatives(s);
        V e{d[0][0] - xy[0], d[0][1] - xy[1]};
        double denominator = dot(d[1], d[1]) + dot(e, d[2]);
        if (std::abs(denominator) < 1e-10)
          break;
        s = wrap(s - std::clamp(dot(e, d[1]) / denominator, -.1, .1), length);
      }
      auto d = derivatives(s);
      V e{xy[0] - d[0][0], xy[1] - d[0][1]};
      double err = dot(e, e);
      if (err < best) {
        best = err;
        out = s;
        lateral =
            (-e[0] * d[1][1] + e[1] * d[1][0]) / std::sqrt(dot(d[1], d[1]));
      }
    }
    return {{"progress", out}, {"lateral_error", lateral}};
  }
};
struct Health {
  std::string epoch, reason = "Localization health unavailable";
  std::set<std::string> retired;
  int64_t sequence = -1, health_sequence = -1, loss_sequence = -1, stamp = -1;
  double anchor_received = NAN, received = NAN;
  bool ready = false, qualified = false, committed = false;
  std::array<double, 3> alignment{0, 0, 0};
  std::vector<double> transform;
  bool observe(const json &v, int64_t now, double steady) {
    bool changed = false;
    try {
      finite(steady);
      need(v.at("protocol_version") == "1", "protocol");
      std::string e = v.at("epoch");
      int64_t hs = integer_string(v.at("health_sequence")),
              seq = integer_string(v.at("anchor_sequence")),
              st = integer_string(v.at("last_anchor_stamp_ns"));
      std::string r = v.at("ready");
      need(!e.empty() && std::min({hs, seq, st}) >= 0 &&
               (r == "true" || r == "false"),
           "health identity");
      if (retired.count(e))
        return false;
      if (!epoch.empty() && e != epoch) {
        need(retired.size() < 64, "epoch history exhausted; restart required");
        retired.insert(epoch);
        changed = true;
      }
      if (e != epoch) {
        epoch = e;
        sequence = health_sequence = loss_sequence = stamp = -1;
        anchor_received = NAN;
        ready = qualified = committed = false;
      }
      if (hs <= health_sequence)
        return changed;
      health_sequence = hs;
      if (seq < sequence)
        return changed;
      need(!(seq == sequence && stamp >= 0 && st != stamp),
           "anchor stamp changed without sequence");
      std::vector<double> tr;
      double norm = 1;
      if (v.value("alignment_valid", std::string()) == "true") {
        need(v.at("alignment_epoch") == e &&
                 integer_string(v.at("alignment_anchor_sequence")) == seq &&
                 integer_string(v.at("alignment_stamp_ns")) == st,
             "alignment identity mismatch");
        for (auto k : {"x", "y", "z", "qx", "qy", "qz", "qw"}) {
          double f = float_string(v.at(std::string("map_odom_") + k));
          finite(f);
          tr.push_back(f);
        }
        norm = std::sqrt(tr[3] * tr[3] + tr[4] * tr[4] + tr[5] * tr[5] +
                         tr[6] * tr[6]);
        need(std::abs(norm - 1) <= .01, "alignment quaternion");
        need(!(seq == sequence && committed && tr != transform),
             "alignment changed without sequence");
      }
      if (seq > sequence)
        committed = false;
      qualified = !tr.empty();
      if (qualified) {
        committed = true;
        transform = tr;
        double qx = tr[3] / norm, qy = tr[4] / norm, qz = tr[5] / norm,
               qw = tr[6] / norm;
        alignment = {
            tr[0], tr[1],
            std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))};
      }
      if (seq > sequence)
        anchor_received = steady - std::max(0., (now - st) * 1e-9);
      sequence = seq;
      stamp = st;
      received = steady;
      if (r == "false")
        loss_sequence = std::max(loss_sequence, seq);
      ready = r == "true" && seq > loss_sequence;
      reason = v.value("state", std::string("Localization not ready"));
    } catch (const std::exception &) {
      ready = qualified = false;
      reason = "Malformed localization health";
    }
    return changed;
  }
  bool usable(int64_t now, double steady) const {
    double ra = (now - stamp) * 1e-9, aa = steady - anchor_received,
           ha = steady - received;
    return ready && stamp >= 0 && std::isfinite(ra) && std::isfinite(aa) &&
           std::isfinite(ha) && ra >= 0 && ra <= 1 && aa >= 0 && aa <= 1 &&
           ha >= 0 && ha <= .3;
  }
};
} // namespace
json load_config(const std::string &vehicle, const std::string &tuning) {
  auto c = yaml_mapping(vehicle, true);
  std::set<std::string> geom = {
      "wheelbase",         "steer_limit",     "rear_offset", "half_length",
      "front_extent",      "rear_extent",     "half_width",  "max_speed",
      "geometry_verified", "enforce_corridor"};
  for (auto &k : geom)
    if (k != "enforce_corridor")
      need(c.contains(k), "incomplete vehicle configuration");
  for (auto it = c.begin(); it != c.end();) {
    if (!geom.count(it.key()))
      it = c.erase(it);
    else
      ++it;
  }
  if (!tuning.empty()) {
    auto t = yaml_mapping(tuning);
    for (auto it = t.begin(); it != t.end(); ++it) {
      need(!geom.count(it.key()),
           "PP tuning cannot override vehicle geometry or unknown fields");
      c[it.key()] = it.value();
    }
  }
  return checked_config(c);
}
struct Runtime::Impl {
  json config;
  Reference ref;
  bool reverse, overshoot;
  Health health;
  std::optional<int64_t> last_ros;
  std::string map_identity, pose_epoch, fault_reason;
  bool map_valid = false, authority = false, command_valid = false;
  double authority_received = NAN, command_source = NAN, command_received = NAN,
         source = NAN, received = NAN;
  int64_t command_stamp = -1, source_stamp = -1;
  std::optional<std::array<double, 4>> pose;
  uint64_t generation = 0;
  bool enabled = false, repeat = false;
  std::string phase = "DISABLED", reason = "Explicit enable required";
  double progress = 0, traveled = 0, last_tick = NAN, speed_limit = 1;
  uint64_t run_generation = 0;
  Impl(const std::string &dir, const json &c, bool r, bool o)
      : config(checked_config(c)), ref(dir, config), reverse(r), overshoot(o) {}
  double c(const char *k) const { return config.at(k).get<double>(); }
  void invalidate(const std::string &r) {
    generation++;
    fault_reason = r;
  }
  std::string localization(int64_t now, double steady,
                           bool anchor = true) const {
    if (!pose || !(steady - source >= 0 && steady - source <= .1) ||
        !(steady - received >= 0 && steady - received <= .1) ||
        !((now - source_stamp) * 1e-9 >= 0 &&
          (now - source_stamp) * 1e-9 <= .1))
      return "Fresh source odometry required";
    if (map_identity != ref.hash || !map_valid)
      return "Matching valid map required";
    if (!health.qualified || (anchor && !health.usable(now, steady)) ||
        pose_epoch != health.epoch)
      return "Fresh trusted qualified localization required";
    return "";
  }
  std::string gate(int64_t now, double steady, int publishers) const {
    if (publishers != 1)
      return "Exclusive /drive publisher required";
    if (!authority || !(steady - authority_received >= 0 &&
                        steady - authority_received <= .1))
      return "Fresh autonomy speed authority required";
    auto r = localization(now, steady);
    if (!r.empty())
      return r;
    if (!command_valid ||
        !(steady - command_source >= 0 && steady - command_source <= .1) ||
        !(steady - command_received >= 0 && steady - command_received <= .1) ||
        !((now - command_stamp) * 1e-9 >= 0 &&
          (now - command_stamp) * 1e-9 <= .1))
      return "Fresh final VESC speed command required";
    return "";
  }
  json map_pose() const {
    need(pose && health.qualified && pose_epoch == health.epoch,
         "qualified map state unavailable");
    auto p = *pose;
    double x = p[0] - c("rear_offset") * std::cos(p[2]),
           y = p[1] - c("rear_offset") * std::sin(p[2]),
           a = health.alignment[2];
    return json::array({health.alignment[0] + std::cos(a) * x - std::sin(a) * y,
                        health.alignment[1] + std::sin(a) * x + std::cos(a) * y,
                        angle(p[2] + a), p[3]});
  }
  double lookahead(double v) const {
    finite(v);
    return std::clamp(c("lookahead_gain") * std::abs(v) + c("min_lookahead"),
                      c("min_lookahead"), c("max_lookahead"));
  }
  void fault(const std::string &r) {
    enabled = false;
    phase = "FAULT";
    reason = r;
  }
};
Runtime::Runtime(const std::string &dir, const json &c, bool r, bool o)
    : p_(new Impl(dir, c, r, o)) {}
Runtime::~Runtime() = default;
Runtime::Runtime(Runtime &&) noexcept = default;
Runtime &Runtime::operator=(Runtime &&) noexcept = default;
void Runtime::invalidate(const std::string &r) { p_->invalidate(r); }
void Runtime::observe_clock(int64_t now) {
  auto &p = *p_;
  if (p.last_ros && now < *p.last_ros - 1000000) {
    std::string id = p.map_identity == p.ref.hash ? p.map_identity : "";
    p.health = Health{};
    p.map_identity = id;
    p.map_valid = p.authority = p.command_valid = false;
    p.authority_received = p.command_source = p.command_received = p.source =
        p.received = NAN;
    p.command_stamp = p.source_stamp = -1;
    p.pose.reset();
    p.pose_epoch = "";
    invalidate("ROS clock moved backwards; fresh inputs and explicit re-enable "
               "required");
  }
  p.last_ros = now;
}
void Runtime::observe_map_identity(const std::string &hash) {
  p_->map_identity = hash;
  if (hash != p_->ref.hash)
    invalidate("Map identity mismatch");
}
void Runtime::observe_map_valid(bool v) {
  p_->map_valid = v;
  if (!v)
    invalidate("Map validity withdrawn");
}
bool Runtime::observe_health(const json &v, int64_t now, double steady) {
  observe_clock(now);
  auto &p = *p_;
  if (!v.is_object() || !v.contains("map_sha256") ||
      !v["map_sha256"].is_string() || v["map_sha256"] != p.ref.hash ||
      !v.contains("map_valid") || v["map_valid"] != "true") {
    p.health.observe(json::object(), now, steady);
    invalidate("Localization alignment map identity or validity mismatch");
    return false;
  }
  bool changed = p.health.observe(v, now, steady);
  if (changed) {
    p.pose.reset();
    invalidate("Localization epoch changed; explicit re-enable required");
  }
  if (!p.health.qualified || !p.health.usable(now, steady))
    invalidate("Trusted localization unavailable");
  return changed;
}
void Runtime::observe_authority(bool enabled, double steady) {
  finite(steady);
  p_->authority = enabled;
  p_->authority_received = steady;
  if (!enabled)
    invalidate(
        "Autonomy speed authority withdrawn; explicit re-enable required");
}
bool Runtime::observe_odometry(const json &v, int64_t now, double steady) {
  observe_clock(now);
  auto &p = *p_;
  try {
    finite(steady);
    double x = number(v.at("x")), y = number(v.at("y")), yaw;
    if (v.contains("quaternion")) {
      auto q = v.at("quaternion");
      need(q.size() == 4, "odometry quaternion size");
      std::array<double, 4> a;
      double norm = 0;
      for (int i = 0; i < 4; i++) {
        a[i] = number(q[i]);
        norm += a[i] * a[i];
      }
      norm = std::sqrt(norm);
      need(std::abs(norm - 1) <= .01, "nonunit odometry quaternion");
      for (auto &f : a)
        f /= norm;
      yaw = std::atan2(2 * (a[3] * a[2] + a[0] * a[1]),
                       1 - 2 * (a[1] * a[1] + a[2] * a[2]));
    } else
      yaw = number(v.at("yaw"));
    double velocity = number(v.at("velocity"));
    int64_t stamp = v.at("stamp_ns");
    double age = (now - stamp) * 1e-9;
    need(stamp >= 0 && age >= 0 && age <= .1, "stale or future source stamp");
    need(v.value("frame_id", std::string("odom")) == "odom" &&
             v.value("child_frame_id", std::string("base_link")) ==
                 "base_link" &&
             !(velocity < -.05 && !p.reverse) &&
             !(velocity > p.c("max_speed") + .5 && !p.overshoot),
         "invalid odom/base_link forward state");
    if (stamp <= p.source_stamp)
      return false;
    if (p.pose) {
      auto old = *p.pose;
      double dt = (stamp - p.source_stamp) * 1e-9,
             bound =
                 p.overshoot
                     ? std::max({p.c("max_speed"), std::abs(velocity), old[3]})
                     : p.c("max_speed");
      need(std::hypot(x - old[0], y - old[1]) <= bound * dt + .30 &&
               std::abs(angle(yaw - old[2])) <= .5,
           "odometry discontinuity");
    }
    p.pose = std::array<double, 4>{x, y, yaw, std::max(0., velocity)};
    p.source = steady - age;
    p.received = steady;
    p.source_stamp = stamp;
    p.pose_epoch = p.health.epoch;
    return true;
  } catch (const std::exception &) {
    p.pose.reset();
    invalidate("Odometry invalid or discontinuous");
    throw;
  }
}
void Runtime::observe_command(const json &v, int64_t now, double steady) {
  observe_clock(now);
  auto &p = *p_;
  try {
    finite(steady);
    double st = number(v.at("steering")), speed = number(v.at("speed"));
    int64_t stamp = v.at("stamp_ns");
    double age = (now - stamp) * 1e-9;
    need(stamp >= 0 && age >= 0 && age <= .1, "stale or future source stamp");
    json t = v.value("targets", json::array({speed, 0., 0., 0., 0.}));
    need(t.size() == 5 &&
             v.value("frame_id", std::string("base_link")) == "base_link" &&
             v.at("control_mode") == 0,
         "VESC speed authority withdrawn or command frame invalid");
    for (auto f : t)
      number(f);
    need(number(t[0]) == speed && number(t[1]) == 0 && number(t[2]) == 0 &&
             number(t[3]) == 0 && number(t[4]) == 0 &&
             std::abs(st) <= p.c("steer_limit") + 1e-6 && speed >= -.05 &&
             speed <= p.c("max_speed") + 1e-6,
         "invalid final VESC command");
    if (stamp <= p.command_stamp)
      return;
    p.command_stamp = stamp;
    p.command_source = steady - age;
    p.command_received = steady;
    p.command_valid = true;
  } catch (const std::exception &) {
    p.command_valid = false;
    invalidate("Final VESC command invalid or authority withdrawn");
    throw;
  }
}
json Runtime::snapshot(int64_t now, double steady, bool anchor) const {
  auto &p = *p_;
  json out = {{"reason", p.localization(now, steady, anchor)},
              {"pose", nullptr},
              {"fault_generation", p.generation},
              {"fault_reason", p.fault_reason},
              {"map_sha256", p.map_identity},
              {"anchor_epoch", p.health.epoch},
              {"anchor_sequence", p.health.sequence},
              {"source_age_s", steady - p.source}};
  if (p.pose && p.health.qualified && p.pose_epoch == p.health.epoch)
    out["pose"] = p.map_pose();
  return out;
}
json Runtime::associate(const json &v, std::optional<double> expected,
                        std::optional<double> max_delta) const {
  auto &p = *p_;
  auto a = pose_values(v);
  auto out = p.ref.project({a[0], a[1]});
  double s = out["progress"];
  auto at = p.ref.at(s);
  double heading = angle(a[2] - number(at["yaw"]));
  need(std::abs(heading) <= 1.5, "Forward reference heading mismatch");
  if (p.config["enforce_corridor"] == true) {
    double c = std::cos(a[2]), h = std::sin(a[2]),
           nx = -std::sin(number(at["yaw"])), ny = std::cos(number(at["yaw"]));
    for (double longitudinal : {-p.c("rear_extent"), p.c("front_extent")})
      for (double side : {-p.c("half_width"), p.c("half_width")}) {
        double cross =
            (a[0] + c * longitudinal - h * side - number(at["x"])) * nx +
            (a[1] + h * longitudinal + c * side - number(at["y"])) * ny;
        need(cross <= p.ref.left && cross >= -p.ref.right,
             "Vehicle footprint outside reference corridor");
      }
  }
  if (expected) {
    double delta =
        wrap(s - *expected + p.ref.length / 2, p.ref.length) - p.ref.length / 2;
    if (max_delta)
      need(std::abs(delta) <= *max_delta, "Reference progress discontinuity");
    out["progress_delta"] = delta;
  }
  out["pose"] = json::array({a[0], a[1], a[2], a[3]});
  out["heading_error"] = heading;
  out["reference"] = at;
  return out;
}
json Runtime::steering(const json &v) const {
  need(v.is_array() && v.size() == 8, "8 finite steering values required");
  std::array<double, 8> a;
  for (int i = 0; i < 8; i++)
    a[i] = number(v[i]);
  auto &p = *p_;
  double ld = std::max(p.lookahead(a[7]), 1e-3), dx = a[3] - a[0],
         dy = a[4] - a[1], yl = -std::sin(a[2]) * dx + std::cos(a[2]) * dy,
         ct = 2 * yl / std::max(ld * ld, 1e-6), he = angle(a[5] - a[2]);
  double raw = std::atan(p.c("wheelbase") * ct) * p.c("lateral_error_gain") +
               p.c("heading_error_gain") * he + p.c("curvature_ff_gain") * a[6];
  double low = p.c("steer_limit"),
         high = std::min(low, p.c("high_speed_max_steering")),
         speed = std::abs(a[7]), limit = low;
  if (speed > p.c("steering_limit_start_speed")) {
    if (p.c("steering_limit_full_speed") <=
            p.c("steering_limit_start_speed") + 1e-6 ||
        speed >= p.c("steering_limit_full_speed"))
      limit = high;
    else
      limit = low + (speed - p.c("steering_limit_start_speed")) /
                        std::max(p.c("steering_limit_full_speed") -
                                     p.c("steering_limit_start_speed"),
                                 1e-6) *
                        (high - low);
  }
  double st = std::clamp(raw, -limit, limit);
  return {{"steering", st},
          {"debug",
           {{"lookahead_distance", ld},
            {"lateral_offset", yl},
            {"heading_error", he},
            {"curvature_term", ct},
            {"path_curvature", a[6]},
            {"raw_steering", raw},
            {"steering_limit", limit},
            {"steering_clipped", std::abs(raw - st) > 1e-6}}}};
}
json Runtime::evaluate(double speed, int64_t now, double steady,
                       bool anchor) const {
  finite(speed);
  auto out = snapshot(now, steady, anchor);
  out["steering"] = 0.;
  out["speed"] = speed;
  if (out["reason"] != "")
    return out;
  try {
    auto association = associate(out["pose"]);
    out.update(association);
    auto a = pose_values(out["pose"]);
    auto target = p_->ref.at(number(out["progress"]) + p_->lookahead(a[3]));
    out["lookahead"] = target;
    out.update(steering(json::array({a[0], a[1], a[2], number(target["x"]),
                                     number(target["y"]), number(target["yaw"]),
                                     number(target["curvature"]), a[3]})));
  } catch (const std::exception &e) {
    out["reason"] = e.what();
  }
  return out;
}
json Runtime::reference_at(double s) const { return p_->ref.at(s); }
json Runtime::project(const json &xy) const {
  need(xy.is_array() && xy.size() >= 2, "planar position required");
  return p_->ref.project({number(xy[0]), number(xy[1])});
}
const json &Runtime::reference_metadata() const { return p_->ref.metadata; }
const json &Runtime::effective_config() const { return p_->config; }
double Runtime::total_length() const { return p_->ref.length; }
std::string Runtime::reference_checksum() const { return p_->ref.checksum; }
json Runtime::reference_points() const {
  json points = json::array();
  auto &r = p_->ref;
  size_t stride = std::max(size_t(1), r.samples.size() / 2000);
  for (size_t i = 0; i < r.samples.size(); i += stride)
    points.push_back(r.at(r.samples[i]));
  if (!points.empty())
    points.push_back(points[0]);
  return points;
}
void Runtime::configure_tracking(double speed, bool repeat) {
  finite(speed);
  need(speed > 0, "positive speed cap required");
  need(!p_->enabled, "disable before configuring tracking");
  p_->speed_limit = std::min(speed, p_->c("max_speed"));
  p_->repeat = repeat;
}
void Runtime::reset_controller() {
  disable();
  p_->progress = p_->traveled = 0;
  p_->last_tick = NAN;
}
json Runtime::enable(int64_t now, double steady, int publishers) {
  observe_clock(now);
  auto &p = *p_;
  auto reason = p.gate(now, steady, publishers);
  if (p.enabled)
    return {{"success", false},
            {"message",
             "Already enabled; disable before resetting lap association"}};
  if (!reason.empty())
    return {{"success", false}, {"message", reason}};
  try {
    p.progress = number(associate(p.map_pose())["progress"]);
  } catch (const std::exception &e) {
    return {{"success", false}, {"message", e.what()}};
  }
  p.traveled = 0;
  p.last_tick = steady;
  p.run_generation = p.generation;
  p.enabled = true;
  p.phase = "RUNNING";
  p.reason = "Enabled";
  return {{"success", true}, {"message", p.reason}};
}
void Runtime::disable() {
  p_->enabled = false;
  p_->phase = "DISABLED";
  p_->reason = "Operator stop requested; explicit re-enable required";
}
json Runtime::tick(int64_t now, double steady, int publishers) {
  observe_clock(now);
  auto &p = *p_;
  json out = {{"speed", 0.},
              {"steering", 0.},
              {"progress", p.progress},
              {"traveled", p.traveled}};
  if (!p.enabled)
    return out;
  auto reason = p.gate(now, steady, publishers);
  if (p.generation != p.run_generation)
    reason = p.fault_reason.empty()
                 ? "Input validity withdrawn; explicit re-enable required"
                 : p.fault_reason;
  if (!reason.empty()) {
    p.fault(reason);
    return out;
  }
  try {
    auto association = associate(p.map_pose());
    double s = number(association["progress"]),
           delta = wrap(s - p.progress + p.ref.length / 2, p.ref.length) -
                   p.ref.length / 2,
           elapsed = steady - p.last_tick;
    need(elapsed >= 0 && elapsed <= .1,
         "PP publication timer stale or clock reset");
    need(delta >= -.05 && std::abs(delta) <= p.c("max_speed") * elapsed + .15,
         "Reference progress discontinuity or reverse traversal");
    p.traveled += delta;
    p.progress = s;
    p.last_tick = steady;
    out["progress"] = s;
    out["traveled"] = p.traveled;
    out["lateral_error"] = association["lateral_error"];
    if (!p.repeat && p.traveled >= p.ref.length - 1e-6) {
      p.enabled = false;
      p.phase = "COMPLETE";
      p.reason = "One lap complete; explicit re-enable required";
      return out;
    }
    auto a = pose_values(association["pose"]);
    auto target = p.ref.at(s + p.lookahead(a[3]));
    out.update(steering(json::array({a[0], a[1], a[2], number(target["x"]),
                                     number(target["y"]), number(target["yaw"]),
                                     number(target["curvature"]), a[3]})));
    out["speed"] = std::min(p.ref.speed(s), p.speed_limit);
    out["lookahead"] = target;
  } catch (const std::exception &e) {
    p.fault(e.what());
  }
  return out;
}
json Runtime::status(int64_t now, double steady, int publishers) const {
  auto &p = *p_;
  json out = snapshot(now, steady);
  out.update({{"controller_generation", p.run_generation},
              {"controller", "enhanced_pp"},
              {"phase", p.phase},
              {"enabled", p.enabled},
              {"reason", p.reason},
              {"gate", p.gate(now, steady, publishers)},
              {"progress", p.progress},
              {"traveled", p.traveled},
              {"speed_limit", p.speed_limit},
              {"map_sha256", p.ref.hash}});
  return out;
}
} // namespace aims_pp
