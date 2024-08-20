#include "cliparser/CliParser.h"
#include "collision/VoxelOctree.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "util/openfile_check.h"
#include "util/vector_ops.h"

//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <itkImage.h>
#include <itkImageFileReader.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <functional>
#include <iomanip>
#include <random>
#include <string>
#include <vector>

namespace E = Eigen;
//using NNType = ompl::NearestNeighborsGNATNoThreadSafety<E::Vector3d>;
using NNType = ompl::NearestNeighborsGNAT<E::Vector3d>;

using PixelType  = typename motion_planning::VoxelEnvironment::PixelType;
using ImageType  = typename motion_planning::VoxelEnvironment::ImageType;
using ImagePtr   = typename motion_planning::VoxelEnvironment::ImagePtr;

namespace {

namespace defaults {
  const std::string output = "samples.csv";
  const int number = 1000;
  const int k = 10;
}

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Generate many valid samples of the configuration space and store them in\n"
      "  a CSV file.  It will store the configuration state as well as the tip\n"
      "  position and tip orientation.");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "The problem specification toml file to use");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output", "CSV file to output\n"
      "                (default is " + defaults::output + ")");

  parser.add_flag("-g", "--grid");
  parser.set_description("--grid", "Sample in a grid instead of randomly.\n"
      "                The number of samples with --number will specify the\n"
      "                number in each dimension instead of total number of\n"
      "                samples.");

  parser.add_argflag("-N", "--number");
  parser.set_description("--number",
                      "Number of valid samples to take (if doing random\n"
      "                samples).\n"
      "                If --grid is specified, then this is the\n"
      "                discretization of each control dimension, which will\n"
      "                result in N^k total samples, where k is the number of\n"
      "                control dimensions.\n"
      "                (default is " + std::to_string(defaults::number) + ")");

  parser.add_flag("--repel-near");
  parser.set_description("--repel-near",
                      "Have previous samples from this run repel new samples\n"
      "                in the tip-position space.  Multiple samples are taken\n"
      "                and the one with the highest distance from these\n"
      "                samples will be kept.");

  parser.add_argflag("--repel-near-these");
  parser.set_description("--repel-near-these",
                      "A CSV file full of previous samples.  The samples in\n"
      "                this file are to repel new samples in the tip-position\n"
      "                space.  Multiple samples are taken and the one with\n"
      "                the highest distance from these samples will be kept.");

  parser.add_argflag("-k");
  parser.set_description("-k", "Number of samples for repelling\n"
      "                (default is " + std::to_string(defaults::k) + ")");

  parser.add_flag("--voxel-and-nonvoxel");
  parser.set_description("--voxel-and-nonvoxel",
                      "Use the [voxel_environment] from the problem toml file\n"
      "                as well as the [environment] section for collision\n"
      "                checking.  Basically, only generate points that are\n"
      "                valid in BOTH environments.\n"
      "                This implies --voxel.");

  parser.add_flag("-v", "--voxel");
  parser.set_description("--voxel", "Use the [voxel_environment] from the\n"
      "                problem toml file instead of the [environment]\n"
      "                section for collision checking.");

  parser.add_flag("--backbone");
  parser.set_description("--backbone", "Only collision check of backbone\n"
      "                against the voxel environment instead of the full\n"
      "                robot shape.  This option is only used if --voxel\n"
      "                is specified.");
}

std::vector<E::Vector3d> load_tip_samples(const std::string &csv_file) {
  std::ifstream in;
  util::openfile_check(in, csv_file);
  csv::CsvReader reader(in);
  csv::CsvRow row;
  std::vector<E::Vector3d> samples;
  while (reader >> row) {
    samples.emplace_back(E::Vector3d{std::stod(row["tip_x"]),
                                     std::stod(row["tip_y"]),
                                     std::stod(row["tip_z"])});
  }
  return samples;
}

struct Config {
  tendon::TendonResult shape;
  std::vector<double> state;
};

std::function<bool(Config)>
make_voxel_collision_function(const motion_planning::Problem &problem,
                              bool backbone_only)
{
  auto voxels = problem.venv.get_obstacles();

  if (backbone_only) {
    return [&problem, voxels = std::move(voxels)] (Config conf) {
      auto &rotated_points = conf.shape.p;
      problem.venv.rotate_points(rotated_points);
      auto fk_voxels = voxels->empty_copy();
      for (auto &pos : rotated_points) { fk_voxels.add(pos); }
      return voxels->collides(fk_voxels);
    };
  } else {
    return [&problem, voxels = std::move(voxels)] (Config conf) {
      auto &rotated_points = conf.shape.p;
      problem.venv.rotate_points(rotated_points);
      auto fk_voxels = voxels->empty_copy();
      for (auto &pos : rotated_points) {
        fk_voxels.add_sphere({pos, problem.robot.r});
      }
      return voxels->collides(fk_voxels);
    };
  }
}

// does rejection sampling to generate valid samples
template <typename SamplerFunc, typename ValidFunc>
class ValidSampler {
public:
  ValidSampler(SamplerFunc &sampler, ValidFunc &is_valid)
    : _sampler(sampler), _is_valid(is_valid) {}

  Config operator()() const {
    Config conf;
    do {
      conf = _sampler();
    } while(!_is_valid(conf));
    return conf;
  }

private:
  SamplerFunc &_sampler;
  ValidFunc &_is_valid;
};

// takes k valid samples and returns the one furthest from neighbors
template<typename SamplerType>
class RepulsiveNeighborSampler {
public:
  RepulsiveNeighborSampler(SamplerType &sampler,
                           NNType *neighborhood,
                           size_t k = 3)
    : _nn(neighborhood)
    , _k(k)
    , _sampler(sampler)
  {}

  size_t k() const      { return _k; }
  void set_k(size_t k)  { _k  =  k;  }

  Config operator()() const {
    Config conf = _sampler();
    if (_nn->size() == 0) { return conf; }
    double dist = distance(conf);
    for (size_t i = 1; i < _k; ++i) {
      Config candidate = _sampler();
      double candidate_dist = distance(candidate);
      if (dist < candidate_dist) {
        conf = candidate;
        dist = candidate_dist;
      }
    }
    return conf;
  }

  double distance(const Config &conf) const {
    auto &tip = conf.shape.p.back();
    auto nearest = _nn->nearest(tip);
    return _nn->getDistanceFunction()(tip, nearest);
  }

private:
  NNType *_nn;
  size_t _k;
  SamplerType &_sampler;
};

class GridIterator {
public:
  using vec = std::vector<double>;
  using subiter = vec::const_iterator;

private:
  static const std::vector<vec> empty;

public:

  // the end iterator
  GridIterator () : _vecs(empty), _is_end(true), _N(0) {}

  explicit GridIterator (const std::vector<vec> &grid_values)
    : _vecs(grid_values)
    , _is_end(false)
    , _N(grid_values.size())
  {
    for (auto &v : grid_values) {
      _iters.emplace_back(v.begin());
      _current.emplace_back(v.front());
      if (v.size() == 0) {
        _is_end = true;
        break;
      }
    }
  }

  GridIterator& operator++() { // prefix
    if (_is_end) { return *this; }

    for (size_t i = 0; i < _N; ++i) {
      ++_iters[i];
      if (_iters[i] == _vecs[i].end()) {
        if (i < _N - 1) {
          _iters[i] = _vecs[i].begin();
          _current[i] = _vecs[i].front(); // update current
        } else {
          _is_end = true;
          return *this;
        }
      } else {
        _current[i] = *_iters[i];
        break;
      }
    }

    return *this;
  }

  operator bool() const { return !_is_end; }

  const vec& operator*() const { return _current; }
  const vec* operator->() const { return &_current; }

  bool operator== (const GridIterator &other) const {
    if (_is_end && other._is_end) { return true;  }
    if (_is_end || other._is_end) { return false; }

    for (size_t i = 0; i < _N; ++i) {
      if (_iters[i] != other._iters[i]) {
        return false;
      }
    }
    return true;
  }

  bool operator!= (const GridIterator &other) const { return !(*this == other); }

private:
  const std::vector<vec> &_vecs;
  bool                    _is_end;
  size_t                  _N;
  std::vector<subiter>    _iters;
  vec                     _current;
};

const std::vector<GridIterator::vec> GridIterator::empty;

class Grid {
public:
  Grid() {}

  void add_dim(const std::vector<double> &values) {
    vecs.emplace_back(values);
  }

  void add_dim(double lower, double upper, size_t N) {
    auto dx = (upper - lower) / (N - 1);
    add_dim(util::range(lower, upper, dx));
  }

  GridIterator begin() const { return GridIterator(vecs); }
  GridIterator end()   const { return GridIterator();     }

private:
  std::vector<std::vector<double>> vecs;
};

void write_header(csv::CsvWriter &writer, const motion_planning::Problem &problem) {
  writer << "probfile";
  for (size_t i = 0; i < problem.robot.tendons.size(); ++i) {
    writer << "tau_" + std::to_string(i+1);
  }
  if (problem.robot.enable_rotation) {
    writer << "theta";
  }
  if (problem.robot.enable_retraction) {
    writer << "s_start";
  }
  writer << "tip_x" << "tip_y" << "tip_z"
         << "quat_w" << "quat_x" << "quat_y" << "quat_z";
  writer.new_row();
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto probfile       = parser["problem"];
  auto problem        = cpptoml::from_file<motion_planning::Problem>(probfile);
  auto output         = parser.get("--output", defaults::output);
  auto number         = parser.get("--number", defaults::number);
  auto k              = parser.get("-k", defaults::k);
  bool both_envs      = parser.has("--voxel-and-nonvoxel");
  bool voxel_samples  = parser.has("--voxel") || both_envs;
  bool voxel_backbone = parser.has("--backbone");

  using ValidFunc = std::function<bool(Config)>;
  ValidFunc is_config_valid = nullptr;
  if (!voxel_samples) {
    problem.env.setup(); // to make collision checking thread-safe
    is_config_valid = [&problem](Config conf) {
      return problem.is_valid(conf.state, problem.robot.home_shape(conf.state),
                              conf.shape);
    };
  } else {
    auto is_voxel_collision = make_voxel_collision_function(problem,
                                                            voxel_backbone);
    if (!both_envs) {
      is_config_valid = [is_voxel_collision = std::move(is_voxel_collision),
                         &problem]
        (Config conf) {
          return problem.robot.is_valid(conf.state,
                                        problem.robot.home_shape(conf.state),
                                        conf.shape)
              && !is_voxel_collision(conf);
        };
    } else {
      problem.env.setup(); // to make collision checking thread-safe
      is_config_valid = [is_voxel_collision = std::move(is_voxel_collision),
                         &problem]
        (Config conf) {
          return problem.robot.is_valid(conf.state,
                                        problem.robot.home_shape(conf.state),
                                        conf.shape)
              && !is_voxel_collision(conf)
              && !problem.env.collides(
                    collision::CapsuleSequence{conf.shape.p, problem.robot.r});
        };
    }
  }

  std::ofstream out;
  util::openfile_check(out, output);
  csv::CsvWriter writer(out);
  std::cout << "writing to " << output << std::endl;

  write_header(writer, problem);

  auto write_row = [&writer, &probfile](Config config) {
    writer << probfile;

    for (auto val : config.state) { writer << val; }

    auto tip_position = config.shape.p.back();
    writer << tip_position[0] << tip_position[1] << tip_position[2];

    auto tip_quat = E::Quaterniond(config.shape.R.back());
    writer << tip_quat.w() << tip_quat.x() << tip_quat.y() << tip_quat.z();

    writer.new_row();
  };

  if (parser.has("--grid")) {
    Grid grid;
    for (auto &tendon : problem.robot.tendons) {
      grid.add_dim(0.0, tendon.max_tension, number);
    }
    if (problem.robot.enable_rotation) {
      grid.add_dim(-M_PI, M_PI, number);
    }
    if (problem.robot.enable_retraction) {
      grid.add_dim(0.0, problem.robot.specs.L, number);
    }

    for (auto state : grid) {
      Config config;
      config.state = state;
      config.shape = problem.robot.shape(state);
      if (is_config_valid(config)) {
        write_row(config);
      }
    }
  } else {
    NNType neighborhood;
    neighborhood.setDistanceFunction(
        [](auto a, auto b) { return (b - a).norm(); });
    bool repel = parser.has("--repel-near-these") || parser.has("--repel-near");

    auto random_sampler = [&problem]() {
      Config conf;
      conf.state = problem.robot.random_state();
      conf.shape = problem.robot.shape(conf.state);
      return conf;
    };
    ValidSampler regular_sampler(random_sampler, is_config_valid);
    RepulsiveNeighborSampler repulse_sampler(regular_sampler, &neighborhood, k);

    using SamplerFunc = std::function<Config(void)>;
    SamplerFunc sampler = nullptr;
    if (repel) {
      sampler = [&repulse_sampler]() { return repulse_sampler(); };
    } else {
      sampler = [&regular_sampler]() { return regular_sampler(); };
    }

    if (parser.has("--repel-near-these")) {
      neighborhood.add(load_tip_samples(parser["--repel-near-these"]));
    }
    # pragma omp parallel for
    for (int i = 0; i < number; ++i) {
      auto config = sampler();
      if (parser.has("--repel-near")) {
        # pragma omp critical
        neighborhood.add(config.shape.p.back());
      }
      # pragma omp critical
      write_row(config);
    }
  }

  return 0;
}
