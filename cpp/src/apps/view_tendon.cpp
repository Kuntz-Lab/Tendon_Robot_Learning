#include "cliparser/CliParser.h"
#include "collision/CapsuleSequence.h"
#include "collision/Sphere.h"
#include "collision/VoxelOctree.h"
#include "cpptoml/toml_conversions.h"
#include "motion-planning/Environment.h"
#include "motion-planning/VoxelEnvironment.h"
#include "vistendon/EnvironmentRvizPublisher.h"
#include "vistendon/ManualRvizMarkerArrayPublisher.h"
#include "vistendon/TendonBackboneRvizPublisher.h"
#include "vistendon/TendonRvizPublisher.h"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Publish the shape of a single hard-coded backbone shape to RViz\n"
      "  This is an example of how to do visualize the robot and to\n"
      "  visually inspect this shape.");
  parser.add_flag("--sphere");
  parser.set_description("--sphere",
                      "Show spheres along the backbone instead of a mesh.\n"
      "                Cannot be specified with --voxel.");
  parser.add_argflag("--voxel");
  parser.set_description("--voxel",
                      "Show voxel shape instead of a mesh.\n"
      "                Give a toml file with a [voxel_environment] section.\n"
      "                Cannot be specified with --sphere.");
  parser.add_flag("--backbone");
  parser.set_description("--backbone",
                      "Only for --voxel, only voxelize backbone centerline.");
}

/// Return false if conflicting flags are specified, true otherwise.
bool check_parsed(const CliParser &parser) {
  if (parser.has("--sphere") && parser.has("--voxel")) {
    std::cerr << "Cannot specify both --sphere and --voxel\n";
    return false;
  }
  if (parser.has("--backbone") && !parser.has("--voxel")) {
    std::cerr << "Cannot specify --backbone without --voxel\n";
    return false;
  }
  return true;
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);
  if (!check_parsed(parser)) {
    return 1;
  }

  collision::CapsuleSequence sequence {{
    {0,                      0,                      0                    },
    {0.00013282379244664586, 8.8542686704915893e-05, 0.0048183362822623761},
    {0.00052783960456719817, 0.00017734411430676313, 0.0096222944924363833},
    {0.0011807888469575864,  0.00023774422435236941, 0.014398475543442689 },
    {0.0020848691901629154,  0.00024181221791107988, 0.019134001636465916 },
    {0.0032308218771827562,  0.00016204630127320717, 0.023816186856668101 },
    {0.0046069523943798856, -2.7725061486809984e-05, 0.028432680572448337 },
    {0.0061991284067021016, -0.00035193988107289743, 0.032971585424409165 },
    {0.0079909432484972492, -0.00083330351577287284, 0.037421377328421053 },
    {0.0099636590414980825, -0.0014929670596176901,  0.041770601259497581 },
    {0.012096483349451518,  -0.0023495436147191605,  0.046008424857334432 },
    {0.014366757923367069,  -0.0034187943683490949,  0.050124817023101453 },
    {0.016750135665100578,  -0.0047137365964838585,  0.054110548703798278 },
    {0.019220482205211914,  -0.0062448052947935203,  0.057957099895916533 },
    {0.021750667340813275,  -0.008019190198795885,   0.061657335489431711 },
    {0.024312913976440365,  -0.010040679141174146,   0.065205714795564404 },
    {0.026878975393225701,  -0.012309912655683711,   0.068598291197242908 },
    {0.029420097745194464,  -0.014824600981260724,   0.071832780109992153 },
    {0.031908079887735023,  -0.017579403035378263,   0.074908999842337148 },
    {0.034315681524303088,  -0.020565993965422275,   0.077828976374351017 },
    {0.036616697558473403,  -0.023773401539954657,   0.080596867379787301 },
    {0.038785880570702397,  -0.027188300501592648,   0.083219048076941721 },
    {0.040799881477261858,  -0.030795295294191095,   0.085704083226517097 },
    {0.042637572355448683,  -0.03457712912139755,    0.088062652661978325 },
    {0.044279952290016557,  -0.038514956165630457,   0.090307409998782284 },
    {0.045709898811377929,  -0.042588556794032881,   0.092453001552805364 },
    {0.04691283674803131,   -0.046776697971979746,   0.094515610880525625 },
    {0.047876926155331852,  -0.051057344086618592,   0.096512743273968762 },
    {0.048592856307176031,  -0.055407764210548729,   0.098463071437137284 },
    {0.049053496497500931,  -0.059804466118157332,   0.10038640775831636  },
    {0.04925440285973795,   -0.064223533491367668,   0.1023029919714103   },
    {0.049193918878868759,  -0.068640799325951934,   0.10423320813694903  },
    {0.048872975101987508,  -0.073031822319777631,   0.10619744256943542  },
    {0.048294857894473034,  -0.077371535870307609,   0.10821604897814674  },
    {0.047465604350272333,  -0.081634790293452553,   0.11030850839790331  },
    {0.046394046775197574,  -0.085796573499466469,   0.11249312009473898  },
    {0.045091655369252255,  -0.089831984936857096,   0.11478689615013155  },
    {0.043572546963745788,  -0.093715821745073757,   0.11720552268591571  },
    {0.041853549366646772,  -0.097423574286689893,   0.11976254988524013  },
    {0.039954123043569545,  -0.10093178396060946,    0.12246911529529105  },
    {0.037896267549349043,  -0.10421790770166464,    0.12533401771210487  },
    }, 0.01};

  std::string node_name         = "view_tendon";
  std::string frame             = "/map";
  std::string tendon_namespace  = "tendon-backbone";
  std::string env_namespace     = "obstacles";
  vistendon::Color tendon_color = {1.0f, 1.0f, 0.0f, 1.0f}; // yellow
  vistendon::Color env_color    = {0.0f, 0.0f, 1.0f, 0.2f}; // transparent blue

  rclcpp::init(arg_count, arg_list);
  auto tendon_node = std::make_shared<rclcpp::Node>(node_name);

  std::unique_ptr<vistendon::RvizMarkerArrayPublisher> tendon_publisher;
  motion_planning::VoxelEnvironment venv;
  if (parser.has("--sphere") || parser.has("--voxel")) {
    tendon_publisher.reset(
        new vistendon::ManualRvizMarkerArrayPublisher(tendon_node, frame,
                                                      tendon_namespace));
    auto manual_pub = static_cast<vistendon::ManualRvizMarkerArrayPublisher*>(
        tendon_publisher.get());
    if (parser.has("--sphere")) {
      for (auto &point : sequence.points) {
        manual_pub->add_marker(
            vistendon::to_marker(collision::Sphere{point, sequence.r}));
      }
    } else if (parser.has("--voxel")) {
      venv = cpptoml::from_file<motion_planning::VoxelEnvironment>(
          parser["--voxel"]);
      auto voxels = venv.get_obstacles();
      auto robot_voxels = voxels->empty_copy();
      if (parser.has("--backbone")) {
        std::cout << "robot_voxels.dx: " << robot_voxels.dx() << "\n"
                  << "robot_voxels.dy: " << robot_voxels.dy() << "\n"
                  << "robot_voxels.dz: " << robot_voxels.dz() << "\n";
        auto prev = sequence.points.front();
        for (auto &point : sequence.points) {
          std::cout << "backbone distance: " << (point - prev).norm() << std::endl;
          prev = point;
          robot_voxels.add(point);
        }
      } else {
        for (auto &point : sequence.points) {
          robot_voxels.add(collision::Sphere{point, sequence.r});
        }
      }
      manual_pub->add_marker(vistendon::to_marker(robot_voxels.to_mesh()));
    }
  } else {
    tendon_publisher.reset(
        new vistendon::TendonBackboneRvizPublisher(tendon_node, frame,
                                                   tendon_namespace));
    auto backbone_pub = static_cast<vistendon::TendonBackboneRvizPublisher*>(
        tendon_publisher.get());
    backbone_pub->set_robot(sequence);
  }
  tendon_publisher->set_color(tendon_color);

  motion_planning::Environment env;
  env.push_back(collision::Sphere{{0, 0, 0.15}, 0.06});
  vistendon::EnvironmentRvizPublisher env_publisher(
      tendon_node, frame, env_namespace);
  env_publisher.set_color(env_color);
  env_publisher.set_env(env);

  rclcpp::spin(tendon_node);
  rclcpp::shutdown();
  return 0;
}

