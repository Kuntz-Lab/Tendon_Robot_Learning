#include "cliparser/CliParser.h"
#include "collision/CapsuleSequence.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "util/vector_ops.h"
#include "vistendon/EnvironmentRvizPublisher.h"
#include "vistendon/StreamingInterleaver.h"
#include "vistendon/TendonBackboneRvizPublisher.h"
#include "vistendon/TendonRvizPublisher.h"
#include "vistendon/marker_array_conversions.h"

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <chrono>
#include <iterator>

using namespace std::chrono_literals;
using collision::CapsuleSequence;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description("View the given tendon plan in RViz");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "problem toml file");

  parser.add_positional("plan-csv");
  parser.set_required("plan-csv");
  parser.set_description("plan-csv",
                      "CSV file with a plan to show.  Expected columns are\n"
      "                  - tau_i   (for i in [1 : # tendons])\n"
      "                  - theta   (if problem.robot.enable_rotation)\n"
      "                  - s_start (if problem.robot.enable_retraction)");

  parser.add_flag("--smooth-tip");
  parser.set_description("--smooth-tip",
                      "In addition to smoothing over configuration space, this\n"
      "                option tells the viewer to smooth over tip position\n"
      "                space.  Basically, after smoothing over configuration\n"
      "                space, if the tip moves more than some threshold, then\n"
      "                it will be subdivided until each visualization point is\n"
      "                within the tip-position tolerance.\n"
      "                Note: with --smooth-tip, it attempts the make the tip\n"
      "                motion relatively constant by removing overly slow tip\n"
      "                movements as well.");

  parser.add_argflag("--smooth-tip-threshold");
  parser.set_description("--smooth-tip-threshold",
                      "Set the tip threshold for --smooth-tip.  This option\n"
      "                implies --smooth-tip.  The default is to use\n"
      "                problem.robot.specs.dL");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output",
                      "Output interpolated plan to csv file instead of\n"
      "               visualizing to RViz.  Columns are\n"
      "               - i: row index\n"
      "               - same columns as the input plan csv\n"
      "               - tip_{x,y,z}: robot tip positions");
}

/** print the plan to the stream */
void print_plan(std::ostream &out, const std::vector<std::vector<double>> &plan) {
  out << "plan\n";
  for (auto &config : plan) {
    out << "  [";
    bool first = true;
    for (auto &val : config) {
      if (!first) { out << ", "; }
      first = false;
      out << val;
    }
    out << "]\n";
  }
  out << std::endl;
}

/** interpolates the plan to be smooth in configuration space
 *
 * It is interpolated based on a multiple of the problem's min_tension_change
 *
 * @param plan: plan to be interpolated.  Expected to be of length 2 or more
 * @param tension_threshold: how often to interpolate
 *
 * @return a longer plan interpolated.
 */
std::vector<std::vector<double>>
interpolate_plan(const std::vector<std::vector<double>> &plan,
                 double tension_threshold)
{
  if (plan.size() < 2) { throw std::invalid_argument("plan is too short"); }
  std::vector<std::vector<double>> interpolated;
  for (size_t i = 1; i < plan.size(); ++i) {
    auto range = util::range(plan[i-1], plan[i], tension_threshold);
    interpolated.insert(interpolated.end(), range.begin(), range.end());
  }
  return interpolated;
}

/** convert a plan to shapes that can be viewed
 *
 * @param problem: problem description
 * @param plan: plan to convert
 *
 * @return two vectors: (plan_poses, plan_tendon_poses) as a pair
 *   - plan_poses: the pose of the tendon robot at each point in the plan
 *   - plan_tendon_poses: the pose of each tendon within the robot at each
 *     point in the plan
 */
auto plan_to_shapes(const motion_planning::Problem &problem,
                    const std::vector<std::vector<double>> &plan,
                    bool verbose = true)
{
  std::vector<CapsuleSequence> plan_poses;
  std::vector<std::vector<CapsuleSequence>> plan_tendon_poses;
  std::vector<double> min_dlength (problem.robot.tendons.size(), 0.0);
  std::vector<double> max_dlength (problem.robot.tendons.size(), 0.0);

  for (auto &state : plan) {
    auto shape = problem.robot.shape(state);
    auto home_shape = problem.robot.home_shape(state);
    auto [robot_shape, tendon_shapes] =
        vistendon::robot_and_tendon_shape(problem.robot, shape, home_shape);
    plan_poses.emplace_back(std::move(robot_shape));
    plan_tendon_poses.emplace_back(std::move(tendon_shapes));

    auto dl = problem.robot.calc_dl(home_shape.L_i, shape.L_i);
    for (size_t i = 0; i < dl.size(); ++i) {
      if (dl[i] < min_dlength[i]) {
        min_dlength[i] = dl[i];
      } else if (max_dlength[i] < dl[i]) {
        max_dlength[i] = dl[i];
      }
    }
  }

  if (verbose) {
    std::cout << "\nPath string maximum and minimum lengths:\n";
    for (size_t i = 0; i < problem.robot.tendons.size(); i++) {
      std::cout << "  tendon[" << i << "]:   [" << min_dlength[i] << ", "
                << max_dlength[i] << "]\n";
    }
    std::cout << std::endl;
  }

  return std::make_pair(plan_poses, plan_tendon_poses);
}

/** smooth plan movement based on tip distance
 *
 * Attempts to make the motion smoother by
 *  1. subdividing when tip moves more than the threshold
 *  2. removing intermediate poses when the tip movement is less than half of
 *     the threshold
 * This attempts to achieve a near-constant tip movement throughout the
 * visualization of the plan.
 *
 * @param problem: problem description
 * @param tip_threshold: threshold for adding or removing poses
 * @param plan (in/out): plan to modify
 * @param poses (in/out): robot shape to modify
 * @param tendon_poses (in/out): tendon shapes to modify
 *
 * @return void.  The plan, poses, and tendon_poses are modified in place
 */
void smooth_plan_tip_movement(
    const motion_planning::Problem &problem,
    double tip_threshold,
    std::vector<std::vector<double>> &plan,
    std::vector<CapsuleSequence> &poses,
    std::vector<std::vector<CapsuleSequence>> &tendon_poses)
{
  if (plan.size() < 2) { throw std::invalid_argument("plan is too short"); }

  if (plan.size() != poses.size() || plan.size() != tendon_poses.size()) {
    throw std::invalid_argument("vector size mismatch");
  }

  std::cout << "before tip smoothing:\n"
               "  threshold:           " << tip_threshold << "\n"
               "  plan.size():         " << plan.size() << "\n"
               "  poses.size():        " << poses.size() << "\n"
               "  tendon_poses.size(): " << tendon_poses.size() << "\n"
            << std::endl;

  std::vector<std::vector<double>> new_plan;
  std::vector<CapsuleSequence> new_poses;
  std::vector<std::vector<CapsuleSequence>> new_tendon_poses;

  // pop off one element from given vectors and move to new vectors
  auto move_pop = [&plan, &poses, &tendon_poses,
                   &new_plan, &new_poses, &new_tendon_poses]()
  {
    new_plan.emplace_back(std::move(plan.back()));
    new_poses.emplace_back(std::move(poses.back()));
    new_tendon_poses.emplace_back(std::move(tendon_poses.back()));
    plan.pop_back();
    poses.pop_back();
    tendon_poses.pop_back();
  };

  // move the final position
  move_pop();

  // work my way from the back of the plan
  while (plan.size() > 0) {
    auto &prev_tip = new_poses.back().points.back();
    auto &curr_tip = poses.back().points.back();
    double dtip = (prev_tip - curr_tip).norm();

    // if the tip distance is larger than our threshold, then subdivide
    // otherwise, check if the next pose is within tip distance of our threshold
    if (dtip > tip_threshold) {
      // I can add to the plan and poses so that the next while loop iteration
      // will check this subdivision.
      size_t N(std::ceil(dtip / tip_threshold));
      auto plan_subdiv = util::linspace(plan.back(), new_plan.back(), N + 1);
      plan_subdiv.erase(plan_subdiv.begin());
      plan_subdiv.pop_back();
      auto [poses_subdiv, tendon_poses_subdiv] =
          plan_to_shapes(problem, plan_subdiv, false);
      plan.insert(plan.end(), std::make_move_iterator(plan_subdiv.begin()),
                              std::make_move_iterator(plan_subdiv.end()));
      poses.insert(poses.end(), std::make_move_iterator(poses_subdiv.begin()),
                                std::make_move_iterator(poses_subdiv.end()));
      tendon_poses.insert(tendon_poses.end(),
                          std::make_move_iterator(tendon_poses_subdiv.begin()),
                          std::make_move_iterator(tendon_poses_subdiv.end()));
      continue;
    }

    // see if a future pose is still within the tip tolerance.
    // we can drop the poses between the previous and this future pose
    auto it = poses.rbegin();
    for (; it != poses.rend() && dtip < tip_threshold; ++it) {
      auto &future_tip = it->points.back();
      dtip = (prev_tip - future_tip).norm();
    }
    auto to_remove = std::distance(poses.rbegin(), std::prev(it)) - 1;
    // skip these ones
    if (to_remove > 0) {
      plan.erase(std::prev(plan.end(), to_remove), plan.end());
      poses.erase(std::prev(poses.end(), to_remove), poses.end());
      tendon_poses.erase(std::prev(tendon_poses.end(), to_remove),
                         tendon_poses.end());
    }

    // move over the last one
    move_pop();
  }

  plan.clear();
  poses.clear();
  tendon_poses.clear();

  plan.reserve(new_plan.size());
  poses.reserve(new_poses.size());
  tendon_poses.reserve(new_tendon_poses.size());

  // move back and reverse ordering
  plan.insert(plan.end(), std::make_move_iterator(new_plan.rbegin()),
                          std::make_move_iterator(new_plan.rend()));
  poses.insert(poses.end(), std::make_move_iterator(new_poses.rbegin()),
                            std::make_move_iterator(new_poses.rend()));
  tendon_poses.insert(tendon_poses.end(),
                      std::make_move_iterator(new_tendon_poses.rbegin()),
                      std::make_move_iterator(new_tendon_poses.rend()));

  std::cout << "after tip smoothing:\n"
               "  plan.size():         " << plan.size() << "\n"
               "  poses.size():        " << poses.size() << "\n"
               "  tendon_poses.size(): " << tendon_poses.size() << "\n"
            << std::endl;
}

void view_plan(const motion_planning::Problem &problem,
               const std::vector<CapsuleSequence> &plan_poses,
               const std::vector<std::vector<CapsuleSequence>> &plan_tendon_poses)
{
  // initialize visualization
  auto tendon_node = std::make_shared<rclcpp::Node>("view_tendon");

  vistendon::TendonBackboneRvizPublisher robot_publisher(tendon_node);
  vistendon::TendonBackboneRvizPublisher goal_publisher(
      tendon_node, "/map", "tendon-robot-goal");
  vistendon::TendonRvizPublisher tendon_publisher(tendon_node);
  vistendon::TendonRvizPublisher goal_tendon_publisher(
      tendon_node, "/map", "tendon-robot-goal-tendons");
  vistendon::EnvironmentRvizPublisher env_publisher(tendon_node);

  {
    auto [start_robot_shape, start_tendon_shapes] =
        vistendon::robot_and_tendon_shape(problem.robot, problem.start_shape(),
                                          problem.start_home_shape());
    robot_publisher .set_color(1.0f, 1.0f, 0.0f, 0.4f); // transparent yellow
    tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
    robot_publisher .set_robot(start_robot_shape);
    tendon_publisher.set_tendons(start_tendon_shapes);
  }

  {
    auto [goal_robot_shape, goal_tendon_shapes] =
        vistendon::robot_and_tendon_shape(problem.robot, problem.goal_shape(),
                                          problem.goal_home_shape());
    goal_publisher.set_color(0.0f, 1.0f, 0.0f, 0.4f); // transparent green
    goal_tendon_publisher.set_color(0.0f, 0.0f, 0.0f, 1.0f); // solid black
    goal_publisher.set_robot(goal_robot_shape);
    goal_tendon_publisher.set_tendons(goal_tendon_shapes);
  }

  env_publisher.set_color(0.0f, 0.0f, 1.0f, 0.5f); // transparent blue
  env_publisher.set_env(problem.env);

  // create timer to cycle through the plan poses
  auto timer = tendon_node->create_wall_timer(33ms,
    [&plan_poses, &plan_tendon_poses, &robot_publisher, &tendon_publisher]() -> void {
      static size_t i = 0;
      robot_publisher.set_robot(plan_poses.at(i));
      tendon_publisher.set_tendons(plan_tendon_poses.at(i));
      i = (i + 1) % plan_poses.size();
    });

  std::cout << "Publishing to RViz.  Press Ctrl-C to exit" << std::endl;
  rclcpp::spin(tendon_node);
  rclcpp::shutdown();
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  rclcpp::init(arg_count, arg_list);
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto problem = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);
  auto plan = motion_planning::Problem::load_plan(parser["plan-csv"]);
  auto default_state_threshold = problem.min_tension_change * 10.0;
  auto default_tip_threshold = problem.robot.specs.dL;
  bool smooth_tip = parser.has("--smooth-tip")
                 || parser.has("--smooth-tip-threshold");
  auto smooth_tip_threshold = parser.get("--smooth-tip-threshold",
                                         default_tip_threshold);
  bool go_to_file = parser.has("--output");
  auto output     = parser.get("--output", std::string());

  print_plan(std::cout, plan);

  if (plan.size() <  2) {
    std::cerr << "Error: plan is not long enough (size = " << plan.size() << ")"
              << std::endl;
    return 1;
  }

  //vistendon::StreamingInterleaver interleaver(&problem.robot,
  //                                            default_state_threshold);
  //interleaver.set_smooth_tip(smooth_tip);
  //interleaver.set_tip_threshold(smooth_tip_threshold);
  //auto plan_poses = interleaver.interleave(plan);

  //// empty tendon poses
  //std::vector<std::vector<CapsuleSequence>> plan_tendon_poses {plan_poses.size()};

  auto interpolated_plan = interpolate_plan(plan, default_state_threshold);
  auto [plan_poses, plan_tendon_poses] =
      plan_to_shapes(problem, interpolated_plan);
  if (smooth_tip) {
    smooth_plan_tip_movement(problem, smooth_tip_threshold,
                             interpolated_plan, plan_poses, plan_tendon_poses);
  }

  assert(interpolated_plan.size() == plan_poses.size());
  assert(interpolated_plan.size() == plan_tendon_poses.size());

  if (!go_to_file) {
    view_plan(problem, plan_poses, plan_tendon_poses);
  } else {
    csv::CsvRow header;
    {
      std::ifstream fin;
      util::openfile_check(fin, parser["plan-csv"]);
      csv::CsvReader reader(fin);
      header = *reader.header();
    }
    std::ofstream fout;
    util::openfile_check(fout, output);
    csv::CsvWriter writer(fout);
    header.emplace_back("tip_x");
    header.emplace_back("tip_y");
    header.emplace_back("tip_z");
    writer.write_row(header);

    for (size_t i = 0; i < interpolated_plan.size(); ++i) {
      auto &config = interpolated_plan[i];
      auto &capseq = plan_poses[i];
      writer << i;
      for (auto &val : config) {
        writer << val;
      }
      auto &tip = capseq.points.back();
      writer << tip[0] << tip[1] << tip[2];
      writer.new_row();
    }
  }

  return 0;
}
