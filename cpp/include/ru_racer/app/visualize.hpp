#pragma once

#include "ru_racer/app/fs_utils.hpp"

#include <filesystem>
#include <cstdlib>

namespace ru_racer::app {

inline void tryAutoVisualize(const std::filesystem::path& out_dir,
                             const std::filesystem::path& bg_img,
                             const std::string& bg_transform,
                             bool show_start_goal) {
  namespace fs = std::filesystem;
  // Non-fatal best-effort visualization by calling the Python plotting script.
  const fs::path nodes_csv = out_dir / "rrt_nodes.csv";
  if (!fs::exists(nodes_csv)) return;

  const fs::path script = fs::path("cpp") / "scripts" / "plot_rrt_tree.py";
  if (!fs::exists(script)) return;

  const std::string nodes_arg = "--nodes " + shellQuote(nodes_csv.string());
  const std::string out1 = "--out " + shellQuote((out_dir / "rrt_tree.png").string());
  std::string base = "python3 " + shellQuote(script.string()) + " " + nodes_arg + " " + out1;

  // Plot tree (no background)
  (void)std::system(base.c_str());

  // Also plot speed-colored trajectories in open space (no background).
  {
    const std::string sg = show_start_goal ? " --show-start-goal" : "";
    const std::string out = "--out " + shellQuote((out_dir / "rrt_speed.png").string());
    const std::string cmd = "python3 " + shellQuote(script.string()) + " " + nodes_arg + sg + " --color-by speed " + out;
    (void)std::system(cmd.c_str());
  }

  // Plot on background if provided
  if (!bg_img.empty() && fs::exists(bg_img)) {
    const std::string bg = "--bg " + shellQuote(bg_img.string());
    const std::string bgt = "--bg-transform " + shellQuote(bg_transform);
    const std::string sg = show_start_goal ? " --show-start-goal" : "";

    const std::string out2 = "--out " + shellQuote((out_dir / "rrt_tree_on_track.png").string());
    const std::string cmd2 = "python3 " + shellQuote(script.string()) + " " + nodes_arg + " " + bg + " " + bgt + sg + " " + out2;
    (void)std::system(cmd2.c_str());

    const std::string out3 = "--out " + shellQuote((out_dir / "rrt_tree_on_track_start_goal.png").string());
    const std::string cmd3 = "python3 " + shellQuote(script.string()) + " " + nodes_arg + " " + bg + " " + bgt + " --show-start-goal " + out3;
    (void)std::system(cmd3.c_str());

    const std::string out4 = "--out " + shellQuote((out_dir / "rrt_speed_on_track.png").string());
    const std::string cmd4 =
        "python3 " + shellQuote(script.string()) + " " + nodes_arg + " " + bg + " " + bgt + " --show-start-goal --color-by speed " + out4;
    (void)std::system(cmd4.c_str());
  }
}

} // namespace ru_racer::app

