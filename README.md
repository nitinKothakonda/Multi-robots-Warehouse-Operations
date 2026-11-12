# Multi-Robot Warehouse Operations (MATLAB simulation)

MATLAB simulation of 10 robots doing pick-and-carry tasks in a 14 m x 10 m warehouse with shelves drawn as obstacles. Course project (MAE / multi-robot systems).

## Files

| File | What it does |
|---|---|
| `maefinal.m` | Task allocation plus cooperative carrying. 10 robots, single-robot tasks and large-box tasks that need 2 robots. Uses a cost matrix with greedy task allocation (no A*). Plots formation error for one cooperative task and a Gantt-style chart of robot activity. |
| `maefinal3.m` | Dynamic version: tasks spawn over time, simple collision avoidance, and battery drain with charging stations. Robots head to charge when battery is low. |
| `maeplot.m` | Report-plot generator. Loads `simulation_data.mat` and plots paths, robot states and cooperative-task data. |

## How to run

1. Open MATLAB and set the repository folder as the current folder.
2. Run `maefinal.m` or `maefinal3.m`. Each is a standalone script (local functions are at the bottom of the file) and opens its own figures.

## Known limitations

- Shelves are visual only in `maefinal.m`; robots do not plan paths around them.
- `maeplot.m` needs `simulation_data.mat`, and its error message points to a `maefinal2.m` that is not in this repository. None of the scripts here save that file, so `maeplot.m` will not run as-is.
