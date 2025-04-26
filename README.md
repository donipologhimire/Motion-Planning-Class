# Motion Planning 

This repository contains implementations of various motion planning algorithms

## Repository Contents

- **BUG1_TEAM1.py**: Implementation of the Bug 1 algorithm for path planning around obstacles
- **computeBFSpath.m**: Function to compute paths using Breadth-First Search (BFS)
- **computeBFStree.m**: Function to construct a BFS tree from an adjacency table
- **doTwoSegmentsIntersect.m**: Helper function to determine if two line segments intersect
- **trapezoidal_decomposition.m**: Implementation of the trapezoidal decomposition algorithm for path planning

## Algorithms

### Bug 1 Algorithm
A simple path planning algorithm that follows these steps:
1. Move directly toward the goal
2. When an obstacle is encountered, follow its boundary completely
3. Leave the obstacle at the point closest to the goal
4. Resume moving toward the goal

### Trapezoidal Decomposition
A cell decomposition approach that:
1. Divides the workspace into trapezoid-shaped cells
2. Builds a roadmap by connecting cell centroids
3. Uses BFS to find a path from start to goal through the roadmap

## Running the Code

### Bug 1 Algorithm

Required libraries:
- numpy
- matplotlib
- shapely

### Trapezoidal Decomposition

Required MATLAB toolboxes:
- Mapping Toolbox

## Platform Support

The code has been tested on:
- Windows
- macOS (See installation instructions for macOS in README_PROJECT1.txt)





