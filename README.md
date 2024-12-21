# Maze Solver Program

This repository contains a Python-based maze solver application with multiple algorithms and a graphical user interface (GUI). The program can solve text-based mazes using Breadth-First Search (BFS), Depth-First Search (DFS), A* Search, and Bidirectional Search. Additionally, it provides a visual representation of the maze and its solution.

## Features
- **Maze Solving Algorithms**:
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS)
  - A* Search with Manhattan Distance Heuristic
  - Bidirectional Search
- **Graphical User Interface (GUI)**: Built with Tkinter, featuring:
  - Maze file selection
  - Algorithm selection
  - Visualization of maze and solution
- **Support for Image-Based Mazes**: Integration with the `image_maze_gui` module.

## Getting Started

### Prerequisites
- Python 3.x
- Required Python libraries:
  - `Pillow`
  - `tkinter`

Install the dependencies using pip:
```bash
pip install pillow
```

### Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/maze-solver.git
   cd maze-solver
   ```

2. Run the main application:
   ```bash
   python main.py
   ```

## How to Use
1. Launch the application by running `main.py`.
2. Select a maze file (text-based format) using the **Select Maze** button.
3. Choose an algorithm from the options provided:
   - BFS
   - DFS
   - A*
   - Bidirectional Search
4. Click **Solve Maze** to solve the selected maze.
5. View the solution displayed on the canvas.

## Maze Format
The maze file should be in a text format with the following conventions:
- `A`: Starting point
- `B`: Goal point
- `#`: Wall
- Space or any other character: Open path

Example:
```
########
#A     #
# ###  #
#   #  #
### #  #
#     B#
########
```

## Future Improvements
- Add support for more heuristic functions in A* Search.
- Enhance the GUI for better user experience.
- Extend support for solving larger mazes.

## License
This project is open-source and available under the MIT License.

