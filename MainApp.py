import sys
from queue import PriorityQueue
from PIL import Image, ImageDraw, ImageTk
import tkinter as tk
from tkinter import messagebox, filedialog
import tkinter.font as tkFont
from image_maze_gui import MazeSolverApp 


class Node:
    def __init__(self, state, parent=None, action=None, cost=0, heuristic=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost
        self.heuristic = heuristic

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)


class StackFrontier:
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            return self.frontier.pop()  
        
class QueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]  
            self.frontier = self.frontier[1:]  
            return node       


class Maze:
    def __init__(self, filename):
        try:
            with open(filename, encoding='utf-8') as f:
                contents = f.read()
        except UnicodeDecodeError:
            with open(filename, encoding='ISO-8859-1') as f:
                contents = f.read()

        if contents.count("A") != 1:
            raise Exception("Maze must have exactly one start point (A)")
        if contents.count("B") != 1:
            raise Exception("Maze must have exactly one goal (B)")

        contents = contents.splitlines()
        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        self.walls = set()
        for i, line in enumerate(contents):
            for j, char in enumerate(line):
                if char == "A":
                    self.start = (i, j)
                elif char == "B":
                    self.goal = (i, j)
                elif char == "#":
                    self.walls.add((i, j))

        self.solution = None

    def manhattan_distance(self, state):
        x1, y1 = state
        x2, y2 = self.goal
        return abs(x1 - x2) + abs(y1 - y2)

    def neighbors(self, state):
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1)),
        ]
        return [(action, (r, c)) for action, (r, c) in candidates
                if 0 <= r < self.height and 0 <= c < self.width and (r, c) not in self.walls]
    

    def solve_BFS(self):
        self.num_explored = 0

        start = Node(state=self.start, parent=None, action=None)
        frontier = QueueFrontier()  
        frontier.add(start)

        self.explored = set()  

        while True:
            if frontier.empty():
                raise Exception("no solution")

            node = frontier.remove()
            self.num_explored += 1

            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            self.explored.add(node.state)

            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action)
                    frontier.add(child)


    def solve_DFS(self):
        
        self.num_explored = 0

        start = Node(state=self.start, parent=None, action=None)
        frontier = StackFrontier()
        frontier.add(start)

        self.explored = set()

        while True:
            if frontier.empty():
                raise Exception("no solution")

            node = frontier.remove()
            self.num_explored += 1

            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            self.explored.add(node.state)

            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    frontier.add(Node(state=state, parent=node, action=action))

    def solve_Astar(self):
        self.num_explored = 0

        start_node = Node(self.start, cost=0, heuristic=self.manhattan_distance(self.start))
        frontier = PriorityQueue()
        frontier.put(start_node)

        self.explored = set()

        while not frontier.empty():
            node = frontier.get()
            self.num_explored += 1

            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            self.explored.add(node.state)

            for action, state in self.neighbors(node.state):
                if state not in self.explored:
                    child = Node(state, parent=node, action=action,
                                 cost=node.cost + 1, heuristic=self.manhattan_distance(state))
                    frontier.put(child)

    def make_path(self, common_state, l1, l2):
        s1, s2 = None, None
        for i in l1:
            if i.state == common_state:
                s1 = i
                break
        for i in l2:
            if i.state == common_state:
                s2 = i
                break
        s2 = s2.parent
        front = []
        while s1:
            front.append(s1.state)
            s1 = s1.parent
        front.reverse()
        back = []
        while s2:
            back.append(s2.state)
            s2 = s2.parent
        front.extend(back)
        return front

    def bidirectional_bfs(self):
        start_state = self.start
        goal_state = self.goal
        dir1_states = {start_state}
        l1 = [Node(start_state)]
        dir2_states = {goal_state}
        l2 = [Node(goal_state)]
        visited1, visited2 = set(), set()

        while True:
            l1 = [
                Node(state, prev_node) 
                for prev_node in l1 
                for state in self.neighbors(prev_node.state) 
                if state not in visited1
            ]
            visited1.update(node.state for node in l1)
            if not l1:
                break
            dir1_states.update(node.state for node in l1)

            common = dir1_states.intersection(dir2_states)
            if common:
                return self.make_path(common.pop(), l1, l2)

            l2 = [
                Node(state, prev_node) 
                for prev_node in l2 
                for state in self.neighbors(prev_node.state) 
                if state not in visited2
            ]
            visited2.update(node.state for node in l2)
            dir2_states.update(node.state for node in l2)

            common = dir1_states.intersection(dir2_states)
            if common:
                return self.make_path(common.pop(), l1, l2)

        return None  

    def output_image(self, filename, show_solution=True, show_explored=False):
        cell_size = 50
        cell_border = 2

        img = Image.new("RGBA", (self.width * cell_size, self.height * cell_size), "black")
        draw = ImageDraw.Draw(img)

        solution = self.solution[1] if self.solution else None
        for i in range(self.height):
            for j in range(self.width):
                cell = (i, j)
                if cell in self.walls:
                    fill = (40, 40, 40)
                elif cell == self.start:
                    fill = (255, 0, 0)
                elif cell == self.goal:
                    fill = (0, 171, 28)
                elif solution and show_solution and cell in solution:
                    fill = (220, 235, 113)
                elif show_explored and cell in self.explored:
                    fill = (212, 97, 85)
                else:
                    fill = (237, 240, 252)

                draw.rectangle(
                    [(j * cell_size + cell_border, i * cell_size + cell_border),
                     ((j + 1) * cell_size - cell_border, (i + 1) * cell_size - cell_border)],
                    fill=fill
                )

        img.save(filename)

class MainApplication:
    def __init__(self, root):
        self.root = root
        self.root.title("Maze Solver")
        self.root.geometry("650x500")
        self.root.configure(bg="#f0f4f8")
        self.create_widgets()

    def create_widgets(self):
        custom_font = tkFont.Font(family="Helvetica", size=14, weight="bold")
        button_font = tkFont.Font(family="Arial", size=12)

        main_frame = tk.Frame(self.root, bg="#e8eef7")
        main_frame.pack(pady=10, padx=10, fill="both", expand=True)

        maze_frame = tk.Frame(main_frame, bg="#e8eef7", bd=2, relief="groove")
        maze_frame.pack(pady=5, fill="x")

        self.maze_path_entry = tk.Entry(maze_frame, width=50, font=("Arial", 12))
        self.maze_path_entry.grid(row=0, column=0, padx=10, pady=10)

        select_maze_btn = tk.Button(maze_frame, text="Select Maze", command=self.select_maze, bg="#3498db", fg="black", font=button_font, padx=10, pady=5)
        select_maze_btn.grid(row=0, column=1, padx=5, pady=10)

        algo_frame = tk.Frame(main_frame, bg="#f7f9fc", bd=2, relief="groove")
        algo_frame.pack(side=tk.LEFT, padx=20, pady=10, fill="y")

        tk.Label(algo_frame, text="Choose Algorithm", bg="#3498db", fg="white", font=custom_font, pady=10).grid(row=0, column=0, columnspan=2, sticky="ew")

        self.algo_var = tk.StringVar(value="BFS")
        algo_buttons = [
            ("BFS", "Breadth-First Search"),
            ("DFS", "Depth-First Search"),
            ("Astar", "A* Heuristic"),
            ("bi", "Optimal path (Bidirectional Search)")
        ]
        for idx, (value, text) in enumerate(algo_buttons, start=1):
            tk.Radiobutton(algo_frame, text=text, variable=self.algo_var, value=value, font=button_font, bg="#f7f9fc").grid(row=idx, column=0, sticky="w", padx=10, pady=5)

        maze_canvas_frame = tk.Frame(main_frame, bg="#e8eef7", bd=2, relief="groove")
        maze_canvas_frame.pack(side=tk.RIGHT, padx=20, pady=10)
        self.maze_canvas = tk.Canvas(maze_canvas_frame, width=300, height=300, bg="white", bd=1, relief="solid")
        self.maze_canvas.pack()

        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=15)

        solve_btn = tk.Button(button_frame, text="Solve Maze", command=self.solve_maze, bg="#27ae60", fg="black", font=custom_font, padx=20, pady=10)
        solve_btn.pack(side=tk.LEFT, padx=10)

        open_image_maze_btn = tk.Button(button_frame, text="Open Image Maze Solver", command=self.open_image_maze_solver, bg="#27ae60", fg="black", font=custom_font, padx=20, pady=10)
        open_image_maze_btn.pack(side=tk.LEFT, padx=10)

    def open_image_maze_solver(self):
        image_maze_root = tk.Toplevel()  # Create a new window
        app = MazeSolverApp(image_maze_root)
        image_maze_root.mainloop()

    def select_maze(self):
        file_path = filedialog.askopenfilename()
        self.maze_path_entry.delete(0, tk.END)
        self.maze_path_entry.insert(0, file_path)
        self.display_maze(file_path)

    def display_maze(self, path):
        try:
            maze = Maze(path)
            maze.output_image("maze_render.png")
            img = Image.open("maze_render.png")
            img = img.resize((300, 300), Image.LANCZOS)
            self.maze_image = ImageTk.PhotoImage(img)
            self.maze_canvas.create_image(0, 0, anchor=tk.NW, image=self.maze_image)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load maze: {e}")

    def display_solution(self):
        img = Image.open("solution.png")
        img = img.resize((300, 300), Image.LANCZOS)
        self.maze_image = ImageTk.PhotoImage(img)
        self.maze_canvas.create_image(0, 0, anchor=tk.NW, image=self.maze_image)

    def solve_maze(self):
        algorithm = self.algo_var.get()
        maze_path = self.maze_path_entry.get()
        if algorithm and maze_path:
            maze = Maze(maze_path)
            if algorithm == "BFS":
                maze.solve_BFS()
                maze.output_image("solution.png")
                self.display_solution()
                messagebox.showinfo("Solution", "Maze solved using BFS algorithm!")
            elif algorithm == "DFS":
                maze.solve_DFS()
                maze.output_image("solution.png")
                self.display_solution()
                messagebox.showinfo("Solution", "Maze solved using DFS algorithm!")
            elif algorithm == "Astar":
                maze.solve_Astar()
                maze.output_image("solution.png")
                self.display_solution()
                messagebox.showinfo("Solution", "Maze solved using A* algorithm!")
            elif algorithm == "bi":
                maze.solve_Astar()
                maze.output_image("solution.png")
                self.display_solution()
                messagebox.showinfo("Solution", "Maze solved using Bidirectional algorithm!")
        else:
            messagebox.showwarning("Missing Information", "Please select a maze file and an algorithm.")

if __name__ == "__main__":
   root = tk.Tk()
   app = MainApplication(root)
   root.mainloop()