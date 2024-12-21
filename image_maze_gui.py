import cv2
import numpy as np
from collections import deque
import tkinter as tk
from tkinter import filedialog, messagebox
import tkinter.font as tkFont
from PIL import Image, ImageTk
import heapq

class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return False

    def __hash__(self):
        return hash((self.x, self.y))

class MazeSolverApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Maze Solver")
        self.create_widgets()

        self.mouse_click_status = 0  # 0 for start, 1 for end
        self.start = Point()
        self.end = Point()
        self.directions = [Point(0, -1), Point(0, 1), Point(1, 0), Point(-1, 0)]
        self.rw = 10  # Size of the rectangle for start and end points
        self.img = None
        self.h, self.w = 0, 0
        self.algorithm = "BFS"  # Default algorithm

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

        solve_btn = tk.Button(maze_frame, text="Solve Maze", command=self.solve_maze, bg="#27ae60", fg="Black", font=button_font, padx=10, pady=5)
        solve_btn.grid(row=0, column=2, padx=5, pady=10)

        self.algorithm_var = tk.StringVar(value="BFS")
        tk.Radiobutton(main_frame, text="BFS", variable=self.algorithm_var, value="BFS", bg="#e8eef7").pack(anchor=tk.W)
        tk.Radiobutton(main_frame, text="DFS", variable=self.algorithm_var, value="DFS", bg="#e8eef7").pack(anchor=tk.W)
        tk.Radiobutton(main_frame, text="A*", variable=self.algorithm_var, value="A*", bg="#e8eef7").pack(anchor=tk.W)
        tk.Radiobutton(main_frame, text="Bidirectional", variable=self.algorithm_var, value="Bidirectional BFS", bg="#e8eef7").pack(anchor=tk.W)

        self.maze_canvas = tk.Canvas(main_frame, width=600, height=600, bg="white", bd=1, relief="solid")
        self.maze_canvas.pack(pady=20)

        restart_btn = tk.Button(self.root, text="Restart", command=self.restart, bg="#e74c3c", fg="white", font=button_font, padx=20, pady=10)
        restart_btn.pack(pady=5)

        self.note_label = tk.Label(self.root, text="Note: Image must be in PNG format with a white background.", bg="#e8eef7", font=("Arial", 10))
        self.note_label.pack(pady=10)

        self.maze_canvas.bind("<Button-1>", self.set_point)


    def select_maze(self):
        file_path = filedialog.askopenfilename()
        self.maze_path_entry.delete(0, tk.END)
        self.maze_path_entry.insert(0, file_path)
        self.load_image(file_path)

    def load_image(self, path):
        self.img = cv2.imread(path)
        if self.img is None:
            messagebox.showerror("Error", "Error loading image.")
            return
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.img = cv2.resize(self.img, (600, 600), interpolation=cv2.INTER_NEAREST)
        _, self.img = cv2.threshold(self.img, 120, 255, cv2.THRESH_BINARY)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        self.h, self.w = self.img.shape[:2]

        self.display_maze()

    def display_maze(self):
        img_rgb = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        img_tk = ImageTk.PhotoImage(img_pil)
        self.maze_canvas.create_image(0, 0, anchor=tk.NW, image=img_tk)
        self.maze_canvas.image = img_tk

    def set_point(self, event):
        if self.mouse_click_status == 0:  # Setting start point
            self.start = Point(event.x, event.y)
            cv2.rectangle(self.img, (event.x - self.rw, event.y - self.rw), (event.x + self.rw, event.y + self.rw), (255, 0, 0), -1)
            self.mouse_click_status = 1
            self.display_maze()
            messagebox.showinfo("Info", "Start point set. Click to set the end point.")
        elif self.mouse_click_status == 1:  # Setting end point
            self.end = Point(event.x, event.y)
            cv2.rectangle(self.img, (event.x - self.rw, event.y - self.rw), (event.x + self.rw, event.y + self.rw), (0, 255, 0), -1)
            self.mouse_click_status = 2
            self.display_maze()
            messagebox.showinfo("Info", "End point set. Ready to solve the maze.")

    def bfs(self, s, e):
        queue = deque([s])
        visited = np.zeros((self.h, self.w), dtype=bool)
        parent = np.full((self.h, self.w), None, dtype=object)

        visited[s.y, s.x] = True

        while queue:
            p = queue.popleft()

            for d in self.directions:
                cell = p + d

                if (0 <= cell.x < self.w and 0 <= cell.y < self.h and
                        not visited[cell.y, cell.x] and
                        not np.array_equal(self.img[cell.y, cell.x], [0, 0, 0])):

                    queue.append(cell)
                    visited[cell.y, cell.x] = True
                    parent[cell.y, cell.x] = p

                    if cell == e:
                        path = []
                        while p is not None:
                            path.append(p)
                            p = parent[p.y, p.x]
                        path.reverse()
                        for p in path:
                            cv2.circle(self.img, (p.x, p.y), 3, (0, 0, 255), -1)
                        return True
        return False

    def bidirectional_bfs(self, s, e):
        q_start = deque([s])
        q_end = deque([e])
        visited_start = np.zeros((self.h, self.w), dtype=bool)
        visited_end = np.zeros((self.h, self.w), dtype=bool)
        parent_start = np.full((self.h, self.w), None, dtype=object)
        parent_end = np.full((self.h, self.w), None, dtype=object)

        visited_start[s.y, s.x] = True
        visited_end[e.y, e.x] = True

        while q_start and q_end:
            p_start = q_start.popleft()
            for d in self.directions:
                cell = p_start + d
                if (0 <= cell.x < self.w and 0 <= cell.y < self.h and 
                    not visited_start[cell.y, cell.x] and 
                    not np.array_equal(self.img[cell.y, cell.x], [0, 0, 0])):

                    q_start.append(cell)
                    visited_start[cell.y, cell.x] = True
                    parent_start[cell.y, cell.x] = p_start

                    if visited_end[cell.y, cell.x]:
                        return self.reconstruct_path(cell, parent_start, parent_end)

            p_end = q_end.popleft()
            for d in self.directions:
                cell = p_end + d
                if (0 <= cell.x < self.w and 0 <= cell.y < self.h and 
                    not visited_end[cell.y, cell.x] and 
                    not np.array_equal(self.img[cell.y, cell.x], [0, 0, 0])):

                    q_end.append(cell)
                    visited_end[cell.y, cell.x] = True
                    parent_end[cell.y, cell.x] = p_end

                    if visited_start[cell.y, cell.x]:
                        return self.reconstruct_path(cell, parent_start, parent_end)

        return False

    def reconstruct_path(self, meeting_point, parent_start, parent_end):
        path = []
        # Reconstruct path from start to meeting point
        p = meeting_point
        while p is not None:
            path.append(p)
            p = parent_start[p.y, p.x]
        path.reverse()

        # Reconstruct path from meeting point to end
        p = parent_end[meeting_point.y, meeting_point.x]
        while p is not None:
            path.append(p)
            p = parent_end[p.y, p.x]

        # Mark the path on the maze image
        for p in path:
            cv2.circle(self.img, (p.x, p.y), 3, (0, 0, 255), -1)
        return True

    def a_star(self, s, e):
        open_set = []
        heapq.heappush(open_set, (0, s))
        came_from = np.full((self.h, self.w), None, dtype=object)
        g_score = np.full((self.h, self.w), np.inf)
        g_score[s.y, s.x] = 0
        f_score = np.full((self.h, self.w), np.inf)
        f_score[s.y, s.x] = self.heuristic(s, e)

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == e:
                self.reconstruct_path_astar(came_from, current)
                return True

            for d in self.directions:
                neighbor = current + d
                if (0 <= neighbor.x < self.w and 0 <= neighbor.y < self.h and 
                    not np.array_equal(self.img[neighbor.y, neighbor.x], [0, 0, 0])):

                    tentative_g_score = g_score[current.y, current.x] + 1
                    if tentative_g_score < g_score[neighbor.y, neighbor.x]:
                        came_from[neighbor.y, neighbor.x] = current
                        g_score[neighbor.y, neighbor.x] = tentative_g_score
                        f_score[neighbor.y, neighbor.x] = g_score[neighbor.y, neighbor.x] + self.heuristic(neighbor, e)
                        heapq.heappush(open_set, (f_score[neighbor.y, neighbor.x], neighbor))

        return False

    def reconstruct_path_astar(self, came_from, current):
        path = []
        while current is not None:
            path.append(current)
            current = came_from[current.y, current.x]
        path.reverse()

        for p in path:
            cv2.circle(self.img, (p.x, p.y), 3, (0, 0, 255), -1)

    def heuristic(self, p1, p2):
        return abs(p1.x - p2.x) + abs(p1.y - p2.y)

    def solve_maze(self):
        if self.mouse_click_status != 2:
            messagebox.showerror("Error", "Start and end points must be set.")
            return

        algorithm = self.algorithm_var.get()

        if algorithm == "BFS":
            solved = self.bfs(self.start, self.end)
        elif algorithm == "DFS":
            solved = self.dfs(self.start, self.end)
        elif algorithm == "A*":
            solved = self.a_star(self.start, self.end)
        elif algorithm == "Bidirectional BFS":
            solved = self.bidirectional_bfs(self.start, self.end)
        else:
            messagebox.showerror("Error", f"Unknown algorithm: {algorithm}")
            return

        if solved:
            self.display_maze()
            messagebox.showinfo("Success", "Maze solved!")
        else:
            messagebox.showerror("Failure", "No solution found.")

    def restart(self):
        if self.img is not None:
            self.load_image(self.maze_path_entry.get())
        self.mouse_click_status = 0
        messagebox.showinfo("Restart", "Restarted! Set start and end points again.")

    def dfs(self, s, e):
        stack = [s]
        visited = np.zeros((self.h, self.w), dtype=bool)
        parent = np.full((self.h, self.w), None, dtype=object)

        visited[s.y, s.x] = True

        while stack:
            p = stack.pop()

            for d in self.directions:
                cell = p + d

                if (0 <= cell.x < self.w and 0 <= cell.y < self.h and
                        not visited[cell.y, cell.x] and
                        not np.array_equal(self.img[cell.y, cell.x], [0, 0, 0])):

                    stack.append(cell)
                    visited[cell.y, cell.x] = True
                    parent[cell.y, cell.x] = p

                    if cell == e:
                        path = []
                        while p is not None:
                            path.append(p)
                            p = parent[p.y, p.x]
                        path.reverse()
                        for p in path:
                            cv2.circle(self.img, (p.x, p.y), 3, (0, 0, 255), -1)
                        return True
        return False

# Run the app
if __name__ == "__main__":
    root = tk.Tk()
    app = MazeSolverApp(root)
    root.mainloop()

