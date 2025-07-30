import heapq
from collections import deque


class DronePathfinder:
    def __init__(self, grid):
        self.grid = grid
        self.rows, self.cols = len(grid), len(grid[0])
        self.start = self._find_pos('S')
        self.goal = self._find_pos('G')
        self.directions = [(-1,0), (1,0), (0,-1), (0,1)]
    
    def _find_pos(self, symbol):
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == symbol:
                    return (i, j)
    
    def _is_valid(self, row, col):
        return (0 <= row < self.rows and 0 <= col < self.cols and 
                self.grid[row][col] != 'X')
    
    def _get_neighbors(self, pos):
        row, col = pos
        return [(row+dr, col+dc) for dr, dc in self.directions 
                if self._is_valid(row+dr, col+dc)]
    
    def _manhattan_dist(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def _build_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
    
    def dfs(self):
        """Depth-First Search"""
        stack, visited, came_from, nodes = [self.start], set(), {}, 0
        
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            nodes += 1
            
            if current == self.goal:
                return self._build_path(came_from, current), nodes
            
            for neighbor in self._get_neighbors(current):
                if neighbor not in visited:
                    stack.append(neighbor)
                    if neighbor not in came_from:
                        came_from[neighbor] = current
        return None, nodes
    
    def bfs(self):
        """Breadth-First Search"""
        queue = deque([self.start])
        visited = {self.start}
        came_from, nodes = {}, 0
        
        while queue:
            current = queue.popleft()
            nodes += 1
            
            if current == self.goal:
                return self._build_path(came_from, current), nodes
            
            for neighbor in self._get_neighbors(current):
                if neighbor not in visited:
                    visited.add(neighbor)
                    came_from[neighbor] = current
                    queue.append(neighbor)
        return None, nodes
    
    def ucs(self):
        """Uniform Cost Search"""
        heap = [(0, self.start)]
        visited, came_from, cost, nodes = set(), {}, {self.start: 0}, 0
        
        while heap:
            curr_cost, current = heapq.heappop(heap)
            if current in visited:
                continue
            visited.add(current)
            nodes += 1
            
            if current == self.goal:
                return self._build_path(came_from, current), nodes
            
            for neighbor in self._get_neighbors(current):
                new_cost = curr_cost + 1
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(heap, (new_cost, neighbor))
        return None, nodes
    
    def a_star(self):
        """A* Search with Manhattan distance"""
        heap = [(0, self.start)]
        visited, came_from, g_score, nodes = set(), {}, {self.start: 0}, 0
        
        while heap:
            _, current = heapq.heappop(heap)
            if current in visited:
                continue
            visited.add(current)
            nodes += 1
            
            if current == self.goal:
                return self._build_path(came_from, current), nodes
            
            for neighbor in self._get_neighbors(current):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._manhattan_dist(neighbor, self.goal)
                    heapq.heappush(heap, (f_score, neighbor))
        return None, nodes
    
    def greedy(self):
        """Greedy Best-First Search"""
        heap = [(self._manhattan_dist(self.start, self.goal), self.start)]
        visited, came_from, nodes = set(), {}, 0
        
        while heap:
            _, current = heapq.heappop(heap)
            if current in visited:
                continue
            visited.add(current)
            nodes += 1
            
            if current == self.goal:
                return self._build_path(came_from, current), nodes
            
            for neighbor in self._get_neighbors(current):
                if neighbor not in visited:
                    came_from[neighbor] = current
                    h = self._manhattan_dist(neighbor, self.goal)
                    heapq.heappush(heap, (h, neighbor))
        return None, nodes


def validate_grid(grid):
    """Check grid is valid with proper S and G positions"""
    if not grid or not grid[0]:
        return False, "Empty grid"
    
    # Check equal row lengths and valid symbols
    cols = len(grid[0])
    s_count = g_count = 0
    
    for row in grid:
        if len(row) != cols:
            return False, "Unequal row lengths"
        for cell in row:
            if cell not in 'X.SG':
                return False, f"Invalid symbol: {cell}"
            if cell == 'S': s_count += 1
            elif cell == 'G': g_count += 1
    
    if s_count != 1: return False, "Need exactly one S"
    if g_count != 1: return False, "Need exactly one G"
    return True, "Valid"


def show_grid(grid, path=None):
    """Display grid with optional path"""
    display = [row[:] for row in grid]
    if path:
        for r, c in path:
            if display[r][c] not in 'SG':
                display[r][c] = '*'
    
    print("\nGrid:")
    for row in display:
        print(' '.join('■' if cell == 'X' else cell for cell in row))
    print("Legend: S=Start, G=Goal, ■=Obstacle, *=Path, .=Open")


def get_grid():
    """Get grid from user (preset or custom)"""
    presets = {
        1: [['S','.','X','.','G'], ['.','X','.','X','.'], ['.','.','.','.','.']],
        2: [['S','.','.','.'], ['.','X','X','.'], ['.','.','.','G']],
        3: [['S','.'], ['.','G']]
    }
    
    print("\n1. Preset grid  2. Custom grid")
    choice = int(input("Choose (1-2): "))
    
    if choice == 1:
        print("Presets: 1=Complex 2=Medium 3=Simple")
        preset = int(input("Choose (1-3): "))
        return presets.get(preset, presets[1])
    else:
        print("Enter rows (e.g. 'S.XG'), type 'done' to finish:")
        grid = []
        while True:
            row = input(f"Row {len(grid)+1}: ").strip().upper()
            if row.lower() == 'done':
                break
            if row:
                grid.append(list(row))
        return grid


def main():
    """Main program"""
    print("=== Drone Pathfinding System ===")
    algorithms = {1: ("DFS", "dfs"), 2: ("BFS", "bfs"), 3: ("UCS", "ucs"), 
                  4: ("A*", "a_star"), 5: ("Greedy", "greedy")}
    
    while True:
        try:
            # Get and validate grid
            grid = get_grid()
            valid, msg = validate_grid(grid)
            if not valid:
                print(f"Error: {msg}")
                continue
            
            show_grid(grid)
            
            # Choose algorithm
            print("\nAlgorithms:")
            for i, (name, _) in algorithms.items():
                print(f"{i}. {name}")
            
            algo_num = int(input("Choose (1-5): "))
            if algo_num not in algorithms:
                print("Invalid choice!")
                continue
            
            # Run pathfinding
            algo_name, method = algorithms[algo_num]
            pathfinder = DronePathfinder(grid)
            path, nodes = getattr(pathfinder, method)()
            
            # Show results
            print(f"\n=== {algo_name} Results ===")
            print(f"Nodes explored: {nodes}")
            if path:
                print(f"Path found! Length: {len(path)} steps")
                print(f"Route: {' → '.join(f'({r},{c})' for r,c in path)}")
                show_grid(grid, path)
            else:
                print("No path exists!")
            
        except (ValueError, KeyboardInterrupt):
            print("Invalid input!")
        except Exception as e:
            print(f"Error: {e}")
        
        if input("\nTry again? (y/n): ").lower() != 'y':
            break
    
    print("Goodbye!")


if __name__ == "__main__":
    main()
