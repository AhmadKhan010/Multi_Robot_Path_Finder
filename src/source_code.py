import heapq
import random


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal, agents, current_time):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)

            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue

            if grid[neighbor[0]][neighbor[1]] == 'X':
                continue

            # Get agent positions at the corresponding time step
            timestamp = g_score[current] + current_time  
            agent_positions = agents_at_time(agents, timestamp)

            if neighbor in agent_positions:
                continue

            tentative_g_score = g_score[current] + 1

            # Update scores if a better path is found
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return [] 


def agents_at_time(agents, time):
    """Return a list of agent positions at the given timestamp, handling cyclic movement."""
    positions = []
    for agent_data in agents.values():
        if not agent_data:
            continue

        max_time = max(t for t, _ in agent_data)
        cycle_length = 2 * max_time

        # Calculate the effective time in the cycle
        effective_time = time % cycle_length
        if effective_time <= max_time:
            # Forward movement
            for t, position in agent_data:
                if t == effective_time:
                    positions.append(position)
                    break
        else:
            # Backward movement
            backward_time = cycle_length - effective_time
            for t, position in reversed(agent_data):
                if t == backward_time:
                    positions.append(position)
                    break

    return positions

# Move robots and avoid collisions with dynamic obstacles
def move_robot(grid, robot, agents, robot_positions, current_time):
    start, end = robot["start"], robot["end"]
    
    # if the start position of robot is an obstacle X, then it should randomly check the position in it's coordinates and select it as starting position
    if grid[start[0]][start[1]] == 'X':
        print(f"Starting position {start} for Robot {robot['id']} is an obstacle.")
        valid_positions = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            adj_pos = (start[0] + dx, start[1] + dy)
            if (0 <= adj_pos[0] < len(grid)) and (0 <= adj_pos[1] < len(grid[0])) and grid[adj_pos[0]][adj_pos[1]] == ' ':
                valid_positions.append(adj_pos)
        
        if valid_positions:
            start = random.choice(valid_positions)
            robot["start"] = start
            print(f"Robot {robot['id']} has chosen new starting position {start}.")
        else:
            print(f"No valid adjacent positions for Robot {robot['id']} to start. Unable to proceed.")
            return []
    path = astar(grid, start, end, agents, current_time)

    if not path:
        print(f"Robot starting at {start} has no valid path to {end}.")
        return []

    final_path = []
    for i, step in enumerate(path):
        timestamp = current_time + i
        agent_positions = agents_at_time(agents, timestamp)

        if step in agent_positions:
            print(f"Collision detected at {step} with an agent at time {timestamp}. Robot waiting...")
        # Add the current position to the path to reflect waiting
            step = final_path[-1] if final_path else robot["start"]  
            final_path.append(step)  
            continue  
        
        # Update robot's position on the grid
        if i > 0:
            prev_step = path[i - 1]
            grid[prev_step[0]][prev_step[1]] = ' '  

        grid[step[0]][step[1]] = "R" 
        robot_positions[robot["id"]] = step
        final_path.append(step)

    robot["start"] = end  
    return final_path

    # Check for robot collisions and resolve them
def resolve_robot_collisions(grid, robots, robot_paths, agents, current_time):
    max_path_length = max(len(path) for path in robot_paths.values())

    for t in range(max_path_length):
        positions_at_time = {}
        for robot_id, path in robot_paths.items():
            if t < len(path):
                position = path[t]
                if position in positions_at_time:
                    # Collision detected
                    other_robot_id = positions_at_time[position]
                    print(f"Collision detected between {robot_id} and {other_robot_id} at {position} at time {t}.")

                    # Roll back both robots to their previous positions
                    previous_position_robot = path[t - 1] if t > 0 else robots[robot_id]["start"]
                    previous_position_other = robot_paths[other_robot_id][t - 1] if t > 0 else robots[other_robot_id]["start"]

                    path[t] = previous_position_robot
                    robot_paths[other_robot_id][t] = previous_position_other

                    # Recalculate paths for both robots
                    new_path_robot = astar(grid, previous_position_robot, robots[robot_id]["end"], agents, current_time + t)
                    new_path_other = astar(grid, previous_position_other, robots[other_robot_id]["end"], agents, current_time + t)

                    robot_paths[robot_id] = path[:t + 1] + new_path_robot
                    robot_paths[other_robot_id] = robot_paths[other_robot_id][:t + 1] + new_path_other
                else:
                    positions_at_time[position] = robot_id


# Process and print the results
def process(grid, agents, robots):
    robot_paths = {}  # Track paths for each robot
    robot_positions = {robot_id: data["start"] for robot_id, data in robots.items()}

    for robot_id, robot in robots.items():
        path = move_robot(grid, robot, agents, robot_positions, current_time=0)
        robot_paths[robot_id] = path
    
    # Resolve robot collisions
    resolve_robot_collisions(grid, robots, robot_paths, agents, current_time=0)

    print("\nFinal Paths and Costs:")
    for robot_id, path in robot_paths.items():
        cost = len(path)
        print(f"{robot_id}: Path: {path}, Cost: {cost}")
        print("\n================================ ")


# Data extraction functions
def load_agents_data(file_path):
    """Reads and parses agents' data from the file."""
    try:
        with open(file_path, "r") as file:
            content = file.readlines()
    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
        return {}

    agents_data = {}
    for line in content:
        if not line.startswith("Agent"):
            continue

        try:
            # Split the line into parts
            parts = line.split(":")
            if len(parts) < 2:
                print(f"Skipping invalid line: {line.strip()}")
                continue

            agent_id = parts[0].strip()
            coords_part = line.split("[")[1].split("]")[0]
            coords_part = coords_part.replace("((", "").replace("))", "").replace("), (", "|")
            positions = [tuple(map(int, coord.split(", "))) for coord in coords_part.split("|")]

            times_part = line.split("at times [")[1].split("]")[0]
            times = list(map(int, times_part.split(", ")))

            # Ensure positions and times are correctly paired
            if len(positions) != len(times):
                print(f"Mismatch in the number of positions and times for {agent_id}. Skipping.")
                continue

            # Combine positions and times, then sort by time
            combined_data = sorted(zip(times, positions))
            agents_data[agent_id] = combined_data

        except Exception as e:
            print(f"Error processing line for agent: {line.strip()} - {e}")
            continue
    
    # Edit the below code so that after each agents data, there is a line space

    print("Parsed agents data:")
    for agent_id, data in agents_data.items():
        print(f"{agent_id}: {data}\n")
    print("==================================")
    return agents_data
          


def load_grid_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        grid_size = int(lines[0].strip())
        grid = [list(line.rstrip("\n")) for line in lines[1:]]
    
    # print the grid
#    print("Grid:")
#    for row in grid:
#        print(" ".join(row))
#    print("==================================")
    return grid_size, grid

def load_robots_data(file_path):
    robots_data = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            if line.startswith("Robot"):
                parts = line.split(":")
                robot_id = parts[0].strip()
                start_end = parts[1].split("End")
                start = tuple(map(int, start_end[0].replace("Start", "").strip()[1:-1].split(",")))
                end = tuple(map(int, start_end[1].strip()[1:-1].split(",")))
                robots_data[robot_id] = {"id": robot_id, "start": start, "end": end}
    
    print("Parsed robots data:")
    for robot_id, robot in robots_data.items():
        print(f"{robot_id}: {robot}")
    print("=================================")
    return robots_data

def load_example_data(example_number):
    agents_file = f"Agent{example_number}.txt"
    grid_file = f"data{example_number}.txt"
    robots_file = f"Robots{example_number}.txt"

    agents = load_agents_data(agents_file)
    grid_size, grid = load_grid_data(grid_file)
    robots = load_robots_data(robots_file)

    return agents, grid_size, grid, robots

# Example execution
if __name__ == "__main__":
    example_number = 0 
    agents, grid_size, grid, robots = load_example_data(example_number)
    process(grid, agents, robots)






