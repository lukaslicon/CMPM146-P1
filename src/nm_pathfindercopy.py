from math import sqrt
from queue import Queue
from heapq import heappop, heappush

def find_path(source_point, destination_point, mesh):
    path = []
    boxes = {}
    detail_points = {}
    line_segments = []

    def inRectangle(box, point):
        x1, x2, y1, y2 = box
        ax, ay = point
        return (ax > x1 and ax < x2 and ay > y1 and ay < y2)

    def findBox(point):
        for box in mesh['boxes']:
            if inRectangle(box, point) and box not in boxes:
                return box

    def euclidean(a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2) * 0.5

    source_box = findBox(source_point)
    destination_box = findBox(destination_point)

    # Initialize detail_points for source and destination
    detail_points[source_box] = source_point
    detail_points[destination_box] = destination_point

    q = []
    heappush(q, (0, source_box))

    reached = set()
    reached.add(source_box)

    distances = {}
    distances[source_box] = 0

    path_costs = {}
    path_costs[source_box] = 0

    backpointers = {}
    backpointers[source_box] = None

    while q:
        current_dist, current_box = heappop(q)
        if current_box == destination_box:
            break  # Reached the destination, exit the loop

        for next_box in mesh['adj'].get(current_box, []):
            if next_box not in reached:
                reached.add(next_box)

                # Copy the x, y position within the current box
                current_position = detail_points[current_box]

                x_range = [max(current_box[0], next_box[0]), min(current_box[1], next_box[1])]
                y_range = [max(current_box[2], next_box[2]), min(current_box[3], next_box[3])]

                a_cost = euclidean((x_range[0], y_range[0]), current_box) + euclidean(destination_point, (x_range[0], y_range[0]))
                b_cost = euclidean((x_range[1], y_range[1]), current_box) + euclidean(destination_point, (x_range[1], y_range[1]))

                if a_cost <= b_cost:
                    edge_cost = a_cost
                else:
                    edge_cost = b_cost

                # Constrain the position to the bounds of the destination box
                x1, x2, y1, y2 = next_box
                constrained_position = (
                    max(x1, min(x2, current_position[0])),
                    max(y1, min(y2, current_position[1]))
                )
                # Update detail_points for the next box
                detail_points[next_box] = constrained_position

                # Store line segment based on detail points
                line_segments.append((current_position, constrained_position))

                backpointers[next_box] = current_box
                distances[next_box] = edge_cost + current_dist
                heappush(q, (current_dist + edge_cost + euclidean(detail_points[next_box],destination_point), next_box))

    # Reconstruct a simplified path
    current_box = destination_box
    while current_box is not None:
        path.insert(0, detail_points[current_box])
        if current_box not in backpointers:
            path = []  # Reset the path
            break  # Exit the loop if current_box is not in backpointers
        current_box = backpointers[current_box]
    #final destination spot
    path.append(destination_point)
    if path:
        print("Path found:")
        print(path)
        return path, list(reached)
    else:
        print("No path!")
        return None, list(reached)
    

    
'''
from math import sqrt
from heapq import heappop, heappush

def find_path(source_point, destination_point, mesh):
    path = []
    boxes = {}
    detail_points = {}
    line_segments = []
    forward_prev = {}
    backward_prev = {}
    forward_dist = {}
    backward_dist = {}

    def inRectangle(box, point):
        x1, x2, y1, y2 = box
        ax, ay = point
        return (ax > x1 and ax < x2 and ay > y1 and ay < y2)

    def findBox(point):
        for box in mesh['boxes']:
            if inRectangle(box, point) and box not in boxes:
                return box

    def euclidean(a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2) * 0.5

    source_box = findBox(source_point)
    destination_box = findBox(destination_point)

    # Initialize detail_points for source and destination
    detail_points[source_box] = source_point
    detail_points[destination_box] = destination_point

    q = []
    heappush(q, (0, source_box, 'forward'))
    heappush(q, (0, destination_box, 'backward'))
    
    reached = set()
    reached.add(source_box)

    forward_prev[source_box] = None
    backward_prev[destination_box] = None
    forward_dist[source_box] = 0
    backward_dist[destination_box] = 0

    reached_goal = None  # Store the goal when destination is reached
    while q:
        current_priority, current_box, current_goal = heappop(q)

        print("Current Box:", current_box, "Goal:", current_goal)
        print("Reached:", reached)

        # Check if either forward or backward goal is reached
        if current_box == destination_box and (current_goal == 'forward' or current_goal == 'backward'):
            # Store the goal when destination is reached
            reached_goal = current_goal
            break  # Reached the destination, exit the loop

        # Recover the true distance from the distance tables
        true_distance = forward_dist.get(current_box, 0) if current_goal == 'forward' else backward_dist.get(current_box, 0)

        for next_box in mesh['adj'].get(current_box, []):
            if current_goal == 'forward':
                prev_table = forward_prev
                dist_table = forward_dist
            else:
                prev_table = backward_prev
                dist_table = backward_dist

            if next_box not in reached:
                reached.add(next_box)

                # Copy the x, y position within the current box
                current_position = detail_points[current_box]

                # Constrain the position to the bounds of the destination box
                x1, x2, y1, y2 = next_box
                constrained_position = (
                    max(x1, min(x2, current_position[0])),
                    max(y1, min(y2, current_position[1]))
                )
                # Update detail_points for the next box
                detail_points[next_box] = current_position  # Use current_position, not constrained_position

                # Store line segment based on detail points
                if current_goal == 'forward':
                    line_segments.append((current_position, constrained_position))
                else:
                    line_segments.insert(0, (constrained_position, current_position))

                # Calculate Euclidean distance to destination for the new detail point
                remaining_distance = euclidean(constrained_position, destination_point)

                # Adjust the remaining distance based on the search direction
                if current_goal == 'forward':
                    remaining_distance_forward = remaining_distance
                    remaining_distance_backward = euclidean(constrained_position, source_point)
                else:
                    remaining_distance_forward = euclidean(constrained_position, destination_point)
                    remaining_distance_backward = remaining_distance

                # Augment the priority with the adjusted remaining distance estimate
                new_priority = true_distance + euclidean(current_position, constrained_position) + remaining_distance_forward + remaining_distance_backward
                print("Next Box:", next_box, "Priority:", new_priority)

                prev_table[next_box] = current_box
                heappush(q, (new_priority, next_box, current_goal))

                # Update Distance Table
                dist_table[next_box] = true_distance + euclidean(current_position, constrained_position)

    # Reconstruct a simplified path using the stored goal
    current_box = destination_box
    while current_box is not None:
        path.insert(0, detail_points[current_box])
        print("Intermediate Point:", detail_points[current_box])
        if reached_goal == 'forward':
            current_box = forward_prev[current_box]
        else:
            current_box = backward_prev[current_box]

    print("Reached Goal:", reached_goal)
    print("Final Path:", path)
    # Return only the path and the list of reached points
    if path:
        print("Path found:", path)
        return path, list(reached)
    else:
        print("No path!")
        return None, list(reached)

'''