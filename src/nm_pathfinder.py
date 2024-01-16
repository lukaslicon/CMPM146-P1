
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
from math import sqrt
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

    # Check if source and destination are in the same box
    if source_box == destination_box:
        return [source_point, destination_point], [source_box]

    if source_box in mesh['adj'].get(destination_box, []) or destination_box in mesh['adj'].get(source_box, []):
        # Create a direct connection between source and destination within the adjacent boxes
        return [source_point, destination_point], [source_box, destination_box]

    # Initialize detail_points for source and destination
    detail_points[source_box] = source_point
    detail_points[destination_box] = destination_point

    # Initialize forward search
    forward_q = []
    heappush(forward_q, (0, source_box))
    forward_reached = set()
    forward_reached.add(source_box)
    forward_backpointers = {}
    forward_backpointers[source_box] = None
    forward_distance_table = {}

    # Initialize backward search
    backward_q = []
    heappush(backward_q, (0, destination_box))
    backward_reached = set()
    backward_reached.add(destination_box)
    backward_backpointers = {}
    backward_backpointers[destination_box] = None
    backward_distance_table = {}

    intersection_box = None

    while forward_q and backward_q:
        # Forward search
        current_priority_forward, current_box_forward = heappop(forward_q)
        true_distance_forward = forward_distance_table.get(current_box_forward, 0)

        for next_box_forward in mesh['adj'].get(current_box_forward, []):
            if next_box_forward not in forward_reached:
                forward_reached.add(next_box_forward)

                current_position_forward = detail_points[current_box_forward]

                x1, x2, y1, y2 = next_box_forward
                constrained_position_forward = (
                    max(x1, min(x2, current_position_forward[0])),
                    max(y1, min(y2, current_position_forward[1]))
                )

                detail_points[next_box_forward] = constrained_position_forward
                line_segments.append((current_position_forward, constrained_position_forward))

                remaining_distance_forward = euclidean(constrained_position_forward, destination_point)

                new_priority_forward = true_distance_forward + euclidean(current_position_forward, constrained_position_forward) + remaining_distance_forward

                forward_backpointers[next_box_forward] = current_box_forward
                heappush(forward_q, (new_priority_forward, next_box_forward))

                forward_distance_table[next_box_forward] = true_distance_forward + euclidean(current_position_forward, constrained_position_forward)

                # Check for intersection between forward and backward searches
                if next_box_forward in backward_reached:
                    intersection_box = next_box_forward
                    break

        if intersection_box:
            break

        # Backward search
        current_priority_backward, current_box_backward = heappop(backward_q)
        true_distance_backward = backward_distance_table.get(current_box_backward, 0)

        for next_box_backward in mesh['adj'].get(current_box_backward, []):
            if next_box_backward not in backward_reached:
                backward_reached.add(next_box_backward)

                current_position_backward = detail_points[current_box_backward]

                x1, x2, y1, y2 = next_box_backward
                constrained_position_backward = (
                    max(x1, min(x2, current_position_backward[0])),
                    max(y1, min(y2, current_position_backward[1]))
                )

                detail_points[next_box_backward] = constrained_position_backward
                line_segments.append((current_position_backward, constrained_position_backward))

                remaining_distance_backward = euclidean(constrained_position_backward, source_point)

                new_priority_backward = true_distance_backward + euclidean(current_position_backward, constrained_position_backward) + remaining_distance_backward

                backward_backpointers[next_box_backward] = current_box_backward
                heappush(backward_q, (new_priority_backward, next_box_backward))

                backward_distance_table[next_box_backward] = true_distance_backward + euclidean(current_position_backward, constrained_position_backward)

                # Check for intersection between forward and backward searches
                if next_box_backward in forward_reached:
                    intersection_box = next_box_backward
                    break

        if intersection_box:
            break

    # Reconstruct path
    if intersection_box:
        forward_path = []
        backward_path = []

        current_box = intersection_box
        while current_box is not None:
            forward_path.insert(0, detail_points[current_box])
            if current_box not in forward_backpointers:
                break
            current_box = forward_backpointers[current_box]

        current_box = intersection_box
        while current_box is not None:
            backward_path.append(detail_points[current_box])
            if current_box not in backward_backpointers:
                break
            current_box = backward_backpointers[current_box]

        # Combine forward and backward paths
        path = forward_path + backward_path[1:]

    if path:
        print("Final Path:", path)
        # union returns a set that contains all items from both sets
        return path, list(forward_reached.union(backward_reached))
    else:
        print("No path!")
        return path, list(forward_reached.union(backward_reached))
