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
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    source_box = findBox(source_point)
    destination_box = findBox(destination_point)

    # Check if source and destination are in the same box
    if source_box == destination_box:
        print("Final Path:", [source_point, destination_point])
        return [source_point, destination_point], [source_box]

    if source_box in mesh['adj'].get(destination_box, []) or destination_box in mesh['adj'].get(source_box, []):
        # Create a direct connection between source and destination within the adjacent boxes
        print("Final Path:", [source_point, destination_point])
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
    intersection_point_forward = None
    intersection_point_backward = None
    next_box_forward = None 
    next_box_backward = None

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
                    intersection_point_forward = constrained_position_forward

                    # Check if the intersection point is on the line segment
                    if intersection_point_backward:
                        # Calculate the intersection point based on both searches
                        intersection_point = (
                            (intersection_point_forward[0] + intersection_point_backward[0]) / 2,
                            (intersection_point_forward[1] + intersection_point_backward[1]) / 2
                        )

                        # Check if the intersection point is within the boundaries of the intersected boxes
                        if inRectangle(next_box_forward, intersection_point) and inRectangle(next_box_backward, intersection_point):
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
                    intersection_point_backward = constrained_position_backward

                    # Check if the intersection point is on the line segment
                    if intersection_point_forward:
                        # Calculate the intersection point based on both searches
                        intersection_point = (
                            (intersection_point_forward[0] + intersection_point_backward[0]) / 2,
                            (intersection_point_forward[1] + intersection_point_backward[1]) / 2
                        )

                        # Check if the intersection point is within the boundaries of the intersected boxes
                        if inRectangle(next_box_forward, intersection_point) and inRectangle(next_box_backward, intersection_point):
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
        print(forward_path)
        print(backward_path)
        return path, list(forward_reached.union(backward_reached))
    else:
        print("No path!")
        return path, list(forward_reached.union(backward_reached))
