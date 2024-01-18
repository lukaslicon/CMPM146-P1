from math import sqrt
from heapq import heappop, heappush

def find_path(source_point, destination_point, mesh):
    path = []
    boxes = {}
    detail_points = {}

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

    print(f"Source Box: {source_box}, Destination Box: {destination_box}")

    # Check if source and destination are in the same box
    if source_box == destination_box:
        print("Final Path:", [source_point, destination_point])
        return [source_point, destination_point], [source_box]
    
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

    next_box_forward = None 
    next_box_backward = None
    intersection_box = None
    intersection_point_forward = None
    intersection_point_backward = None

    while forward_q and backward_q:
        # Forward search
        current_priority_forward, current_box_forward = heappop(forward_q)
        true_distance_forward = forward_distance_table.get(current_box_forward, 0)

        #print(f"\nForward Search: Current Box: {current_box_forward}, True Distance: {true_distance_forward}")

        for next_box_forward in mesh['adj'].get(current_box_forward, []):
            if next_box_forward not in forward_reached:
                forward_reached.add(next_box_forward)
                current_position_forward = detail_points[current_box_forward]
                
                #get constrained position for next_box_forward
                x1, x2, y1, y2 = next_box_forward
                constrained_position_forward = (
                    max(x1, min(x2, current_position_forward[0])),
                    max(y1, min(y2, current_position_forward[1]))
                )

                #apply detail points to forward queue
                detail_points[next_box_forward] = constrained_position_forward
                remaining_distance_forward = euclidean(constrained_position_forward, destination_point)
                new_priority_forward = true_distance_forward + euclidean(current_position_forward, constrained_position_forward) + remaining_distance_forward
                forward_backpointers[next_box_forward] = current_box_forward
                heappush(forward_q, (new_priority_forward, next_box_forward))
                forward_distance_table[next_box_forward] = true_distance_forward + euclidean(current_position_forward, constrained_position_forward)

                # Check for intersection between forward and backward searches
                if next_box_forward in backward_reached:
                    intersection_box = next_box_forward
                    intersection_point_forward = constrained_position_forward  # Update the forward intersection point

                    # Debug prints
                    #print(f"Forward intersection detected: {intersection_box}, {intersection_point_forward}")

                    # Check if the intersection point is within the boundaries of the intersected boxes
                    if inRectangle(intersection_box, intersection_point_forward):
                        # Break out of the intersection detection loop
                        break

        # Continue the search within the intersected boxes to find additional points
        if intersection_box:
            for next_box_forward in mesh['adj'].get(intersection_box, []):
                if next_box_forward not in forward_reached:
                    current_position_forward = detail_points[intersection_box]

                    #get constrained position for intersection
                    x1, x2, y1, y2 = next_box_forward
                    constrained_position_forward = (
                        max(x1, min(x2, current_position_forward[0])),
                        max(y1, min(y2, current_position_forward[1]))
                    )

                    #apply detail points to queue for intersections
                    detail_points[next_box_forward] = constrained_position_forward
                    remaining_distance_forward = euclidean(constrained_position_forward, destination_point)
                    new_priority_forward = true_distance_forward + euclidean(current_position_forward, constrained_position_forward) + remaining_distance_forward
                    forward_backpointers[next_box_forward] = intersection_box
                    heappush(forward_q, (new_priority_forward, next_box_forward))

                    forward_distance_table[next_box_forward] = true_distance_forward + euclidean(current_position_forward, constrained_position_forward)

                    # Debug prints
                    #print(f"Additional forward point found within intersected box: {next_box_forward}, {constrained_position_forward}")

                    # Check if the point is within the boundaries of the box
                    if inRectangle(next_box_forward, constrained_position_forward):
                        # Continue the search by adding the point to the queue
                        forward_reached.add(next_box_forward)
                        heappush(forward_q, (new_priority_forward, next_box_forward))

        if intersection_box:
            break

        # Backward search
        current_priority_backward, current_box_backward = heappop(backward_q)
        true_distance_backward = backward_distance_table.get(current_box_backward, 0)

        #print(f"\nBackward Search: Current Box: {current_box_backward}, True Distance: {true_distance_backward}")

        for next_box_backward in mesh['adj'].get(current_box_backward, []):
            if next_box_backward not in backward_reached:
                backward_reached.add(next_box_backward)
                current_position_backward = detail_points[current_box_backward]

                #get constrained position for next_box_backward
                x1, x2, y1, y2 = next_box_backward
                constrained_position_backward = (
                    max(x1, min(x2, current_position_backward[0])),
                    max(y1, min(y2, current_position_backward[1]))
                )

                #apply detail points to backward queue
                detail_points[next_box_backward] = constrained_position_backward
                remaining_distance_backward = euclidean(constrained_position_backward, source_point)
                new_priority_backward = true_distance_backward + euclidean(current_position_backward, constrained_position_backward) + remaining_distance_backward
                backward_backpointers[next_box_backward] = current_box_backward
                heappush(backward_q, (new_priority_backward, next_box_backward))
                backward_distance_table[next_box_backward] = true_distance_backward + euclidean(current_position_backward, constrained_position_backward)

                # Check for intersection between forward and backward searches
                if next_box_backward in forward_reached:
                    intersection_box = next_box_backward
                    intersection_point_backward = constrained_position_backward  # Update the backward intersection point

                    # Debug prints
                    #print(f"Backward intersection detected: {intersection_box}, {intersection_point_backward}")

                    # Check if the intersection point is within the boundaries of the intersected boxes
                    if inRectangle(intersection_box, intersection_point_backward):
                        # Break out of the intersection detection loop
                        break

        # Continue the search within the intersected boxes to find additional points
        if intersection_box:
            for next_box_backward in mesh['adj'].get(intersection_box, []):
                if next_box_backward not in backward_reached:
                    current_position_backward = detail_points[intersection_box]

                    #get constrained position for intersection
                    x1, x2, y1, y2 = next_box_backward
                    constrained_position_backward = (
                        max(x1, min(x2, current_position_backward[0])),
                        max(y1, min(y2, current_position_backward[1]))
                    )

                    #apply intersection detail points to queue
                    detail_points[next_box_backward] = constrained_position_backward
                    remaining_distance_backward = euclidean(constrained_position_backward, source_point)
                    new_priority_backward = true_distance_backward + euclidean(current_position_backward, constrained_position_backward) + remaining_distance_backward
                    backward_backpointers[next_box_backward] = intersection_box
                    heappush(backward_q, (new_priority_backward, next_box_backward))
                    backward_distance_table[next_box_backward] = true_distance_backward + euclidean(current_position_backward, constrained_position_backward)
                    # Debug prints
                    #print(f"Additional backward point found within intersected box: {next_box_backward}, {constrained_position_backward}")

                    # Check if the point is within the boundaries of the box
                    if inRectangle(next_box_backward, constrained_position_backward):
                        # Continue the search by adding the point to the queue
                        backward_reached.add(next_box_backward)
                        heappush(backward_q, (new_priority_backward, next_box_backward))

        if intersection_box:
            break

    # Reconstruct path
    if intersection_box:
        forward_path = []
        backward_path = []
        
        #forward path construction
        current_box = intersection_box
        while current_box is not None:
            forward_path.insert(0, detail_points[current_box])
            if current_box not in forward_backpointers:
                break
            current_box = forward_backpointers[current_box]

        #backwards path construction
        current_box = intersection_box
        while current_box is not None:
            backward_path.append(detail_points[current_box])
            if current_box not in backward_backpointers:
                break
            current_box = backward_backpointers[current_box]

        # Combine forward and backward paths
        path = forward_path + backward_path[1:]
        # Ensure both source and destination are included in the final path
        if source_point not in path:
            path.insert(0, source_point)
        if destination_point not in path:
            path.append(destination_point)

    if path:
        #Debug
        #print(f"backward intersection point: {intersection_point_backward}")
        #print(f"forward intersection point: {intersection_point_forward}")
        for point in path:
            box = findBox(point)
            if box is not None:
                constrained_point = (
                    max(box[0], min(box[1], point[0])),
                    max(box[2], min(box[3], point[1]))
                )
                #Debug
                #print(f"Source Point: {source_point}, Destination Point: {destination_point}\n")
                #print(f"Point: {point}, Constrained: {constrained_point}, Box: {box}\n")
        #debug
        #print(forward_path)
        #print(backward_path)
        #print(f"Final Path: {path}")
        #union returns a set that contains all items from both sets
        return path, list(forward_reached.union(backward_reached))
    else:
        print("No path!")
        # union returns a set that contains all items from both sets
        return path, list(forward_reached.union(backward_reached))