from math import inf, sqrt
from queue import Queue

def find_path(source_point, destination_point, mesh):
    path = []
    boxes = {}

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

    print(f"Source Box: {source_box}")
    print(f"Destination Box: {destination_box}")

    q = Queue()
    q.put(source_box)

    reached = set()
    reached.add(source_box)

    backpointers = {}
    backpointers[source_box] = None

    while not q.empty():
        current_box = q.get()

        if current_box == destination_box:
            break  # Reached the destination, exit the loop

        for next_box in mesh['adj'].get(current_box, []):
            if next_box not in reached:
                reached.add(next_box)
                backpointers[next_box] = current_box
                q.put(next_box)

    # Reconstruct the path
    current_box = destination_box
    while current_box is not None:
        path.insert(0, current_box)
        if current_box not in backpointers:
            path = []  # Reset the path
            break  # Exit the loop if current_box is not in backpointers
        current_box = backpointers[current_box]

    if path:
        print("Path found:")
        print(path)
        print("Boxes explored:")
        print(list(reached))
        return path, list(reached)
    else:
        print("No path!")
        return None, list(reached)  # Return a tuple indicating no path
