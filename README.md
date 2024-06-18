# P1: Navmesh Pathfinding

## Introduction

In this programming assignment, you will implement a bidirectional A* search algorithm in Python to solve the problem of finding paths in navmeshes created from user-provided images. The provided codebase includes programs to build navmeshes from images and a template for the function you need to implement.

## Getting Started

### Prerequisites

Ensure you have the following Python libraries installed:

- numpy
- scipy
- matplotlib
- pickle

You can install them using pip if they are not already installed:

```bash
pip install numpy scipy matplotlib pickle
```

### File Structure

The provided base code is organized as follows:

```
/input
    homer.png
    homer.png.mesh.pickle
/src
    nm_interactive.py
    nm_meshbuilder.py
    nm_pathfinder.py
```

### Running the Example

To test your pathfinder, execute the following command from the `/src` folder:

```bash
python nm_interactive.py ../input/homer.png ../input/homer.png.mesh.pickle
```

This command will display an image where you can interactively define the source and destination points. The program will then use your A* bidirectional algorithm to find and display the path between these points.

## Implementing the Pathfinder

### Function to Implement

Your main task is to implement the `find_path` function in `nm_pathfinder.py`. This function should use a bidirectional A* search algorithm to find a path from a source to a destination point in a navmesh.

### Suggested Workflow

1. **Identify the source and destination boxes.**
    - Scan through the list of boxes to find which contain the source and destination points.

2. **Implement a simple search algorithm.**
    - Start with a breadth-first search to find a sequence of boxes from the source to the destination.

3. **Enhance the search to compute a list of line segments demonstrating the path.**
    - Use Euclidean distances between points within the boxes to determine the path length.

4. **Modify Dijkstra’s implementation into an A* implementation.**
    - Adjust the priority queue to use an estimate of the remaining distance to the destination.

5. **Implement a bidirectional A* search.**
    - Use two sets of distance and previous pointers for the forward and backward searches.
    - Terminate the search when the two search frontiers meet.

### Requirements

- Implement a function to compute a path using a bidirectional A* algorithm.
- Test your implementation with a new navmesh created from an image you provide.

### Grading Criteria

- Correct identification of the source and destination boxes.
- Proper handling of cases where:
    - No path exists (reported via console message).
    - The path is degenerate (a straight line between start and destination).
    - The path is between adjacent or closely spaced cells.
- Legal and connected path visualization.
- Correct implementation of the A* and bidirectional search algorithms.

## Submission Instructions

Submit a zip file named `Lastname-Firstname-P1.zip` containing:

- `nm_pathfinder.py` with the `find_path` function implemented.
- An image file named `test_image.png` used for testing.
- A mesh representation of the image named `test_image.mesh.pickle`.

Include your name and your partner’s name (if applicable) in the comment field of the submission form.

## Creating a Custom Map

1. **Find or create a black-and-white image.**
    - Save the image as a PNG file.

2. **Generate a navmesh:**
    - Run the navmesh builder:
      ```bash
      python nm_meshbuilder.py your_image.png
      ```

3. **Test your pathfinding program:**
    - Execute the following command:
      ```bash
      python nm_interactive.py your_image.png your_image.png.mesh.pickle 1
      ```

## Example Commands

```bash
# Building a navmesh
python nm_meshbuilder.py ucsc_banana_slug.png

# Running the pathfinding visualization
python nm_interactive.py ucsc_banana_slug-orig.png ucsc_banana_slug.png.mesh.pickle 1
```
