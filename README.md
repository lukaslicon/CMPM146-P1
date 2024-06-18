# Navmesh Pathfinding

This Python project implements a bidirectional A* search algorithm to find paths in navmeshes created from user-provided images. The program takes an image and its corresponding navmesh representation as input and outputs an image showing the path from a source to a destination point defined interactively.

To test the pathfinder, follow these steps:

1. Navigate to the `/src` folder.
2. Execute the following command:

```bash
$ python nm_interactive.py ../input/homer.png ../input/homer.png.mesh.pickle
```

Where:
- `../input/homer.png` is the image file to display (must be a PNG).
- `../input/homer.png.mesh.pickle` is the binary file containing the navmesh representation.
- The subsampling factor is an integer used to scale down large images for display.

After running the code, you can interactively define the source and destination points on the image.

## Creating a Custom Map

You can create your own test map by following these steps:

1. Find an image suitable for conversion into a black-and-white occupancy map (must be a PNG).
2. Create a black-and-white version of the image.
3. Run the navmesh builder program to create mesh data.
4. Run the pathfinding program with the original PNG file and the pickled mesh data.

## Note

Ensure you have SciPy installed for the mesh builder program to work.
