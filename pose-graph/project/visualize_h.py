import numpy as np
import matplotlib.pyplot as plt

# Parse the file and store the data
data = []
with open("./pose-graph/project/information_matrix.txt", "r") as file:
    for line in file:
        row, col, value = map(float, line.split())
        data.append((int(row), int(col), value))

# Extract dimensions of the matrix
rows = max(row for row, _, _ in data) + 1
cols = max(col for _, col, _ in data) + 1

# Create a matrix with zeros
matrix = np.zeros((rows, cols))

# Fill the matrix with the provided data
for row, col, value in data:
    matrix[row, col] = value

# Create a colormap with red for positive values, blue for negative values, and white for zero
cmap = plt.cm.get_cmap("coolwarm")
norm = plt.Normalize(vmin=-1, vmax=1)

# Plot the matrix
plt.figure(figsize=(8, 8))
plt.imshow(matrix, cmap=cmap, norm=norm)
plt.colorbar()

# Add annotations to show non-zero values
for i in range(rows):
    for j in range(cols):
        value = matrix[i, j]
        if value != 0:
            color = 'blue' if value < 0 else 'red'
            plt.text(j, i, f'{value:.2f}', ha='center', va='center', color=color)

plt.title("Sparse Matrix Visualization")
plt.xlabel("Columns")
plt.ylabel("Rows")
plt.grid(visible=False)
plt.show()
