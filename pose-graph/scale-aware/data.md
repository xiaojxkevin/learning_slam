# edges.txt
edge_nums

id0, id1, x, y, z, qx, qy, qz, qw, s

# nodes.txt
node_nums

id, x, y, z, qx, qy, qz, qw, s

# GT.txt
id, x, y, z, qx, qy, qz, qw, s

## ps:
id is the identity of each poses

x, y, z, qx, qy, qz, qw in edges.txt means the relative transformation from id0 to id1.

x, y, z, qx, qy, qz, qw in nodes.txt and GT.txt shows the transformation from world frame to camera frame (be careful about this).

s represents the scale, s=-1 means the scale is unknow, which indicates there is a scale jump.
