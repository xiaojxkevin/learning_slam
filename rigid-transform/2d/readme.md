# Explanations For 2d Folder

Let's consider a 2D case where an agent moves on a certain plane. As a result, the degree of freedom in this case is 3, i.e. we only needs $[x,y,\theta]$.

And easily, we could find out the homogeneous coordinate system of the transformation matrix would be
$$T = \begin{bmatrix}
\cos( \theta ) & -\sin( \theta ) & x\\
\sin( \theta ) & \cos( \theta ) & y\\
0 & 0 & 1
\end{bmatrix}\in SE(2) $$

However, the corresponding Lie-algebra for this is not $[x,y,\theta]$ (in fact it would be $[J[x,y],\theta]$)

What I want to explain is that you should be extremely careful when doing optimization for you should notice which kind variable it is!
