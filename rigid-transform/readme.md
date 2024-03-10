# Rigid Transformations

## Homogeneous Coordinates

$$\begin{bmatrix}
x\\
y\\
z
\end{bmatrix}  \rightarrow \begin{bmatrix}
x\\
y\\
z\\
1
\end{bmatrix}  \simeq  \begin{bmatrix}
wx\\
wy\\
wz\\
w
\end{bmatrix}$$
One thing to mention about this form of coordinates is that we often say that they are **equal up to scale** since it is meaningless when the fourth value is not 1. As result, please be careful whenever we use equalities.

## Standard Form

$$ T =
\begin{bmatrix}
R & \mathbf{t}\\
\mathbf{0}^{T} & 1
\end{bmatrix} \in SE(3)
$$

where $R \in SO(3)$

$$
SO(3) = \{R\in \mathbb{R}^{3\times 3}|RR^T = I, det(R) = 1 \}
$$

An interesting question would be that why there are two constraints for $SO(3)$? What's the use of the determinant? Well, it means that the handness of the column space of $R$ must be right handed.

## Rigid Transform

We all know that one rigid transformation can be written as
$$P_{i} = T_{ij}P_j, \quad P_{\{i,j\}} \in \mathbb{R}^4  $$

we should always pay attention to subscripts.

## Various Representations
