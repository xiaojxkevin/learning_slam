# Pose Graph 1

## Notations 

*please ignore this if you haven't seen contents below*

-   $x_{i} =[ t_{x} ,t_{y} ,t_{z} ,\theta_{x} ,\theta_{y} ,\theta_{z}]^{T} \in \mathbb{R}^6$
    to be a state, and $T_{i}$ is the corresponding transformation
    function in $SE(3)$, where

$$T_{i} \ =\ \begin{bmatrix}
R_{i} & t_{i}\\
0 & 1
\end{bmatrix}$$

-   *t2v()* is a function that maps a state to its corresponding
    transformation matrix; and *v2t()* is exactly the inverse function.

-   $\displaystyle x\ =\ \left[ x_{1}^{T} ,\ ...,\ x_{n}^{T}\right]$ to
    be the state vector of $\displaystyle n$ states.

-   $\displaystyle Z_{ij} \ \in \ SE( 3)$ to be the measured
    transformation matrix from scan $\displaystyle i$ to scan
    $\displaystyle j$ (like the reslult of using ICP to match two
    scans). And we define $\displaystyle z_{ij} \ =\ t2v( Z_{ij})$.

$$Z_{ij} \ =\ \begin{bmatrix}
R_{ij} & t_{ij}\\
0 & 1
\end{bmatrix}$$

-   Define $\displaystyle \tilde{Z}_{ij} \ =\ T_{i}^{-1} T_{j}$ to be
    the relative pose via two states, and
    $\displaystyle \tilde{z}_{ij} \ =\ t2v\left(\tilde{Z}_{ij}\right)$.

-   Define $\displaystyle e_{ij}( x) \ =\ \tilde{z}_{ij} -z_{ij}$ to be
    gap between the relative pose given by two states and the measured
    relative pose. (A number of blogs online do not define this way, but
    I think this would be a more efficient way for it has a simplier
    form of Jacobian).

## General idea

Since we use a *graph* to optimize things, it is clear that we have to
define two most important things in any kind of graphs: vertex and edge.

-   A vertex is a state, a random variable in probability, which needs
    to be optimized. In pose graph, it would be a state of pose
    $\displaystyle x_{i}$.

-   An edge is used to constraint two verteces. In pose graph, it could
    be a relative pose given by ICP matching. Notice that we do not
    optimize edges.

A nice metaphor is that consider all verteces are objectes connected by
spring (edges), and at the beginning the system is loose and stable.
Then we introduce some more spring to the system, and this makes almost
all spring to be active (store a large amount of energy). The goal of
optimization is to minimize this energy to make the system stable again.

## Define Loss

Our goal is to minimize all $\displaystyle e_{ij}( x)$. Assume that
$\displaystyle e_{ij}( x) \ \sim \ \mathcal{N}( 0,\ \Omega _{ij})$, and
all states are i.i.d., then it is equivalent to find the MLE:
$$G( e_{ij}( x)) \ =\ \prod _{i,j}\frac{1}{( 2\pi )^{3} |\Omega _{ij} |^{1/2}} \ \exp\left( -\frac{1}{2} e_{ij}^{T}( x) \Omega _{ij} e_{ij}( x)\right)$$

Notice that
$$\ln( G( e_{ij}( x))) \ =\ \sum \frac{1}{( 2\pi )^{3} |\Omega _{ij} |^{1/2}} \ -\ \frac{1}{2}\sum _{i,j} e_{ij}^{T}( x) \Omega _{ij} e_{ij}( x)$$
it would be the same to minimize
$$F( x) \ =\ \sum _{i,j} e_{ij}^{T}( x) \Omega _{ij} e_{ij}( x)$$ 

Of course, we can assume a distinct distribution, say Laplace distribution,
and then the loss function would be different as above. In addtion, in
order to make system robust to outliers, we can also apply Huber loss,
which is widely used in Deep Learning: 
$$Huber( e) \ =\ \begin{cases}
\frac{1}{2} e^{2} , & |e|\leq \delta \\
\delta \left( |e|\ -\ \frac{1}{2} \delta \right) , & \text{otherwise}
\end{cases}$$ 
To make things simple, we will assume Gaussian
distribution.

### Q: what is $\displaystyle \Omega _{ij}$ and how to initialize it?

A widely accepted answer is that $\displaystyle \Omega _{ij}$ accounts
for the uncertainty of the measurements, which is the inverse of the
covariance matrix. Recall that measurements are obtained by ICP
matching, thus we have a matched point set
$\displaystyle \{p_{i} ,q_{i}\}$ of two scans. And ICP algorithms give
us a minimum value of the matching error
$$E( e_{ij}( x)) \ =\ \sum ||p_{i} \ -t2v( z_{ij}) *q_{i} ||_{2}^{2} \ \ $$
then we can find $\displaystyle \Omega _{ij} \ =\ JJ^{T} \in \ M_{6}$, where
$$\displaystyle J\ =\ \frac{\partial E( e_{ij}( x))}{\partial e_{ij}( x)} \in \ \mathbb{R}^{6}$$
to be the Jacobian matrix.

Of course, we can set it to identity or experimental values, there are
more than one initialization.

## Main Process

For the rest of the part, I advise you to look at the paper [A Tutorial on Graph-Based SLAM](https://ieeexplore.ieee.org/document/5681215). 

The reason why I do not write one myself is that currently all my ideas are inherited from that paper, i.e. I have not created some new ideas. Then it would be a nice choice to just provide the original paper to readers.

But there's one thing that I want to mention, it is that the gradient should instead be 

$$ \begin{array}{l}
A_{ij} =  \frac{\partial e_{ij}( x)}{\partial x_{i}^{T}}  = \begin{bmatrix}
-R_{i}^{T} & \frac{\partial R_{i}^T}{\partial \theta _{i}}( t_{j} -t_{i})\\
0 & -1
\end{bmatrix}\\
B_{ij} = \frac{\partial e_{ij}( x)}{\partial x_{j}^{T}}  = \begin{bmatrix}
R_{i}^{T} & 0\\
0 & 1
\end{bmatrix}
\end{array} $$

since we have dropped $z_{ij}$.

## Related blogs

- [Project](https://github.com/xiaojxkevin/learning_slam/tree/main/pose-graph/project)
- [blog-csdn](https://blog.csdn.net/u010507357/article/details/108540110)
- [information-matrix](https://robotics.stackexchange.com/questions/22451/calculate-information-matrix-for-graph-slam)

