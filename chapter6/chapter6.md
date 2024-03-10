# Notes

## SLAM 过程

$$
\begin{cases}
\bold{x}_k = f(\bold{x}_{k-1}, \bold{u}_k, \bold{w}_k), & k = 1, \cdots, K \\
\bold{z}_{k,j} = h(\bold{y}_j, \bold{x}_k, \bold{v}_{k,j}), & (k,j) \in \mathcal{O}
\end{cases}
$$
我们用 $k$ 表示时刻，$x_k,y_j$分别是k时刻小车的位姿和所见路标点， $z_k$ 表示某时刻观测到的路标点，$u_k$ 表示某时刻的运动输入，$w,v$ 是高斯噪声。\
我们所做的是一个状态估计问题：由于噪声的存在，我们不能确定物体的准确位姿，那么所建的图自然就不够准确。我们希望通过带噪声的$z,u$ 反推出最大可能的$x, y$ \
利用贝叶斯，可以得到
$$
P(x,y|z,u) = \frac{P(z,u|x,y)P(x,y)}{P(z,u)}
$$
在先验 $P(x,y)$ 之下，我们要得到MLE $(x,y)^*_{MLE} = argmax(P(z,u|x,y)) $，即“在什么样的情况下，最可能产生现在观测到的数据。”

## Concepts

## Install Ceres

After `sudo make install`, files are in `/usr/local/lib` and  `/usr/local/include`

## Install g2o

Files are in `/usr/local/bin`, `/usr/local/include` and `/usr/local/lib`\
Notice that there are quite a lot of examples in `/usr/local/include/g2o/examples`.

## Questions

1. 为什么SLAM的状态估计是一个最小二乘问题？
