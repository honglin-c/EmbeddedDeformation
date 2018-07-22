## Embedded Deformation for Shape Manipulation

####1. Deformation Graph

node position: $g_j \in R^3, j \in 1\cdots m $ (random uniform sampling: use Meshlab first, then manually simplify the nodes to a suitable density)

neighbor: $N(j)$ consists of all nodes that share an edge with node $j$ 

edge in deformation graph: create an edge between any two nodes that influence the same vertex

rotation matrix: $R_j$  $3 \times 3$ matrix

translation matrix: $t_j$   $3 \times 1$ vector

Mapping of a point p influenced by nodes $g_j$: $\hat p = R_j(p-g_j)+g_j+t_j$

transformed position of vertex $v_j$: $\hat v_j=\sum^m_{j=1}w_j(v_j)[R_j(v_i-g_j)+g_j+t_j]$

transformed normal of vertex $v_j$:$\hat n_i = \sum^m_{j=1}w_j(v_i)R_j^{-1T}n_i$ (+ normalize)

weight: $w_j(v_i)= (1-\Vert v_i-g_j\Vert / d_{max})^2$ (limit the influence of the deformation graph on a particular vertex to the k-nearest nodes, and $d_{max}$ is the distance to k+1 nearest node, use k =4 here)

#### 2. Energy function

$$F(x) = f(x)^Tf(x) = w_{rot}E_{rot}+w_{reg}E_{reg}+w_{con}E_{con}$$

* m nodes in total
* subject to $R_q = I, t_q=0, \forall q \in fixed \space ids$
* use $w_{rot}=1,w_{reg}=10, w_{con}=100 $ here
* treat $q $ fixed constraint variables as constants, 12m-12q variable here (convert to unconstrainted problem)

######(1)Rotation $E_{rot}$

$Rot(R) = (c_1 \cdot c_2)^2 +(c_1 \cdot c_3)^2 +(c_2 \cdot c_3)^2 +(c_1 \cdot c_1 -1 )^2 +(c_2 \cdot c_2 -1 )^2+(c_3 \cdot c_3 -1 )^2$

$E_{rot}=\sum^m_{j=1}Rot(R_j)$

###### (2)Regularization $E_{reg}$

$E_{reg}=\sum^m_{j=1} \sum_{k \in N(j)} \alpha_{jk} \Vert R_j(g_k-g_j)+g_j+t_j-(g_k+t_k)\Vert_2^2$

- use $\alpha_{jk}=1.0 $ here

###### (3) Constraints $E_{con}$ 

(a)handle constraints     (b)fixed constraints(subject to $R = I, t = 0$)

$E_{con} = \sum_{l=1}^p\Vert \hat v_{index(l)} - q_l \Vert_2^2$

Vertex$ \hat v_{index(l)}$ is deformed by the deformation graph according to
Eq. 2. The vector $q_l$ is the user-specified position of constraint l, and index(l) is the index of the constrained vertex.

####3.Gauss-Newton Method (with Cholesky factorization)

Energy function:$F(x) = f(x)^Tf(x) = w_{rot}E_{rot}+w_{reg}E_{reg}+w_{con}E_{con}$

$f(x+\Delta x) = f(x) + J\Delta x$

$\delta_k=argmin_{\Delta x}\Vert f(x_k)+J\Delta x \Vert^2_2$

Gauss-Newton Method:

$$F(x+\Delta x) = \Vert f(x+\Delta x)\Vert_2^2 \approx  \Vert f(x) + J_f\Delta x \Vert^2_2 =f(x)^Tf(x) + 2f(x)^T J_f \Delta x + \Delta x^T J_f^T J_f \Delta x =F(x) + J_F \Delta x + \frac{1}{2} \Delta x^T J_f^T J_f \Delta x  $$

$$F(x+\Delta x)=F(x) + J_F \Delta x + \frac{1}{2} \Delta x^T H_F \Delta x​$$

$\Rightarrow H_F \approx2J_f^TJ_f$

$\Rightarrow F'(x+\Delta x)=F'(x) + F''(x)\Delta x + O(\Vert \Delta x \Vert^2) \approx J_F + H_F \Delta x = 0 $(station point)

$\Rightarrow H_F \Delta x= -J_F  $

$\Rightarrow J_f^TJ_f \Delta x  = -J_f f(x)$

Solve by Cholesky factorization 

Reason(Why not conjugate gradient): 

1. $J_f^TJ_f$ is small (n is small), so O($n^3$) is relatively small.
2. Direct method is more accurate than iterative method.
3. $J_f$'s non-zero structure remains unchanged, so reuse a fill-reducing permutation and symbolic fatorization (eigen SimplicialLDLT : analyzePattern(), compute()).

$$f(x)=\begin{bmatrix}(c_1\cdot c_2)_1 \\ (c_2 \cdot c_3)_1 \\ (c_2 \cdot c_3)_1\\ (c_1 \cdot c_1 -1)_1 \\ (c_2 \cdot c_2 -1)_1 \\ (c_3 \cdot c_3 -1)_1 \\ \vdots \\(c_1\cdot c_2)_m \\ (c_2 \cdot c_3)_m \\ (c_2 \cdot c_3)_m\\ (c_1 \cdot c_1 -1)_m \\ (c_2 \cdot c_2 -1)_m \\ (c_3 \cdot c_3 -1)_m \\ \vdots \\ \sqrt{10}( R_{j11}(g_{k1}-g_{j1})+R_{j12}(g_{k2}-g_{j2})+R_{j13}(g_{k3}-g_{j3})+g_{j1}+t_{j1}-(g_{k1}+t_{k1}))_{j=1,k=n_1}  \\ \sqrt{10}( R_{j21}(g_{k1}-g_{j1})+R_{j22}(g_{k2}-g_{j2})+R_{j23}(g_{k3}-g_{j3})+g_{j2}+t_{j2}-(g_{k2}+t_{k2}))_{j=1,k=n_1} \\ \sqrt{10}( R_{j31}(g_{k1}-g_{j1})+R_{j32}(g_{k2}-g_{j2})+R_{j33}(g_{k3}-g_{j3})+g_{j3}+t_{j3}-(g_{k3}+t_{k3}))_{j=n,k=n_1} \\  \vdots \\ \sqrt{10}( R_{j31}(g_{k1}-g_{j1})+R_{j32}(g_{k2}-g_{j2})+R_{j33}(g_{k3}-g_{j3})+g_{j3}+t_{j3}-(g_{k3}+t_{k3}))_{j=n,k=n_n} \\ \vdots \\ 10\sum_{j\in N(index(1)) } w_j(v_i)[(R_{j11}(v_{i1}-g_{j1})+g_{j1}+t_{j1})+[(R_{j12}(v_{i2}-g_{j2})+g_{j2}+t_{j2})+[(R_{j13}(v_{i3}-g_{j3})+g_{j3}+t_{j3})]-g_{l1}  \\10\sum_{j\in N((index(1)) } w_j(v_i)[(R_{j21}(v_{i1}-g_{j1})+g_{j1}+t_{j1})+[(R_{j22}(v_{i2}-g_{j2})+g_{j2}+t_{j2})+[(R_{j23}(v_{i3}-g_{j3})+g_{j3}+t_{j3})]-g_{l2} \\10\sum_{j\in N((index(1)) } w_j(v_i)[(R_{j31}(v_{i1}-g_{j1})+g_{j1}+t_{j1})+[(R_{j32}(v_{i2}-g_{j2})+g_{j2}+t_{j2})+[(R_{j33}(v_{i3}-g_{j3})+g_{j3}+t_{j3})]-g_{l3}   \\\vdots \\ \\10\sum_{j\in N((index(q)) } w_j(v_i)[(R_{j31}(v_{i1}-g_{j1})+g_{j1}+t_{j1})+[(R_{j32}(v_{i2}-g_{j2})+g_{j2}+t_{j2})+[(R_{j33}(v_{i3}-g_{j3})+g_{j3}+t_{j3})]-g_{l3}  \end{bmatrix}$$

$J_f = \begin{bmatrix}\frac{\partial f}{\partial R_{1-11}} & \frac{\partial f}{\partial R_{1-12}} & \cdots \frac{\partial f}{\partial R_{1-33}} & \frac{\partial f}{\partial t_{1-1}}& \frac{\partial f}{\partial t_{1-2}} & \frac{\partial f}{\partial t_{1-3}} &  \cdots & \frac{\partial f}{\partial t_{m-3}} \end{bmatrix}$

#### 4. Key points

1. Selection the graph nodes
   * Uniform Sampling
   * Distributing points densely over the surface, and repeatedly removing all points within a given radius of a randomly chosen one until a desired sampling density is reached.
   * In order to avoid small weights which makes $J_f$ not full-rank - influencing the constraint term -- Cholesky Decomposition fails because $J_f^TJ_f$ is not positive definite
2. Effectively calculate l2-norm in the Gauss-Newton method
   * e.g. Regularization term
   * Instead of $\Vert R_j(g_k-g_j)+g_j+t_j-(g_k+t_k)\Vert_2 $ in f(x)
   * Calculating  $\begin{bmatrix} R_{j11}(g_{k1}-g_{j1})+R_{j12}(g_{k2}-g_{j2})+R_{j13}(g_{k3}-g_{j3})+g_{j1}+t_{j1}-(g_{k1}+t_{k1})) \\  R_{j21}(g_{k1}-g_{j1})+R_{j22}(g_{k2}-g_{j2})+R_{j23}(g_{k3}-g_{j3})+g_{j2}+t_{j2}-(g_{k2}+t_{k2}) \\  R_{j31}(g_{k1}-g_{j1})+R_{j32}(g_{k2}-g_{j2})+R_{j33}(g_{k3}-g_{j3})+g_{j3}+t_{j3}-(g_{k3}+t_{k3}) \end{bmatrix}$ 
3. Calculation of constraints term
   * Instead of fixing the constraint term of non-constrained vertices to zero, only include constrained vertices in the constraints term.
   * Make sure that $J_f^TJ_f$ is positive definite (full rank)

#### 5. Final result







