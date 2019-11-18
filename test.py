import numpy as np
np.random.seed(1)
n = 10
mu = np.abs(np.random.randn(n, 1))
Sigma = np.random.randn(n, n)
Sigma = Sigma.T.dot(Sigma)
# Long only portfolio optimization.
import cvxpy as cp


w = cp.Variable(n)
gamma = cp.Parameter(True)
ret = mu.T*w
risk = cp.quad_form(w, Sigma)
print(risk)
prob = cp.Problem(cp.Maximize(ret - gamma*risk),
               [cp.sum(w) == 1,
                w >= 0])
