import sympy as sp

# 定義變數
x_1, x_2, x_3, x_4, s, t = sp.symbols('x_1 x_2 x_3 x_4 s t')

# 定義矩陣
A = sp.Matrix([
    [4, -5, 4, 3],
    [-1, 1, 3, 3],
    [3, -4, 7, 6],
    [2, -2, -6, -6]
])

X = sp.Matrix([x_1, x_2, x_3, x_4])

B = sp.Matrix([2, 4, 6, -8])

# 解方程
sol = sp.linsolve((A, B), x_1, x_2, x_3, x_4)

print(sol)