import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt 

func_innprd=lambda f,g: integrate.quad(lambda t: f(t)*g(t),-np.pi,np.pi)[0]
func_norm=lambda f: np.sqrt(func_innprd(f,f))

polynomial_degree=13
def orig_basis_vec(i): return lambda t: t**i
orig_bis=[orig_basis_vec(i) for i in range(polynomial_degree+1)]
orth_bis={}
ips={}

def compute_inner_prods(k=5,degree=5):
    for i in range(k,degree+1):
        xi=orig_bis[i]
        ek=orth_bis[k]
        ips[(i,k)]=func_innprd(xi,ek)
    return

def gram_schmidt(k=5,degree=5):
    fk=lambda t: orig_bis[k](t)-np.sum([ips[(k,i)] * orth_bis[i](t) for i in range(k)])
    nfk=func_norm(fk)
    ek=lambda t: (1/nfk) * fk(t)
    orth_bis[k]=ek
    compute_inner_prods(k=k,degree=degree)
    return ek

for i in range(polynomial_degree+1): gram_schmidt(k=i,degree=polynomial_degree)

def compute_PUv_coeffs(v,degree):
    return [func_innprd(v,orth_bis[i]) for i in range(degree)]

def PUv(t, PUv_coefficients):
    return np.sum(PUv_coefficients[i]*orth_bis[i](t) for i in range(len(PUv_coefficients)))

def graph(funct, x_range, cl='r--'):
    y_range=[]
    for x in x_range:
        y_range.append(funct(x))
    plt.plot(x_range,y_range,cl)
    return

rs=1.0
r=np.linspace(-rs*np.pi,rs*np.pi,80)
v=lambda t: 15*np.sin(t)*np.power(np.cos(t),3)*np.exp(1/(t-19))
#v=lambda t: 15*np.sin(t**3)*np.power(np.cos(t**2),3)*np.exp(1/(t-19))
PUv_coeffs = compute_PUv_coeffs(v, polynomial_degree+1)
graph(lambda t: PUv(t,PUv_coeffs),r,cl='r-')
graph(v,r,cl='b--')
plt.axis('equal')
plt.show()
