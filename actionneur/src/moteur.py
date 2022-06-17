#!/usr/bin/python3

import numpy as np

Ke=0
Kc=0
a=0
R=0
J=0
L=0
E0=0
w0=np.sqrt((Ke*Kc+a*R)/J*L)
Xi=w0/2*((R*J+a*L)/Ke*Kc+a*R)
K=Kc(Ke*Kc+a*R)
t=0
U=0

S=K*E0*(1-1/2*np.sqrt(Xi**2-1)*(np.exp((w0/Xi)-np.sqrt(Xi**2-1)*t)/(Xi-np.sqrt(Xi**2-1))-np.exp(w0/Xi+np.sqrt(Xi**2-1)*t)/(Xi+np.sqrt(Xi**2-1))))*U

