#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np

#fichier = open("cable_command","r")
fichier = open("cable_states","r")
list = fichier.readlines()

count = 0
final = []
final2=[]

while count < (len(list)):
	if count%8==0:
		final.append(list[count-4].strip('\n'))  #-2=> tension -3=> vitesse -4=> position
		print(list[count-4].strip('\n'))
	count+=1

for i in final:
	l=i.split(",")
	for j in range(8):
		l[j]=l[j][1:]
	l[7]=l[7][:-1]
	final2.append(l)



Y1=[]
Y2=[]
Y3=[]

def complete(n):
	Y=[]
	for i in final2:
		Y.append(float(i[n]))
	return(Y)

X = np.linspace(0,20,len(final2))

fig, axs = plt.subplots(8)
fig.suptitle('Vertically stacked subplots')
for i in range(8):
	axs[i].plot(X,complete(i),label=str(i))
#plt.legend([1,2,3,4,5,6,7,8])
plt.show()