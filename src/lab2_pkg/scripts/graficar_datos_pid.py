import matplotlib.pyplot as plt


def leer_archivo(nombre_archivo):
    with open(nombre_archivo, 'r') as archivo:
        lineas = archivo.readlines()
    x = []
    y = []
    z = []
    for linea in lineas:
        valores = linea.strip().split(',')
        x.append(float(valores[0]))
        y.append(float(valores[1]))
        z.append(float(valores[2]))
    return x, y, z


trayectoria1 = leer_archivo(
    'workspace/src/lab2_pkg/data/PI/linx_1.txt')


plt.plot(trayectoria1[0], color='r', label='ref')
plt.plot(trayectoria1[1], color='g', label='vel')
plt.plot(trayectoria1[2], color='b', label='real')


plt.legend()
plt.show()
