import matplotlib.pyplot as plt


def leer_archivo(nombre_archivo):
    with open(nombre_archivo, 'r') as archivo:
        lineas = archivo.readlines()
    x = []
    y = []
    z = []
    for linea in lineas[0:700]:
        valores = linea.strip().split(',')
        x.append(float(valores[0]))
        y.append(float(valores[1]))
        z.append(float(valores[2]))
    return x, y, z


trayectoria1 = leer_archivo(
    'workspace/src/lab2_pkg/data/PI/linx_4.txt')
trayectoria2 = leer_archivo(
    'workspace/src/lab2_pkg/data/PI/liny_4.txt')

plt.plot(trayectoria1[0], color='r', label='ref_x')
plt.plot(trayectoria1[1], color='g', label='vel_x')
plt.plot(trayectoria1[2], color='b', label='real_x')
plt.plot(trayectoria2[0], color='black', label='ref_y')
plt.plot(trayectoria2[1], color='y', label='vel_y')
plt.plot(trayectoria2[2], color='m', label='real_y')


plt.legend()
plt.show()
