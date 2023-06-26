import matplotlib.pyplot as plt


def leer_archivo(nombre_archivo):
    with open(nombre_archivo, 'r') as archivo:
        lineas = archivo.readlines()
    x = []
    y = []
    for linea in lineas:
        valores = linea.strip().split(',')
        x.append(float(valores[0]))
        y.append(float(valores[1]))
    return x, y


trayectoria1 = leer_archivo(
    '/home/govidal/code/robotica-movil/workspace/src/lab1_pkg/scripts/data_1_c.txt')
trayectoria2 = leer_archivo(
    '/home/govidal/code/robotica-movil/workspace/src/lab1_pkg/scripts/data_2_c.txt')
trayectoria3 = leer_archivo(
    '/home/govidal/code/robotica-movil/workspace/src/lab1_pkg/scripts/data_3_c.txt')
trayectoria4 = leer_archivo(
    '/home/govidal/code/robotica-movil/workspace/src/lab1_pkg/scripts/data_4_c.txt')
trayectoria5 = leer_archivo(
    '/home/govidal/code/robotica-movil/workspace/src/lab1_pkg/scripts/data_5_c.txt')

trayectorias = [trayectoria1, trayectoria2,
                trayectoria3, trayectoria4, trayectoria5]
colores = ['red', 'blue', 'green', 'orange', 'purple']
puntos = [(0, 0), (1, 0), (1, 1), (0, 1)]

plt.figure(figsize=(8, 8))
plt.xlim(-1, 3)
plt.ylim(-1, 3)
for i, trayectoria in enumerate(trayectorias):
    plt.plot(trayectoria[0], trayectoria[1],
             color=colores[i], label=f'Trayectoria {i}')
for punto in puntos:
    plt.scatter(punto[0], punto[1], color='black', s=50)
plt.legend()
plt.show()
